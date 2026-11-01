#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Close this task's RViz through Qt before ROS launch invalidates its context.

WM_DELETE_WINDOW uses the ordinary application close path; XDestroyWindow and
kill are not substitutes. Only descendants of the tracked launch are eligible.
"""
import argparse
import ctypes as C
import ctypes.util
import json
import os
import signal
from pathlib import Path
import subprocess
import time


def live(pid):
    try:return Path(f'/proc/{pid}/stat').read_text().rpartition(')')[2].split()[0] not in ('Z','X')
    except FileNotFoundError:return False


def descendants(pid):
    parents={}
    for p in Path('/proc').iterdir():
        if p.name.isdigit():
            try:parents[int(p.name)]=int((p/'stat').read_text().rpartition(')')[2].split()[1])
            except (FileNotFoundError,IndexError):pass
    found=set();front={pid}
    while front:
        front={p for p,par in parents.items() if par in front and p not in found};found|=front
    return found


def send_close(window):
    class Data(C.Union):_fields_=[('b',C.c_char*20),('s',C.c_short*10),('l',C.c_long*5)]
    class Client(C.Structure):_fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',Data)]
    class Event(C.Union):_fields_=[('type',C.c_int),('client',Client),('pad',C.c_long*24)]
    x=C.CDLL(ctypes.util.find_library('X11'))
    x.XOpenDisplay.restype=C.c_void_p;x.XOpenDisplay.argtypes=[C.c_char_p]
    x.XInternAtom.restype=C.c_ulong;x.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int]
    x.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
    x.XFlush.argtypes=[C.c_void_p];x.XCloseDisplay.argtypes=[C.c_void_p]
    d=x.XOpenDisplay(None)
    if not d:raise RuntimeError('No task X display')
    try:
        ev=Event();ev.client.type=33;ev.client.display=d;ev.client.window=int(window);ev.client.message_type=x.XInternAtom(d,b'WM_PROTOCOLS',0);ev.client.format=32;ev.client.data.l[0]=x.XInternAtom(d,b'WM_DELETE_WINDOW',0)
        if not x.XSendEvent(d,int(window),0,0,C.byref(ev)):raise RuntimeError('XSendEvent rejected close')
        x.XFlush(d)
    finally:x.XCloseDisplay(d)


def close_pid(pid,timeout=10.):
    p=subprocess.run(['xdotool','search','--onlyvisible','--pid',str(pid)],capture_output=True,text=True)
    windows=p.stdout.splitlines() if p.returncode==0 else []
    if not windows and live(pid):raise RuntimeError(f'RViz {pid} has no visible window to close')
    started=time.monotonic()
    for window in windows:send_close(window)
    while live(pid) and time.monotonic()-started<timeout:time.sleep(.05)
    return {'pid':pid,'windows':windows,'method':'WM_DELETE_WINDOW before ROS context shutdown','elapsed_s':time.monotonic()-started,'exited':not live(pid)}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root-pid',type=int,required=True);parser.add_argument('--out',required=True);parser.add_argument('--stop-gazebo',action='store_true');a=parser.parse_args();records=[]
    try:
        if a.root_pid <= 1:
            raise ValueError('Refusing a system/init PID as the tracked launch')
        if live(a.root_pid):
            root = Path(f'/proc/{a.root_pid}')
            args = root.joinpath('cmdline').read_bytes().split(b'\0')
            if root.stat().st_uid != os.getuid() or b'launch' not in args or not any(os.path.basename(os.fsdecode(v)) == 'ros2' for v in args):
                raise ValueError('Tracked PID must be a ros2 launch owned by this user')
        for pid in sorted(descendants(a.root_pid)):
            try:args=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
            except FileNotFoundError:continue
            if args and os.path.basename(os.fsdecode(args[0]))=='rviz2' and live(pid):records.append(close_pid(pid))
        if a.stop_gazebo:
            # Ask the native GUI owner to exit once. Its Ruby parent then stops
            # and reaps the server, exits normally, and triggers launch shutdown.
            # Do this before signalling launch's /bin/sh wrapper (which otherwise
            # reports -SIGINT despite both native children exiting normally).
            gui=[]
            for pid in sorted(descendants(a.root_pid)):
                try:cmd=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').strip()
                except FileNotFoundError:continue
                if cmd==b'gz sim gui' and live(pid):gui.append(pid)
            for pid in gui:os.kill(pid,signal.SIGINT)
            if gui:
                started=time.monotonic()
                while live(a.root_pid) and time.monotonic()-started<8.:time.sleep(.05)
                records.append({'pids':gui,'method':'single native GUI SIGINT, launcher owns server shutdown','elapsed_s':time.monotonic()-started,'exited':not live(a.root_pid)})
    except Exception as exc:
        records.append({'error':str(exc),'exited':False})
    Path(a.out).write_text(json.dumps(records,indent=2)+'\n')
    print(json.dumps(records),flush=True)
    return int(any(not row['exited'] for row in records))

if __name__=='__main__':raise SystemExit(main())
