#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Capture one X11 window to a PNG with XGetImage (xwd/import/scrot are not installed; PIL's root grab fails under WSLg). usage: xgrab.py <window id, 0x..> <out.png>"""
import ctypes, sys
from ctypes import c_int, c_uint, c_ulong, c_void_p, POINTER, Structure, c_char_p
from PIL import Image
x = ctypes.CDLL("libX11.so.6")
x.XOpenDisplay.restype = c_void_p; x.XOpenDisplay.argtypes = [c_char_p]
class Attr(Structure):
    _fields_ = [("x", c_int), ("y", c_int), ("width", c_int), ("height", c_int)] + [("pad", c_int * 40)]
x.XGetWindowAttributes.argtypes = [c_void_p, c_ulong, POINTER(Attr)]
class XImage(Structure):
    _fields_ = [("width", c_int), ("height", c_int), ("xoffset", c_int), ("format", c_int), ("data", c_void_p), ("byte_order", c_int),
                ("bitmap_unit", c_int), ("bitmap_bit_order", c_int), ("bitmap_pad", c_int), ("depth", c_int), ("bytes_per_line", c_int), ("bits_per_pixel", c_int)]
x.XGetImage.restype = POINTER(XImage); x.XGetImage.argtypes = [c_void_p, c_ulong, c_int, c_int, c_uint, c_uint, c_ulong, c_int]
d = x.XOpenDisplay(b":0"); assert d, "no display"
win = int(sys.argv[1], 0)
a = Attr(); x.XGetWindowAttributes(d, win, ctypes.byref(a))
img = x.XGetImage(d, win, 0, 0, a.width, a.height, 0xFFFFFFFF, 2)   # ZPixmap
assert img, "XGetImage failed"
n = img.contents.bytes_per_line * a.height
raw = ctypes.string_at(img.contents.data, n)
Image.frombuffer("RGBX", (a.width, a.height), raw, "raw", "BGRX", img.contents.bytes_per_line, 1).convert("RGB").save(sys.argv[2])
print("saved", sys.argv[2], a.width, a.height)
