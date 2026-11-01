#!/usr/bin/env python3
"""mkovl.py NAME [--ws WS] [--desc-edit FILE OLD NEW]... [--moveit-edit FILE OLD NEW]... [--srdf-all-disabled]
Builds $S/ovl/NAME: an ament prefix whose share/ holds copies of the description (and moveit) share dirs with the edits applied."""
import argparse, os, shutil, subprocess, sys, itertools
S="${SCRATCH:-/tmp/m02_bisect}"
ap=argparse.ArgumentParser(); ap.add_argument("name"); ap.add_argument("--ws",default="/home/slash/openamr_ws")
ap.add_argument("--desc-edit",nargs=3,action="append",default=[]); ap.add_argument("--moveit-edit",nargs=3,action="append",default=[])
ap.add_argument("--srdf-all-disabled",action="store_true"); a=ap.parse_args()
root=f"{S}/ovl/{a.name}"; shutil.rmtree(root,ignore_errors=True)
def copy(pkg):
    dst=f"{root}/share/{pkg}"; os.makedirs(f"{root}/share/ament_index/resource_index/packages",exist_ok=True)
    open(f"{root}/share/ament_index/resource_index/packages/{pkg}","w").close()
    shutil.copytree(f"{a.ws}/install/{pkg}/share/{pkg}",dst,symlinks=False)
    if os.path.isdir(f"{a.ws}/install/{pkg}/lib"): os.symlink(f"{a.ws}/install/{pkg}/lib",f"{root}/lib") if not os.path.exists(f"{root}/lib") else None
    return dst
def edit(base,rel,old,new):
    p=f"{base}/{rel}"; s=open(p).read(); assert s.count(old)>=1,(rel,old); open(p,"w").write(s.replace(old,new))
if a.desc_edit:
    d=copy("openamrobot_upperbody_description")
    for rel,old,new in a.desc_edit: edit(d,rel,old,new)
if a.moveit_edit or a.srdf_all_disabled:
    m=copy("openamrobot_upperbody_moveit")
    for rel,old,new in a.moveit_edit: edit(m,rel,old,new)
    if a.srdf_all_disabled:
        import re, xacro, xml.etree.ElementTree as ET
        env=dict(os.environ)
        urdf=subprocess.run(["xacro",f"{a.ws}/install/openamrobot_upperbody_description/share/openamrobot_upperbody_description/urdf/combined_robot.xacro","hardware_profile:=mock"],capture_output=True,text=True,check=True).stdout
        links=sorted(l.get("name") for l in ET.fromstring(urdf).findall("link"))
        p=f"{m}/config/openamrobot.srdf"; s=open(p).read()
        s=re.sub(r"\s*<disable_collisions [^>]*/>","",s)
        pairs="\n".join(f'    <disable_collisions link1="{x}" link2="{y}" reason="Never"/>' for x,y in itertools.combinations(links,2))
        s=s.replace("</robot>",pairs+"\n</robot>"); open(p,"w").write(s)
        print("all-disabled pairs:",len(links)*(len(links)-1)//2)
print(root)
