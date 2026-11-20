#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build the public OpenAMRobot 2.0 model bundle: expanded URDFs, the meshes they reference, a lock and a README.

    build_bundle.py --version 0.1.0 --out dist \
        --source openamr-upperbody-sw=<checkout> --source openamr-platform-sw=<checkout> \
        --source openarm_description=<checkout> [--pins <upstream_pins.yaml>] [--zip]

Needs a sourced ROS 2 workspace in which the packages of bundle.yaml are installed: the URDFs are expanded from the
installed openamrobot_upperbody_description, and each `package://<pkg>/<path>` mesh is copied from that package's
installed share directory to `share/<pkg>/<path>` in the bundle. Each source checkout gives the commit the lock records;
a checkout with local changes, or one whose commit is not its pin, stops the build. Nothing is guessed: a mesh that cannot
be found or that matches no licence rule stops the build too.

Writes <out>/<name>-<version>/ and, with --zip, <out>/<name>-<version>.zip (sorted entries, fixed timestamps, so the same
inputs give the same bytes). The directory is checked with verify_bundle.py before the zip is written.
"""
import argparse
import fnmatch
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import verify_bundle  # noqa: E402

VERSION_RE = re.compile(r"^[0-9A-Za-z][0-9A-Za-z.+-]{0,63}$")
MESH_RE = re.compile(r'filename="(package://[^"]+)"')
COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
LOCK = verify_bundle.LOCK
SUMS = verify_bundle.SUMS
ZIP_EPOCH = (1980, 1, 1, 0, 0, 0)


class BundleError(RuntimeError):
    pass


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_commit(path):
    """Full HEAD commit of a clean checkout. A dirty or missing checkout is an error, never a fallback."""
    head = subprocess.run(["git", "-C", path, "rev-parse", "HEAD"], capture_output=True, text=True)
    if head.returncode != 0:
        raise BundleError(f"{path}: not a git checkout ({head.stderr.strip()})")
    status = subprocess.run(["git", "-C", path, "status", "--porcelain", "--untracked-files=no"],
                            capture_output=True, text=True)
    if status.stdout.strip():
        raise BundleError(f"{path}: the checkout has local changes; the lock would not name what was built")
    return head.stdout.strip()


def ament_share(name):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(name)


def xacro_expand(xacro_file, args):
    import xacro
    return xacro.process_file(xacro_file, mappings=dict(args)).toxml()


def strip_comments(text, source_uri, commit):
    """The expanded URDF without its XML comments, with one header naming the source. The comments are the source's
    working notes (decision references, history); they stay in the repository at the locked commit, not in the model.
    Removing them also removes xacro's header, which names an absolute path on the build machine."""
    body = COMMENT_RE.sub("", text)
    decl, sep, rest = body.partition("?>")
    if not sep or not decl.startswith("<?xml"):
        raise BundleError("the expanded URDF does not start with an XML declaration")
    header = (f"\n<!-- Expanded from {source_uri} at openAMRobot/openamr-upperbody-sw {commit} by "
              f"tools/model_bundle/build_bundle.py. Source comments removed; read the source at that commit. -->")
    return decl + sep + header + "\n" + rest.lstrip("\n")


def count_children(text, tag):
    import xml.etree.ElementTree as ET
    return sum(1 for child in ET.fromstring(text.encode()) if child.tag == tag)


def licence_for(bundle_path, rules):
    for rule in rules:
        if fnmatch.fnmatch(bundle_path, rule["glob"]):
            return rule["licence"], rule.get("note", "")
    raise BundleError(f"{bundle_path}: no licence rule in bundle.yaml matches; add one, do not guess")


def mesh_uris(urdf_text):
    return sorted(set(MESH_RE.findall(urdf_text)))


def uri_to_bundle_path(uri):
    rest = uri[len("package://"):]
    pkg, _, rel = rest.partition("/")
    if not pkg or not rel or ".." in rel.split("/"):
        raise BundleError(f"unsupported mesh URI {uri}")
    return pkg, rel, f"share/{pkg}/{rel}"


def build(config, version, out_root, sources, resolve_share=ament_share, expand=xacro_expand, pins=None,
          environment=None, readme_template=None):
    """Write the bundle directory and return its path. `sources`: {source name: checkout path}."""
    if not VERSION_RE.match(version):
        raise BundleError(f"version {version!r}: letters, digits, '.', '+' and '-' only")
    environment = environment or {}
    name = f"{config['name']}-{version}"
    root = os.path.join(out_root, name)
    if os.path.exists(root):
        shutil.rmtree(root)
    os.makedirs(root)

    commits = {}
    for key, spec in config["sources"].items():
        if key not in sources:
            raise BundleError(f"no checkout given for source {key} (--source {key}=<path>)")
        commits[key] = git_commit(sources[key])
        pin_key = spec.get("pin")
        if pin_key and pins is not None:
            pinned = pins.get(pin_key)
            if pinned != commits[key]:
                raise BundleError(f"{key}: checkout is at {commits[key]}, the pin {pin_key} is {pinned}")

    package_source = {pkg: key for key, spec in config["sources"].items() for pkg in spec["packages"]}
    share_of = {pkg: resolve_share(pkg) for pkg in package_source}
    xacro_file = os.path.join(share_of["openamrobot_upperbody_description"], "urdf", "combined_robot.xacro")

    files = {}        # bundle path -> record (without hash)
    configurations = []
    for entry in config["configurations"]:
        urdf = strip_comments(expand(xacro_file, entry["args"]),
                              "package://openamrobot_upperbody_description/urdf/combined_robot.xacro",
                              commits["openamr-upperbody-sw"])
        rel = f"urdf/{entry['id']}.urdf"
        _write(root, rel, urdf)
        meshes = []
        for uri in mesh_uris(urdf):
            pkg, inner, bundle_path = uri_to_bundle_path(uri)
            if pkg not in share_of:
                raise BundleError(f"{entry['id']}: mesh {uri} is in package {pkg}, which bundle.yaml does not list")
            src = os.path.join(share_of[pkg], inner)
            if not os.path.isfile(src):
                raise BundleError(f"{entry['id']}: mesh {uri} not found at {src}")
            if bundle_path not in files:
                licence, note = licence_for(bundle_path, config["licence_rules"])
                dest = os.path.join(root, bundle_path)
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                shutil.copyfile(src, dest)
                files[bundle_path] = {"kind": "mesh", "licence": licence, "note": note,
                                      "source": {"repository": config["sources"][package_source[pkg]]["repository"],
                                                 "commit": commits[package_source[pkg]], "package": pkg, "path": inner}}
            meshes.append(bundle_path)
        links = count_children(urdf, "link")
        joints = count_children(urdf, "joint")
        files[rel] = {"kind": "urdf", "licence": config["urdf_licence"], "note": entry["use"],
                      "source": {"repository": config["sources"]["openamr-upperbody-sw"]["repository"],
                                 "commit": commits["openamr-upperbody-sw"], "package": "openamrobot_upperbody_description",
                                 "path": "urdf/combined_robot.xacro"}}
        configurations.append({"id": entry["id"], "use": entry["use"], "args": entry["args"], "urdf": rel,
                               "links": links, "joints": joints, "meshes": meshes})

    # ament index markers: with AMENT_PREFIX_PATH set to the bundle, package:// resolves without a build.
    for pkg in sorted({rec["source"]["package"] for rec in files.values() if rec["kind"] == "mesh"}):
        rel = f"share/ament_index/resource_index/packages/{pkg}"
        _write(root, rel, "")
        files[rel] = {"kind": "index", "licence": "MIT", "note": "empty ament index marker", "source": None}

    for lf in config["licence_files"]:
        src = os.path.join(sources[lf["source"]], lf["path"])
        if not os.path.isfile(src):
            raise BundleError(f"licence file {src} not found")
        dest = os.path.join(root, lf["as"])
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        shutil.copyfile(src, dest)
        files[lf["as"]] = {"kind": "licence", "licence": "", "note": f"copied from {lf['source']}",
                           "source": {"repository": config["sources"][lf["source"]]["repository"],
                                      "commit": commits[lf["source"]], "package": None, "path": lf["path"]}}

    mjcf = environment.get("mjcf") or {
        "included": False,
        "reason": "the MJCF generator (simulation/openamr_sim/mjcf.py) is not on main at this commit",
    }
    lock_head = {
        "schema_version": 1,
        "bundle": name,
        "version": version,
        "evidence_class": "static (xacro expansion and file integrity); simulation, not hardware validation",
        "generated_by": "tools/model_bundle/build_bundle.py",
        "sources": {k: {"repository": config["sources"][k]["repository"], "commit": commits[k]} for k in commits},
        "toolchain": environment.get("toolchain", {}),
        "configurations": configurations,
        "mjcf": mjcf,
    }

    readme = render_readme(readme_template or os.path.join(HERE, "README.bundle.md"), lock_head)
    _write(root, "README.md", readme)
    files["README.md"] = {"kind": "doc", "licence": "CC-BY-4.0", "note": "", "source": None}
    notice = render_notice(lock_head, files)
    _write(root, "NOTICE.md", notice)
    files["NOTICE.md"] = {"kind": "doc", "licence": "CC-BY-4.0", "note": "", "source": None}

    records = []
    for path in sorted(files):
        full = os.path.join(root, path)
        records.append({"path": path, "sha256": sha256_file(full), "bytes": os.path.getsize(full), **files[path]})
    lock = {**lock_head, "files": records}
    _write(root, LOCK, json.dumps(lock, indent=2, sort_keys=False) + "\n")
    sums = [f"{r['sha256']}  {r['path']}" for r in records] + [f"{sha256_file(os.path.join(root, LOCK))}  {LOCK}"]
    _write(root, SUMS, "\n".join(sorted(sums, key=lambda s: s.split("  ", 1)[1])) + "\n")

    problems = verify_bundle.verify_dir(root)
    if problems:
        raise BundleError("the bundle does not verify:\n  " + "\n  ".join(problems))
    return root


def render_readme(template_path, lock):
    with open(template_path) as f:
        text = f.read()
    rows = "\n".join(f"| `{c['urdf']}` | {c['use']} | " + ", ".join(f"`{k}:={v}`" for k, v in c["args"].items())
                     + f" | {c['links']} / {c['joints']} |" for c in lock["configurations"])
    srcs = "\n".join(f"| {k} | `{v['repository']}` | `{v['commit']}` |" for k, v in lock["sources"].items())
    mjcf = ("included" if lock["mjcf"]["included"] else "not included: " + lock["mjcf"]["reason"])
    first = lock["configurations"][0]["urdf"]
    gazebo = next((c["urdf"] for c in lock["configurations"] if "gazebo" in c["id"]), first)
    for key, value in {"{{BUNDLE}}": lock["bundle"], "{{VERSION}}": lock["version"], "{{CONFIG_ROWS}}": rows,
                       "{{SOURCE_ROWS}}": srcs, "{{MJCF}}": mjcf, "{{FIRST_URDF}}": first, "{{GAZEBO_URDF}}": gazebo}.items():
        text = text.replace(key, value)
    if "{{" in text:
        raise BundleError("README template has an unfilled placeholder")
    return text


def render_notice(lock, files):
    by_licence = {}
    for path, rec in sorted(files.items()):
        if rec["kind"] in ("mesh", "urdf"):
            by_licence.setdefault((rec["licence"], rec["note"] if rec["kind"] == "mesh" else "expanded model"), []).append(path)
    lines = [
        f"# Notice for {lock['bundle']}",
        "",
        "Original OpenAMRobot material: Copyright (c) 2021-2026 OpenAMRobot (Botshare LTD).",
        "Software and generated envelopes: MIT (`LICENSE`). Documentation (this file, `README.md`): CC-BY-4.0.",
        "CAD-derived robot geometry: CERN-OHL-P-2.0, https://spdx.org/licenses/CERN-OHL-P-2.0.html",
        "",
        "Third-party material keeps its own licence and notices. The OpenArm 2.0 description",
        "(`share/openarm_description/`) is Copyright 2025 Enactic, Inc., licensed Apache-2.0",
        "(`licenses/openarm_description-Apache-2.0.txt`); its files are copied unmodified from",
        f"`{lock['sources']['openarm_description']['repository']}` at `{lock['sources']['openarm_description']['commit']}`.",
        "",
        "Provenance of the mobile base meshes: `ros2/src/openamrobot_description/meshes/PROVENANCE.yaml` and `NOTICE.md` of",
        f"`{lock['sources']['openamr-platform-sw']['repository']}` at `{lock['sources']['openamr-platform-sw']['commit']}`.",
        "",
        "The names OpenAMRobot and OpenAMR and their logos are not licensed by any of these licences.",
        "",
        "## Files by licence",
        "",
    ]
    for (licence, note), paths in sorted(by_licence.items()):
        lines.append(f"### {licence} ({note})")
        lines.append("")
        lines += [f"- `{p}`" for p in paths]
        lines.append("")
    return "\n".join(lines)


def _write(root, rel, text):
    path = os.path.join(root, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="\n") as f:
        f.write(text)


def write_zip(root):
    """Deterministic zip of the bundle directory, entries under the bundle's name."""
    name = os.path.basename(root.rstrip(os.sep))
    path = root.rstrip(os.sep) + ".zip"
    entries = []
    for dirpath, _, filenames in os.walk(root):
        for fn in filenames:
            full = os.path.join(dirpath, fn)
            entries.append(os.path.relpath(full, root).replace(os.sep, "/"))
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for rel in sorted(entries):
            info = zipfile.ZipInfo(f"{name}/{rel}", date_time=ZIP_EPOCH)
            info.external_attr = 0o644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            with open(os.path.join(root, rel), "rb") as f:
                z.writestr(info, f.read())
    return path


def toolchain():
    info = {"ros_distro": os.environ.get("ROS_DISTRO", "unknown"), "python": sys.version.split()[0]}
    try:
        from importlib.metadata import version
        info["xacro"] = version("xacro")
    except Exception:  # noqa: BLE001 - a ROS install of xacro has no Python distribution metadata
        dpkg = subprocess.run(["dpkg-query", "-W", "-f=${Version}", f"ros-{info['ros_distro']}-xacro"],
                              capture_output=True, text=True)
        info["xacro"] = dpkg.stdout.strip() if dpkg.returncode == 0 and dpkg.stdout.strip() else "unknown"
    return info


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--version", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--config", default=os.path.join(HERE, "bundle.yaml"))
    parser.add_argument("--source", action="append", default=[], metavar="NAME=PATH")
    parser.add_argument("--pins", help="upstream_pins.yaml; each source with a `pin` must be at that commit")
    parser.add_argument("--zip", action="store_true")
    args = parser.parse_args(argv)
    with open(args.config) as f:
        config = yaml.safe_load(f)
    sources = dict(s.split("=", 1) for s in args.source)
    pins = None
    if args.pins:
        with open(args.pins) as f:
            pins = yaml.safe_load(f)["pins"]
    try:
        if os.path.exists(os.path.join(sources.get("openamr-upperbody-sw", ""), "simulation", "openamr_sim", "mjcf.py")):
            raise BundleError("simulation/openamr_sim/mjcf.py is on this commit: wire the MJCF generator into the bundle "
                              "(build_bundle.py, bundle.yaml, README.bundle.md) before publishing without it")
        root = build(config, args.version, args.out, sources, pins=pins, environment={"toolchain": toolchain()})
        print(f"bundle: {root}")
        if args.zip:
            print(f"zip: {write_zip(root)}")
    except BundleError as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
