#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Writes the Build 1 world and its Nav2 map from config/scene.yaml.

    generate_world.py --profile fine|fast --output DIR [--scene scene.yaml] [--no-sensors]

build1.launch.py does the same at start. This tool is for looking at the files.
"""
import argparse
import os
import sys

from openamrobot_build1_scene import scene as scene_model


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["fine", "fast"], default="fine")
    parser.add_argument("--output", required=True)
    parser.add_argument("--scene", default=None)
    parser.add_argument("--no-sensors", action="store_true")
    args = parser.parse_args(argv)
    path = args.scene
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml")
    scene = scene_model.load(path)
    os.makedirs(args.output, exist_ok=True)
    world = os.path.join(args.output, f"build1_{args.profile}.sdf")
    with open(world, "w", encoding="utf-8") as f:
        f.write(scene_model.world_sdf(scene, args.profile, sensors=not args.no_sensors))
    print(world)
    print(scene_model.write_map(scene, args.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())
