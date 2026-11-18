#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Writes the Nav2 map of the grocery dark store scene from config/grocery_scene.yaml.

    grocery_map.py [--scene config/grocery_scene.yaml] [--output config/maps]

-> grocery_dark_store.pgm and grocery_dark_store.yaml. The map is what the LiDAR plane (0.222 m above the floor, M-01)
cuts of the scene's fixed boxes: shelf posts, plinths, bench legs, walls and the dock backing (scene.occupancy_grid).
build1.launch.py generates the same map at start from the scene it loads; the committed copy is for reading and for
RViz without a launch, and test_grocery_scene.py checks that it equals a fresh generation. No ROS import.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import scene as scene_model  # noqa: E402

NAME = "grocery_dark_store"


def main(argv=None):
    package = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=os.path.join(package, "config", "grocery_scene.yaml"))
    parser.add_argument("--output", default=os.path.join(package, "config", "maps"))
    args = parser.parse_args(argv)
    print(scene_model.write_map(scene_model.load(args.scene), args.output, name=NAME))
    return 0


if __name__ == "__main__":
    sys.exit(main())
