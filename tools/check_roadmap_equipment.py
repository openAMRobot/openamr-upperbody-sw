#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compatibility entry point: compare the actual expanded URDF with roadmap JSON.

Raw differences remain failures; selected supplier deviations are documented in
export_docs_model_values.SOURCE_RECONCILIATIONS and tested separately.
"""
import argparse
from export_docs_model_values import main as export_main


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--datums')
    args = parser.parse_args()
    argv = ['--model-js', args.model]
    if args.datums:
        argv += ['--datums', args.datums]
    return export_main(argv)


if __name__ == '__main__':
    raise SystemExit(main())
