# SPDX-License-Identifier: MIT
"""Bundle version from a model release tag: model-2.0.0-rc.1 gives 2.0.0-rc.1, model-2.0.0 gives 2.0.0.

The decided tag names (17 Nov 2026) are model-2.0.0-rc.1, then model-2.0.0, then model-2.0.<n> for model fixes.
Any other tag name is refused, so a mistyped tag never produces a bundle. No ROS, no third-party imports."""
import re
import sys

TAG_RE = re.compile(r"model-([0-9]+\.[0-9]+\.[0-9]+(?:-rc\.[0-9]+)?)")


def version_from_tag(tag):
    match = TAG_RE.fullmatch(tag)
    if not match:
        raise ValueError(f"tag {tag!r} is not a model release tag (model-<major>.<minor>.<patch>[-rc.<n>])")
    return match.group(1)


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print("usage: release_tag.py <tag>", file=sys.stderr)
        return 2
    try:
        print(version_from_tag(args[0]))
    except ValueError as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
