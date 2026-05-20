#!/usr/bin/env python3
"""
Fetch the nrp-core API documentation zip (Doxygen + doxyrest output)
into the repository root as ``nrp-core-docs.zip``.

EBR2-50 rewrites this script to fetch from a stable URL instead of
the retired Nexus artifact repository. The Makefile ``nrp-core-unzip``
target depends on the resulting file and unpacks it into
``src/nrp-core/``.

Resolution order for the source URL (first non-empty wins):

  1. ``--url`` CLI argument.
  2. ``NRP_CORE_DOCS_URL`` environment variable.
  3. ``DEFAULT_URL`` constant below — set to the GitHub Release
     asset on ``vvorobjov/nrp-core`` that the nrp-core CI publishes.
     Adjust this string when the source-of-truth moves.

If a local ``nrp-core-docs.zip`` is already present, the script does
nothing unless ``--force`` is passed. This makes offline builds work
after a single successful fetch.
"""

from __future__ import annotations

import argparse
import os
import sys
import urllib.request
from pathlib import Path

# Adjust this when nrp-core revival publishes the artifact.
# Kept as an explicit URL (not a project alias) so a `grep` in this
# file finds the canonical pointer.
DEFAULT_URL = (
    "https://github.com/vvorobjov/nrp-core/releases/latest/download/"
    "nrp-core-docs.zip"
)

OUTFILE = Path(__file__).resolve().parent.parent / "nrp-core-docs.zip"


def resolve_url(cli_url):
    if cli_url:
        return cli_url
    env_url = os.environ.get("NRP_CORE_DOCS_URL", "").strip()
    if env_url:
        return env_url
    return DEFAULT_URL


def download(url, dest):
    print("[get-nrp-core-docs] fetching {} -> {}".format(url, dest))
    req = urllib.request.Request(url, headers={"User-Agent": "nrp-docs-fetch/1.0"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        if resp.status != 200:
            raise RuntimeError("HTTP {} fetching {}".format(resp.status, url))
        dest.write_bytes(resp.read())
    print("[get-nrp-core-docs] wrote {} bytes to {}".format(dest.stat().st_size, dest))


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--url", help="Override URL (otherwise NRP_CORE_DOCS_URL or DEFAULT_URL)")
    parser.add_argument("--force", action="store_true", help="Re-download even if the zip is already present")
    args = parser.parse_args(argv)

    if OUTFILE.exists() and not args.force:
        print("[get-nrp-core-docs] {} already present; skipping (pass --force to overwrite)".format(OUTFILE))
        return 0

    url = resolve_url(args.url)
    if not url:
        print("[get-nrp-core-docs] no URL configured; set NRP_CORE_DOCS_URL or pass --url", file=sys.stderr)
        return 2

    try:
        download(url, OUTFILE)
    except Exception as exc:
        print("[get-nrp-core-docs] download failed: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
