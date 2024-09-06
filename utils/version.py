#!/usr/bin/python3

# Copyright (C) 2021, Raspberry Pi (Trading) Limited
# Generate version information for rpicam-apps

import subprocess
import sys
from datetime import datetime
from string import hexdigits

digits = 12


def generate_version(git_sha):
    if git_sha is None:
        # Check if this is a git directory
        r = subprocess.run(['git', 'rev-parse', '--git-dir'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        if r.returncode:
            raise RuntimeError('Invalid git directory!')

        # Get commit id
        r = subprocess.run(['git', 'rev-parse', '--verify', 'HEAD'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        if r.returncode:
            raise RuntimeError('Invalid git commit!')

        commit = r.stdout.strip('\n')[0:digits] + '-intree'

        # Check dirty status
        r = subprocess.run(['git', 'diff-index', '--quiet', 'HEAD'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        if r.returncode:
            commit = commit + '-dirty'
    else:
        commit = git_sha.lower().strip()
        if any(c not in hexdigits for c in commit):
            raise RuntimeError('Invalid git sha!')

        commit = commit[0:digits]

    return commit


if __name__ == "__main__":
    try:
        rpicam_apps_ver = '0.0.0' if len(sys.argv) < 2 else sys.argv[1]
        git_sha = None if len(sys.argv) < 3 else sys.argv[2]
        commit = generate_version(git_sha)

    except RuntimeError as e:
        print(f'ERR: {e}', file=sys.stderr)
        commit = '0' * digits + '-invalid'

    finally:
        print(f'v{rpicam_apps_ver}-{commit} {datetime.now().strftime("%d-%m-%Y (%H:%M:%S)")}', end="")
