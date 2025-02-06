#!/usr/bin/python3

# Copyright (C) 2021, Raspberry Pi (Trading) Limited
# Generate version information for rpicam-apps

import subprocess
import sys
from datetime import datetime
from string import hexdigits

digits = 12


def generate_version():
    try:
        if len(sys.argv) == 2:
            # Check if this is a git directory
            r = subprocess.run(['git', 'rev-parse', '--git-dir'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, universal_newlines=True)
            if r.returncode:
                raise RuntimeError('Invalid git directory!')

            # Get commit id
            r = subprocess.run(['git', 'rev-parse', '--verify', 'HEAD'],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True)
            if r.returncode:
                raise RuntimeError('Invalid git commit!')

            commit = r.stdout.strip('\n')[0:digits]

            # Check dirty status
            r = subprocess.run(['git', 'diff-index', '--quiet', 'HEAD'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, universal_newlines=True)
            if r.returncode:
                commit = commit + '-dirty'

        elif len(sys.argv) == 3:
            commit = sys.argv[2].lower().strip()
            if any(c not in hexdigits for c in commit):
                raise RuntimeError('Invalid git sha!')

            commit = commit[0:digits]

        else:
            raise RuntimeError('Invalid number of command line arguments')

    except RuntimeError as e:
        print(f'ERR: {e}', file=sys.stderr)
        commit = '0' * digits + '-invalid'

    finally:
        print(f'v{sys.argv[1]} {commit} {datetime.now().strftime("%d-%m-%Y (%H:%M:%S)")}', end="")


if __name__ == "__main__":
    generate_version()
