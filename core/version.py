#!/usr/bin/python3

# Copyright (C) 2021, Raspberry Pi (Trading) Limited
# Generate version information for libcamera-apps

import subprocess
import sys
from datetime import datetime

digits = 12

def generate_version():
    commit = '0' * digits

    try:
        # Check if this is a git directory
        r = subprocess.run(['git', 'rev-parse', '--git-dir'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        if r.returncode:
            raise RuntimeError('Invalid git directory!')

        # Get commit id
        r = subprocess.run(['git', 'describe', f'--abbrev={digits}', '--always'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        if r.returncode:
            raise RuntimeError('Invalid git commit!')

        commit = r.stdout.strip('\n')

        # Check dirty status
        r = subprocess.run(['git', 'diff-index', '--quiet', 'HEAD'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True)
        if r.returncode:
            commit = commit + '-dirty'

    except RuntimeError as e:
        print(f'ERR: {e}', file=sys.stderr)

    finally:
        print(f'{commit} {datetime.now().strftime("%d-%m-%Y (%H:%M:%S)")}', end="")

if __name__ == "__main__":
    generate_version()
