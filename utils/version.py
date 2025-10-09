#!/usr/bin/python3

# Copyright (C) 2021, Raspberry Pi (Trading) Limited
# Generate version information for rpicam-apps

import os
import subprocess
import sys
import time

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

        else:
            raise RuntimeError('Invalid number of command line arguments')

        commit = f'v{sys.argv[1]} {commit}'

    except RuntimeError as e:
        commit = f'v{sys.argv[1]}'

    finally:
        date_str = time.strftime(
            "%d-%m-%Y (%H:%M:%S)",
            time.gmtime(int(os.environ.get('SOURCE_DATE_EPOCH', time.time())))
        )
        print(f'{commit} {date_str}', end="")


if __name__ == "__main__":
    generate_version()
