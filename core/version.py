#!/usr/bin/python3

# Copyright (C) 2021, Raspberry Pi (Trading) Limited
# Generate version information for libcamera-apps

import subprocess
import sys
from datetime import datetime

digits = 12

def generate_version():
    commit = '0' * digits

    # Check if this is a git directory
    r = subprocess.run('git rev-parse --git-dir',
                       shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode:
        print("VERSION ERR: Invalid git directory!", file=sys.stderr)

    # Get commit id
    r = subprocess.run(f'git describe --abbrev={digits} --always',
                       shell=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    if r.returncode:
        print("VERSION ERR: Invalid git commit!", file=sys.stderr)
    else:
        commit = r.stdout.decode("utf-8").strip('\n')

    # Check dirty status
    r = subprocess.run('git diff-index --quiet HEAD',
                       shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if r.returncode:
        commit = f'{commit}-dirty'

    print(f'{commit} {datetime.now().strftime("%d-%m-%Y (%H:%M:%S)")}', end="")


if __name__ == "__main__":
    generate_version()
