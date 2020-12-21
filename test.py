#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2020, Raspberry Pi (Trading) Limited
#
# test.py - some automated testing for libcamera-apps

# These tests are very far from exhaustive, for which I apologise
# profusely, but still it's better than nothing. The rule going
# forward is that any new feature, or feature that needs a fix, must
# get a test in here.

import argparse
import os
import os.path
import subprocess
from timeit import default_timer as timer

class TestFailure(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

def check_exists(file, preamble):
    if not os.path.isfile(file):
        raise TestFailure(preamble + ": " + file + " not found")

def clean_dir(dir, exts = ('.jpg', '.png', '.bmp', '.dng', '.h264', '.mjpeg', '.raw', '.txt')):
    for file in os.listdir(dir):
        if file.endswith(exts):
            os.remove(os.path.join(dir, file))

def run_executable(args, logfile):
    start_time = timer()
    with open(logfile, 'w') as logfile:
        p = subprocess.Popen(args, stdout = logfile, stderr = subprocess.STDOUT)
        p.communicate()
    time_taken = timer() - start_time
    return p.returncode, time_taken

def check_retcode(retcode, preamble):
    if retcode:
        raise TestFailure(preamble + " failed, return code " + str(retcode))

def check_time(time_taken, low, high, preamble):
    if time_taken < low or time_taken > high:
        raise TestFailure(preamble + " failed, time taken " + str(time_taken) + " seconds")

def check_jpeg(file, preamble):
    # I haven't found a Python exif library that actually reads all the multiple image
    # tags (pyexiv2 does, but only runs on 64-bit systems... I mean, really??).
    # So we'll just use exiftool if it appears to be installed.
    try:
        p = subprocess.Popen(['exiftool', file], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        stdout = p.communicate()[0].decode('utf-8')
    except FileNotFoundError as e:
        print("WARNING:", preamble, "- exiftool not found")
        return
    if p.returncode:
        raise TestFailure(preamble + "- exiftool failed")
    if "Image Width" not in stdout:
        raise TestFailure(preamble + "- bad EXIF data")

def test_hello(dir):
    executable = os.path.join(dir, 'libcamera-hello')
    logfile = os.path.join(dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_hello')
    clean_dir(dir)

    # "run test". Just see if the executable appeared to run.
    print("    run test")
    retcode, time_taken = run_executable([executable, '-t', '2000'], logfile)
    check_retcode(retcode, "test_hello: run test")
    check_time(time_taken, 2, 5, "test_hello: run test")

    # "roi test". Specify an roi and see if it blows up.
    print("    roi test")
    retcode, time_taken = run_executable(
        [executable, '-t', '2000', '--roi', '0.25,0.25,0.5,0.5'], logfile)
    check_retcode(retcode, "test_hello: roi test")
    check_time(time_taken, 2, 5, "test_hello: roi test")

    # "controls test". Specify some image controls and see if it blows up.
    print("    controls test")
    retcode, time_taken = run_executable(
        [executable, '-t', '2000', '--brightness', '0.2', '--contrast', '1.2',
         '--saturation', '1.3', '--sharpness', '1.5'], logfile)
    check_retcode(retcode, "test_hello: controls test")
    check_time(time_taken, 2, 5, "test_hello: controls test")

    print("libcamera-hello tests passed")

def check_size(file, limit, presamble):
    if os.path.getsize(file) < limit:
        raise TestFailure(preamble + " failed, file " + file + " too small")

def test_still(dir):
    executable = os.path.join(dir, 'libcamera-still')
    output_jpg = os.path.join(dir, 'test.jpg')
    output_png = os.path.join(dir, 'test.png')
    output_bmp = os.path.join(dir, 'test.bmp')
    output_dng = os.path.join(dir, 'test.dng')
    logfile = os.path.join(dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_still')
    clean_dir(dir)

    # "jpg test". See if the executable appears to run and write an jpg output file.
    print("    jpg test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg], logfile)
    check_retcode(retcode, "test_still: jpg test")
    check_time(time_taken, 2, 8, "test_still: jpg test")
    check_size(output_jpg, 1024, "test_still: jpg test")

    # "png test". As above, but write a png.
    print("    png test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-e', 'png', '-o', output_png], logfile)
    check_retcode(retcode, "test_still: png test")
    check_time(time_taken, 3, 8, "test_still: png test")
    check_size(output_png, 1024, "test_still: png test")

    # "bmp test". As above, but write a bmp.
    print("    bmp test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-e', 'bmp', '-o', output_bmp], logfile)
    check_retcode(retcode, "test_still: bmp test")
    check_time(time_taken, 3, 8, "test_still: bmp test")
    check_size(output_png, 1024, "test_still: bmp test")

    # "dng test". Write a dng along with the jpg.
    print("    dng test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-o', output_jpg, '-r'], logfile)
    check_retcode(retcode, "test_still: dng test")
    check_time(time_taken, 2, 8, "test_still: dng test")
    check_size(output_jpg, 1024, "test_still: dng test")
    check_size(output_dng, 1024 * 1024, "test_still: dng test")

    # "timelapse test". Check that a timelapse sequence captures more than one jpg.
    print("    timelapse test")
    retcode, time_taken = run_executable(
        [executable, '-t', '10000', '--timelapse', '3500', '-o', os.path.join(dir, 'test%03d.jpg')],
        logfile)
    check_retcode(retcode, "test_still: timelapse test")
    check_time(time_taken, 9, 15, "test_still: timelapse test")
    check_size(os.path.join(dir, 'test000.jpg'), 1024, "test_still: timelapse test")
    check_size(os.path.join(dir, 'test001.jpg'), 1024, "test_still: timelapse test")
    if os.path.isfile(os.path.join(dir, 'test002.jpg')):
               raise("test_still: timelapse test, unexpected output file")

    print("libcamera-still tests passed")
    
def check_jpeg_shutter(file, shutter_string, iso_string, preamble):
    # Verify that the expected shutter_string and iso_string are in the exif.
    try:
        p = subprocess.Popen(['exiftool', file], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        stdout = p.communicate()[0].decode('utf-8').split('\n')
    except FileNotFoundError as e:
        print("WARNING:", preamble, "- exiftool not found")
        return
    shutter_line = [line for line in stdout if "Shutter Speed" in line]
    if len(shutter_line) != 1:
        print("WARNING:", preamble, "- shutter speed not matched")
    elif shutter_string not in shutter_line[0]:
        raise(preamble + " - bad shutter value")
    iso_line = [line for line in stdout if "ISO" in line]
    if len(iso_line) != 1:
        print("WARNING:", preamble, "- ISO not matched")
    elif iso_string not in iso_line[0]:
        raise(preamble + " - bad ISO value")

def test_jpeg(dir):
    executable = os.path.join(dir, 'libcamera-jpeg')
    output_jpg = os.path.join(dir, 'test.jpg')
    output_shutter = os.path.join(dir, 'shutter.jpg')
    logfile = os.path.join(dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_jpeg')
    clean_dir(dir)

    # "jpg test". See if the executable appears to run and write an jpg output file.
    print("    jpg test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg],
                                         logfile)
    check_retcode(retcode, "test_jpeg: jpg test")
    check_time(time_taken, 2, 8, "test_jpeg: jpg test")
    check_size(output_jpg, 1024, "test_jpeg: jpg test")
    # For this one, we're actually going to peak inside the jpeg.
    check_jpeg(output_jpg, "test_jpeg: jpg test")

    # "shutter test". See if we appear to get the shutter/gain we asked for.
    print("    shutter test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-o', output_shutter,
         '--shutter', '20000', '--gain', '1.0', '--awbgains', '1.0,1.0'], logfile)
    check_retcode(retcode, "test_jpeg: shutter test")
    check_time(time_taken, 2, 8, "test_jpeg: shutter test")
    check_size(output_shutter, 1024, "test_jpeg: shutter test")
    check_jpeg_shutter(output_shutter, '1/50', '100', "test_jpeg: shutter test")

    print("libcamera-jpeg tests passed")

def test_vid(dir):
    executable = os.path.join(dir, 'libcamera-vid')
    output_h264 = os.path.join(dir, 'test.h264')
    output_mjpeg = os.path.join(dir, 'test.mjpeg')
    output_circular = os.path.join(dir, 'circular.h264')
    output_pause = os.path.join(dir, 'pause.h264')
    logfile = os.path.join(dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_vid')
    clean_dir(dir)

    # "h264 test". See if the executable appears to run and write an h264 output file.
    print("    h264 test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264],
                                         logfile)
    check_retcode(retcode, "test_vid: h264 test")
    check_time(time_taken, 2, 5, "test_vid: h264 test")
    check_size(output_h264, 1024, "test_vid: h264 test")

    # "mjpeg test". As above, but write an mjpeg file.
    print("    mjpeg test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--codec', 'mjpeg',
                                          '-o', output_mjpeg],
                                         logfile)
    check_retcode(retcode, "test_vid: mjpeg test")
    check_time(time_taken, 2, 5, "test_vid: mjpeg test")
    check_size(output_mjpeg, 1024, "test_vid: mjpeg test")

    # "segment test". As above, write the output in single frame segements.
    print("    segment test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--codec', 'mjpeg',
                                          '--segment', '1', '-o', os.path.join(dir, 'test%03d.jpg')],
                                         logfile)
    check_retcode(retcode, "test_vid: segment test")
    check_time(time_taken, 2, 5, "test_vid: segment test")
    check_size(os.path.join(dir, 'test035.jpg'), 1024, "test_vid: segment test")

    # "circular test". Test circular buffer (really we should wait for it to wrap...)
    print("    circular test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--inline', '--circular',
                                          '-o', output_circular], logfile)
    check_retcode(retcode, "test_vid: circular test")
    check_time(time_taken, 2, 5, "test_vid: circular test")
    check_size(output_circular, 1024, "test_vid: circular test")
    
    # "pause test". Should be no output file if we start 'paused'.
    print("    pause test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--inline',
                                          '--initial', 'pause', '-o', output_pause], logfile)
    check_retcode(retcode, "test_vid: pause test")
    check_time(time_taken, 2, 5, "test_vid: pause test")
    if os.path.isfile(output_pause):
        raise TestFailure("test_vid: pause test - output file was not expected")

    print("libcamera-vid tests passed")

def test_raw(dir):
    executable = os.path.join(dir, 'libcamera-raw')
    output_raw = os.path.join(dir, 'test.raw')
    logfile = os.path.join(dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_raw')
    clean_dir(dir)

    # "raw test". See if the executable appears to run and write an output file.
    print("    raw test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_raw],
                                         logfile)
    check_retcode(retcode, "test_vid: raw test")
    check_time(time_taken, 2, 5, "test_vid: raw test")
    check_size(output_raw, 1024, "test_vid: raw test")

    print("libcamera-raw tests passed")

def test_all(apps, dir):
    try:
        if 'hello' in apps:
            test_hello(dir)
        if 'still' in apps:
            test_still(dir)
        if 'jpeg' in apps:
            test_jpeg(dir)
        if 'vid' in apps:
            test_vid(dir)
        if 'raw' in apps:
            test_raw(dir)

        print("All tests passed")
        clean_dir(dir)

    except TestFailure as e:
        print("ERROR:", e)
    return

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'libcamera-apps automated tests')
    parser.add_argument('--apps', '-a', action = 'store', default = 'hello,still,vid,jpeg,raw',
                       help = 'List of apps to test')
    parser.add_argument('--dir', '-d', action = 'store', default = 'build',
                       help = 'Directory name for executables to test')
    args = parser.parse_args()
    apps = args.apps.split(',')
    dir = args.dir.strip('/')
    test_all(apps, dir)
