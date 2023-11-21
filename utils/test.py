#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2020, Raspberry Pi (Trading) Limited
#
# test.py - some automated testing for rpicam-apps

# These tests are very far from exhaustive, for which I apologise
# profusely, but still it's better than nothing. The rule going
# forward is that any new feature, or feature that needs a fix, must
# get a test in here.

import argparse
from enum import Enum
import fcntl
import json
import os
import os.path
import subprocess
import sys
from timeit import default_timer as timer
import v4l2
import numpy as np


def get_platform():
    platform = 'vc4'
    try:
        for num in range(5):
            device = '/dev/video' + str(num)
            if os.path.exists(device):
                with open(device, 'rb+', buffering=0) as fd:
                    caps = v4l2.v4l2_capability()
                    fcntl.ioctl(fd, v4l2.VIDIOC_QUERYCAP, caps)
                    decoded = caps.card.decode('utf-8')
                    if decoded == 'rp1-cfe':
                        platform = 'pisp'
                        break
                    elif decoded == 'unicam':
                        break
    except Exception:
        pass

    return platform


class TestFailure(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)


def check_exists(file, preamble):
    if not os.path.isfile(file):
        raise TestFailure(preamble + ": " + file + " not found")


def clean_dir(dir, exts=('.jpg', '.png', '.bmp', '.dng', '.h264', '.mjpeg', '.raw', 'log.txt', 'timestamps.txt', 'metadata.json', 'metadata.txt')):
    for file in os.listdir(dir):
        if file.endswith(exts):
            os.remove(os.path.join(dir, file))


def run_executable(args, logfile):
    start_time = timer()
    with open(logfile, 'w') as logfile:
        p = subprocess.Popen(args, stdout=logfile, stderr=subprocess.STDOUT)
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
        p = subprocess.Popen(['exiftool', file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout = p.communicate()[0].decode('utf-8')
    except FileNotFoundError as e:
        print("WARNING:", preamble, "- exiftool not found")
        return
    if p.returncode:
        raise TestFailure(preamble + "- exiftool failed")
    if "Image Width" not in stdout:
        raise TestFailure(preamble + "- bad EXIF data")


def test_hello(exe_dir, output_dir):
    executable = os.path.join(exe_dir, 'rpicam-hello')
    logfile = os.path.join(output_dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_hello')
    clean_dir(output_dir)

    # "run test". Just see if the executable appeared to run.
    print("    run test")
    retcode, time_taken = run_executable([executable, '-t', '2000'], logfile)
    check_retcode(retcode, "test_hello: run test")
    check_time(time_taken, 1.8, 6, "test_hello: run test")

    # "roi test". Specify an roi and see if it blows up.
    print("    roi test")
    retcode, time_taken = run_executable(
        [executable, '-t', '2000', '--roi', '0.25,0.25,0.5,0.5'], logfile)
    check_retcode(retcode, "test_hello: roi test")
    check_time(time_taken, 1.8, 6, "test_hello: roi test")

    # "controls test". Specify some image controls and see if it blows up.
    print("    controls test")
    retcode, time_taken = run_executable(
        [executable, '-t', '2000', '--brightness', '0.2', '--contrast', '1.2',
         '--saturation', '1.3', '--sharpness', '1.5'], logfile)
    check_retcode(retcode, "test_hello: controls test")
    check_time(time_taken, 1.8, 6, "test_hello: controls test")

    # "controls test". Apply flips and see if it blows up.
    print("    flips test")
    retcode, time_taken = run_executable(
        [executable, '-t', '2000', '--hflip', '--vflip'], logfile)
    check_retcode(retcode, "test_hello: flips test")
    check_time(time_taken, 1.8, 6, "test_hello: flips test")

    # "no-raw". Run without a raw stream
    print("    no-raw test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--no-raw'], logfile)
    check_retcode(retcode, "test_hello: no-raw test")
    check_time(time_taken, 1.8, 6, "test_hello: no-raw test")

    print("rpicam-hello tests passed")


def check_size(file, limit, preamble):
    if os.path.getsize(file) < limit:
        raise TestFailure(preamble + " failed, file " + file + " too small")


def check_metadata(file, timestamp_file, preamble):
    try:
        with open(timestamp_file) as f:
            times = np.loadtxt(f)
    except Exception:
        raise TestFailure(preamble + " - could not read timestamps from file")
    try:
        with open(file) as f:
            data = json.load(f)
    except Exception:
        raise TestFailure(preamble + " - could not read data from file")
    if len(data) != len(times):
        raise TestFailure(preamble + " - metadata file has " + ("fewer" if len(data) < len(times) else "more") + " data points than timestamps file")
    diffs = np.diff([x["SensorTimestamp"] for x in data]).astype(np.float64)
    t_diffs = np.diff(times)
    diffs /= 1000000
    if not np.allclose(diffs[2:4], t_diffs[2:4], rtol=0.01):
        print(diffs[2:4], t_diffs[2:4])
        raise TestFailure(preamble + " - metadata times don't match timestamps")


def check_single_metadata(file, preamble):
    try:
        with open(file) as f:
            data = json.load(f)
    except Exception:
        raise TestFailure(preamble + " - could not read data from file")
    if type(data) != dict:
        raise TestFailure(preamble + " - metadata file is not a dict")
    try:
        if type(data["SensorTimestamp"]) != int:
            raise TestFailure(preamble + " - sensor timestamp is not an int")
    except KeyError:
        raise TestFailure(preamble + " - sensor timestamp is not present")


def check_metadata_txt(file, preamble):
    try:
        with open(file) as f:
            line1 = f.readline()
            line2 = f.readline()
            line3 = f.readline()
    except Exception:
        raise TestFailure(preamble + " - could not read data from file")
    try:
        assert "=" in line1
        assert "=" in line2
        assert "=" in line3
    except Exception:
        raise TestFailure(preamble + " - metadata file does not contain expected data")


def test_still(exe_dir, output_dir):
    executable = os.path.join(exe_dir, 'rpicam-still')
    output_jpg = os.path.join(output_dir, 'test.jpg')
    output_png = os.path.join(output_dir, 'test.png')
    output_bmp = os.path.join(output_dir, 'test.bmp')
    output_dng = os.path.join(output_dir, 'test.dng')
    output_metadata = os.path.join(output_dir, 'metadata.json')
    output_metadata_txt = os.path.join(output_dir, 'metadata.txt')
    logfile = os.path.join(output_dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_still')
    clean_dir(output_dir)

    # "jpg test". See if the executable appears to run and write an jpg output file.
    print("    jpg test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg], logfile)
    check_retcode(retcode, "test_still: jpg test")
    check_time(time_taken, 1.2, 8, "test_still: jpg test")
    check_size(output_jpg, 1024, "test_still: jpg test")

    # "no-raw test". As above but without a raw stream.
    print("    no-raw test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg, '--no-raw'], logfile)
    check_retcode(retcode, "test_still: no-raw test")
    check_time(time_taken, 1.2, 8, "test_still: no-raw test")
    check_size(output_jpg, 1024, "test_still: no-raw test")

    # "zsl test". As above, but with zsl enabled
    print("    zsl test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg, '--zsl'], logfile)
    check_retcode(retcode, "test_still: zsl test")
    check_time(time_taken, 1.2, 8, "test_still: zsl test")
    check_size(output_jpg, 1024, "test_still: zsl test")

    # "immediate test". Immediate capture test
    print("    immediate test")
    retcode, time_taken = run_executable([executable, '-o', output_jpg, '--immediate', '--shutter', '20000',
                                          '--gain', '1.0', '--awbgains', '1.5,1.2'], logfile)
    check_retcode(retcode, "test_still: immediate test")
    check_time(time_taken, 0.2, 5, "test_still: immediate test")
    check_size(output_jpg, 1024, "test_still: immediate test")

    # "png test". As above, but write a png.
    print("    png test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-e', 'png', '-o', output_png], logfile)
    check_retcode(retcode, "test_still: png test")
    check_time(time_taken, 1.2, 9, "test_still: png test")
    check_size(output_png, 1024, "test_still: png test")

    # "bmp test". As above, but write a bmp.
    print("    bmp test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-e', 'bmp', '-o', output_bmp], logfile)
    check_retcode(retcode, "test_still: bmp test")
    check_time(time_taken, 1.2, 9, "test_still: bmp test")
    check_size(output_png, 1024, "test_still: bmp test")

    # "dng test". Write a dng along with the jpg.
    print("    dng test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-o', output_jpg, '-r'], logfile)
    check_retcode(retcode, "test_still: dng test")
    check_time(time_taken, 1.2, 10, "test_still: dng test")
    check_size(output_jpg, 1024, "test_still: dng test")
    check_size(output_dng, 1024 * 1024, "test_still: dng test")

    # "timelapse test". Check that a timelapse sequence captures more than one jpg.
    print("    timelapse test")
    retcode, time_taken = run_executable(
        [executable, '-t', '10000', '--timelapse', '3500', '-o', os.path.join(output_dir, 'test%03d.jpg')],
        logfile)
    check_retcode(retcode, "test_still: timelapse test")
    check_time(time_taken, 9, 20, "test_still: timelapse test")
    check_size(os.path.join(output_dir, 'test000.jpg'), 1024, "test_still: timelapse test")
    check_size(os.path.join(output_dir, 'test001.jpg'), 1024, "test_still: timelapse test")
    if os.path.isfile(os.path.join(output_dir, 'test002.jpg')):
               raise("test_still: timelapse test, unexpected output file")

    # "metadata test". Check that the json metadata file is written and looks sensible
    print("    metadata test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg,
                                          '--metadata', output_metadata], logfile)
    check_retcode(retcode, "test_still: metadata test")
    check_time(time_taken, 1.2, 8, "test_still: metadata test")
    check_size(output_jpg, 1024, "test_still: metadata test")
    check_single_metadata(output_metadata, "test_still: metadata test")

    # "metadata txt test". Check that the txt metadata file is written and looks sensible
    print("    metadata txt test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg,
                                          '--metadata', output_metadata_txt,
                                          '--metadata-format', 'txt'], logfile)
    check_retcode(retcode, "test_still: metadata txt test")
    check_time(time_taken, 1.2, 8, "test_still: metadata txt test")
    check_size(output_jpg, 1024, "test_still: metadata txt test")
    check_metadata_txt(output_metadata_txt, "test_still: metadata txt test")

    print("rpicam-still tests passed")


def check_jpeg_shutter(file, shutter_string, iso_string, preamble):
    # Verify that the expected shutter_string and iso_string are in the exif.
    try:
        p = subprocess.Popen(['exiftool', file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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


def test_jpeg(exe_dir, output_dir):
    executable = os.path.join(exe_dir, 'rpicam-jpeg')
    output_jpg = os.path.join(output_dir, 'test.jpg')
    output_shutter = os.path.join(output_dir, 'shutter.jpg')
    logfile = os.path.join(output_dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_jpeg')
    clean_dir(output_dir)

    # "jpg test". See if the executable appears to run and write an jpg output file.
    print("    jpg test")
    retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg],
                                         logfile)
    check_retcode(retcode, "test_jpeg: jpg test")
    check_time(time_taken, 1.2, 8, "test_jpeg: jpg test")
    check_size(output_jpg, 1024, "test_jpeg: jpg test")
    # For this one, we're actually going to peak inside the jpeg.
    check_jpeg(output_jpg, "test_jpeg: jpg test")

    # "isolation test". As above, but force IPA to run is "isolation" mode.
    # Disabled for now due to https://bugs.libcamera.org/show_bug.cgi?id=137
    #print("    isolation test")
    #os.environ['LIBCAMERA_IPA_FORCE_ISOLATION'] = 'true'
    #retcode, time_taken = run_executable([executable, '-t', '1000', '-o', output_jpg],
    #                                     logfile)
    #os.environ.pop('LIBCAMERA_IPA_FORCE_ISOLATION')
    #check_retcode(retcode, "test_jpeg: isolation test")
    #check_time(time_taken, 1.2, 8, "test_jpeg: isolation test")
    #check_size(output_jpg, 1024, "test_jpeg: isolation test")
    # For this one, we're actually going to peak inside the jpeg.
    #check_jpeg(output_jpg, "test_jpeg: isolation test")

    # "shutter test". See if we appear to get the shutter/gain we asked for.
    print("    shutter test")
    retcode, time_taken = run_executable(
        [executable, '-t', '1000', '-o', output_shutter,
         '--shutter', '20000', '--gain', '2.0', '--awbgains', '1.0,1.0'], logfile)
    check_retcode(retcode, "test_jpeg: shutter test")
    check_time(time_taken, 1.2, 8, "test_jpeg: shutter test")
    check_size(output_shutter, 1024, "test_jpeg: shutter test")
    check_jpeg_shutter(output_shutter, '1/50', '200', "test_jpeg: shutter test")

    print("rpicam-jpeg tests passed")


def check_timestamps(file, preamble):
    try:
        with open(file) as f:
            line1 = f.readline()
            line2 = f.readline()
            line3 = f.readline()
    except:
        raise TestFailure(preamble + " - could not read data from file")
    if not line1.startswith("# timecode format"):
        raise TestFailure(preamble + " - bad file header")
    try:
        t2 = float(line2)
        t3 = float(line3)
    except:
        raise TestFailure(preamble + " - timestamp file contains non-numeric values")
    if t2 >= t3:
        raise TestFailure(preamble + " - timestamps not increasing")


def test_vid(exe_dir, output_dir):
    platform = get_platform()
    executable = os.path.join(exe_dir, 'rpicam-vid')
    output_h264 = os.path.join(output_dir, 'test.h264')
    output_mkv = os.path.join(output_dir, 'test.mkv')
    output_mp4 = os.path.join(output_dir, 'test.mp4')
    output_mjpeg = os.path.join(output_dir, 'test.mjpeg')
    output_circular = os.path.join(output_dir, 'circular.h264')
    output_pause = os.path.join(output_dir, 'pause.h264')
    output_timestamps = os.path.join(output_dir, 'timestamps.txt')
    output_metadata = os.path.join(output_dir, 'metadata.json')
    output_metadata_txt = os.path.join(output_dir, 'metadata.txt')
    logfile = os.path.join(output_dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_vid')
    clean_dir(output_dir)

    # "h264 test". See if the executable appears to run and write an h264 output file.
    print("    h264 test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264],
                                         logfile)
    check_retcode(retcode, "test_vid: h264 test")
    check_time(time_taken, 2, 6, "test_vid: h264 test")
    check_size(output_h264, 1024, "test_vid: h264 test")

    # "no-raw". As above, but with no raw stream
    print("    h264 no-raw ltest")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264, '--no-raw'],
                                         logfile)
    check_retcode(retcode, "test_vid: no-raw test")
    check_time(time_taken, 2, 6, "test_vid: no-raw test")
    check_size(output_h264, 1024, "test_vid: no-raw test")

    # "libav x264 mkv test". See if the executable appears to run and write an mkv output file.
    print("    libav libx264 mkv test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_mkv, '--codec', 'libav',
                                          '--libav-video-codec', 'libx264'], logfile)
    check_retcode(retcode, "test_vid: libav libx264 mkv test")
    check_time(time_taken, 2, 6, "test_vid: libav libx264 mkv test")
    check_size(output_mkv, 1024, "test_vid: libav libx264 mkv test")

    # "libav x264 mp4 test". As above, but with mp4
    print("    libav libx264 mp4 test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_mp4, '--codec', 'libav',
                                          '--libav-video-codec', 'libx264'], logfile)
    check_retcode(retcode, "test_vid: libav libx264 mp4 test")
    check_time(time_taken, 2, 6, "test_vid: libav libx264 mp4 test")
    check_size(output_mp4, 1024, "test_vid: libav libx264 mp4 test")

    # "libav x264 options test". See if the executable appears to run and write an h264 output file with codec options.
    print("    libav libx264 options test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264, '--codec', 'libav',
                                          '--libav-video-codec', 'libx264',
                                          '--libav-video-codec-opts', 'preset=ultrafast;profile=high;partitions=i8x8,i4x4'], logfile)
    check_retcode(retcode, "test_vid: libav libx264 options test")
    check_time(time_taken, 2, 6, "test_vid: libav libx264 options test")
    check_size(output_h264, 1024, "test_vid: libav libx264 options test")

    # "mjpeg test". As above, but write an mjpeg file.
    print("    mjpeg test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--codec', 'mjpeg',
                                          '-o', output_mjpeg],
                                         logfile)
    check_retcode(retcode, "test_vid: mjpeg test")
    check_time(time_taken, 2, 6, "test_vid: mjpeg test")
    check_size(output_mjpeg, 1024, "test_vid: mjpeg test")

    if platform == 'pisp':
        print("skipping unsupported Pi 5 rpicam-vid tests")
        return

    # "segment test". As above, write the output in single frame segements.
    print("    segment test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--codec', 'mjpeg',
                                          '--segment', '1', '-o', os.path.join(output_dir, 'test%03d.jpg')],
                                         logfile)
    check_retcode(retcode, "test_vid: segment test")
    check_time(time_taken, 2, 6, "test_vid: segment test")
    # A bug in commit b20dc097621a trunctated each jpg to 4096 bytes, so check against 4100:
    check_size(os.path.join(output_dir, 'test035.jpg'), 4100, "test_vid: segment test")

    # "circular test". Test circular buffer (really we should wait for it to wrap...)
    print("    circular test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--inline', '--circular',
                                          '-o', output_circular], logfile)
    check_retcode(retcode, "test_vid: circular test")
    check_time(time_taken, 2, 6, "test_vid: circular test")
    check_size(output_circular, 1024, "test_vid: circular test")

    # "pause test". Should be no output file if we start 'paused'.
    print("    pause test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '--inline',
                                          '--initial', 'pause', '-o', output_pause], logfile)
    check_retcode(retcode, "test_vid: pause test")
    check_time(time_taken, 2, 6, "test_vid: pause test")
    if os.path.isfile(output_pause):
        raise TestFailure("test_vid: pause test - output file was not expected")

    # "timestamp test". Check that the timestamp file is written and looks sensible.
    print("    timestamp test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264,
                                          '--save-pts', output_timestamps], logfile)
    check_retcode(retcode, "test_vid: timestamp test")
    check_time(time_taken, 2, 6, "test_vid: timestamp test")
    check_size(output_h264, 1024, "test_vid: timestamp test")
    check_timestamps(output_timestamps, "test_vid: timestamp test")

    # "metadata test". Check that the json metadata file is written and looks sensible
    print("    metadata test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264,
                                          '--save-pts', output_timestamps,
                                          '--metadata', output_metadata], logfile)
    check_retcode(retcode, "test_vid: metadata test")
    check_time(time_taken, 2, 6, "test_vid: metadata test")
    check_size(output_h264, 1024, "test_vid: metadata test")
    check_metadata(output_metadata, output_timestamps, "test_vid: metadata test")

    # "metadata txt test". Check that the txt metadata file is written and looks sensible
    print("    metadata txt test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_h264,
                                          '--metadata', output_metadata_txt,
                                          '--metadata-format', 'txt'], logfile)
    check_retcode(retcode, "test_vid: metadata txt test")
    check_time(time_taken, 2, 6, "test_vid: metadata txt test")
    check_size(output_h264, 1024, "test_vid: metadata txt test")
    check_metadata_txt(output_metadata_txt, "test_vid: metadata txt test")

    print("rpicam-vid tests passed")


def test_raw(exe_dir, output_dir):
    executable = os.path.join(exe_dir, 'rpicam-raw')
    output_raw = os.path.join(output_dir, 'test.raw')
    logfile = os.path.join(output_dir, 'log.txt')
    print("Testing", executable)
    check_exists(executable, 'test_raw')
    clean_dir(output_dir)

    # "raw test". See if the executable appears to run and write an output file.
    print("    raw test")
    retcode, time_taken = run_executable([executable, '-t', '2000', '-o', output_raw],
                                         logfile)
    check_retcode(retcode, "test_vid: raw test")
    check_time(time_taken, 2, 8, "test_vid: raw test")
    check_size(output_raw, 1024, "test_vid: raw test")

    print("rpicam-raw tests passed")


def test_post_processing(exe_dir, output_dir, json_dir):
    logfile = os.path.join(output_dir, 'log.txt')
    print("Testing post-processing")
    clean_dir(output_dir)

    # "negate test". See if negate stage appears to run.
    print("    negate test")
    executable = os.path.join(exe_dir, 'rpicam-hello')
    check_exists(executable, 'post-processing')
    json_file = os.path.join(json_dir, 'negate.json')
    check_exists(json_file, 'post-processing')
    retcode, time_taken = run_executable([executable, '-t', '2000',
                                          '--post-process-file', json_file],
                                         logfile)
    check_retcode(retcode, "test_post_processing: negate test")
    check_time(time_taken, 2, 8, "test_post_processing: negate test")

    # "hdr test". Take an HDR capture.
    print("    hdr test")
    executable = os.path.join(exe_dir, 'rpicam-still')
    check_exists(executable, 'post-processing')
    output_hdr = os.path.join(output_dir, 'hdr.jpg')
    json_file = os.path.join(json_dir, 'hdr.json')
    check_exists(json_file, 'post-processing')
    retcode, time_taken = run_executable([executable, '-t', '2000', '--denoise', 'cdn_off',
                                          '--ev', '-2', '-o', output_hdr,
                                          '--post-process-file', json_file],
                                         logfile)
    check_retcode(retcode, "test_post_processing: hdr test")
    check_time(time_taken, 2, 12, "test_post_processing: hdr test")
    check_size(output_hdr, 1024, "test_post_processing: hdr test")

    # "sobel test". Try to run a stage that uses OpenCV.
    print("    sobel test")
    executable = os.path.join(exe_dir, 'rpicam-hello')
    check_exists(executable, 'post-processing')
    json_file = os.path.join(json_dir, 'sobel_cv.json')
    check_exists(json_file, 'post-processing')
    retcode, time_taken = run_executable([executable, '-t', '2000',
                                          '--viewfinder-width', '1024', '--viewfinder-height', '768',
                                          '--post-process-file', json_file],
                                         logfile)
    check_retcode(retcode, "test_post_processing: sobel test")
    check_time(time_taken, 2, 8, "test_post_processing: sobel test")
    if open(logfile, 'r').read().find('No post processing stage found') >= 0:
        print("WARNING: test_post_processing: sobel test - missing stages, test incomplete")

    # "detect test". Try to run a stage that uses TFLite.
    print("    detect test")
    executable = os.path.join(exe_dir, 'rpicam-hello')
    check_exists(executable, 'post-processing')
    json_file = os.path.join(json_dir, 'object_detect_tf.json')
    check_exists(json_file, 'post-processing')
    # Not finding the model files, or the stage not loading, produce warnings but are not errors.
    try:
        json_object = json.load(open(json_file, 'r'))
        model_file = json_object['object_detect_tf']['model_file']
        labels_file = json_object['object_detect_tf']['labels_file']
        check_exists(model_file, 'post-processing')
        check_exists(labels_file, 'post-processing')
    except Exception:
        print('WARNING: test_post_processing: detect test - model unavailable, skipping test')
    else:
        retcode, time_taken = run_executable([executable, '-t', '2000',
                                              '--lores-width', '400', '--lores-height', '300',
                                              '--post-process-file', json_file],
                                             logfile)
        check_retcode(retcode, "test_post_processing: detect test")
        check_time(time_taken, 2, 8, "test_post_processing: detect test")
        log_text = open(logfile, 'r').read()
        if log_text.find('No post processing stage found') >= 0:
            print("WARNING: test_post_processing: detect test - missing stages, test incomplete")
        else:
            if log_text.find('Inference time') < 0:  # relies on "verbose" being set in the JSON
                raise TestFailure("test_post_processing: detect test - TFLite model did not run")

    print("post-processing tests passed")


def test_all(apps, exe_dir, output_dir, json_dir):
    try:
        if 'hello' in apps:
            test_hello(exe_dir, output_dir)
        if 'still' in apps:
            test_still(exe_dir, output_dir)
        if 'jpeg' in apps:
            test_jpeg(exe_dir, output_dir)
        if 'vid' in apps:
            test_vid(exe_dir, output_dir)
        if 'raw' in apps:
            test_raw(exe_dir, output_dir)
        if 'post-processing' in apps:
            test_post_processing(exe_dir, output_dir, json_dir)

        print("All tests passed")
        clean_dir(output_dir)

    except TestFailure as e:
        print("ERROR:", e)
        sys.exit(1)

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'rpicam-apps automated tests')
    parser.add_argument('--apps', '-a', action='store', default='hello,still,vid,jpeg,raw,post-processing',
                        help='List of apps to test')
    parser.add_argument('--exe-dir', '-d', action='store', default='build',
                        help='Directory name for executables to test')
    parser.add_argument('--output-dir', '-o', action='store', default='.',
                        help='Directory name for output files')
    parser.add_argument('--json-dir', '-j', action='store', default='.',
                        help='Directory name for JSON post-processing files')
    args = parser.parse_args()
    apps = args.apps.split(',')
    exe_dir = args.exe_dir.rstrip('/')
    output_dir = args.output_dir
    json_dir = args.json_dir
    print("Exe_dir:", exe_dir, "Output_dir:", output_dir, "Json_dir:", json_dir)
    test_all(apps, exe_dir, output_dir, json_dir)
