#!/bin/zsh
cd build/apps
ln -s ../post_processing_stages/hailo/hailo-postproc.so ./
ln -s ../post_processing_stages/core-postproc.so ./
ln -s ../post_processing_stages/opencv-postproc.so ./
ln -s ../../assets/hailo_yolov6_inference.json ./
cd ../../
