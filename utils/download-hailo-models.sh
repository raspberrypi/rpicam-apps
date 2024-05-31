#!/bin/bash
set -e

networks=(
    # Image classification
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/resnet_v1_50_h8l.hef
    # Yolov6 inference
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov6n.hef
    # Yolov8 inference
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov8s_h8l.hef
    # Yolov5 segmentation
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov5n_seg_h8l_mz.hef
    # YoloX inference
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolox_s_leaky_h8l_mz.hef
    # Yolov8 pose
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov8s_pose_h8l_pi.hef
    # Yolov5 person/face inference
    https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov5s_personface_h8l_pi.hef
)

if [ $# -ne 1 ]; then
    echo "Usage: $0 <directory>"
    exit 1
fi

dir=$1
mkdir -p $dir

for url in ${networks[@]}; do
    wget -nv -N -P $dir $url
done
