#!/bin/bash
set -e

declare -A networks=(
    # Image classification
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/resnet_v1_50_h8l.hef"]="resnet_v1_50_h8l.hef"
    # Yolov6 inference
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov6n.hef"]="yolov6n_h8l.hef"
    ["https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.13.0/hailo8/yolov6n.hef"]="yolov6n_h8.hef"
    # Yolov8 inference
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov8s_h8l.hef"]="yolov8s_h8l.hef"
    ["https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.13.0/hailo8/yolov8s.hef"]="yolov8s_h8.hef"
    # Yolov5 segmentation
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov5n_seg_h8l_mz.hef"]="yolov5n_seg_h8l_mz.hef"
    ["https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.13.0/hailo8/yolov5n_seg.hef"]="yolov5n_seg_h8.hef"
    # YoloX inference
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolox_s_leaky_h8l_mz.hef"]="yolox_s_leaky_h8l_rpi.hef"
    # Yolov8 pose
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov8s_pose_h8l_pi.hef"]="yolov8s_pose_h8l_pi.hef"
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8/yolov8s_pose.hef"]="yolov8s_pose_h8.hef"
    # Yolov5 person/face inference
    ["https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/yolov5s_personface_h8l.hef"]="yolov5s_personface_h8l.hef"
    # Face landmarking
    ["https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.13.0/hailo8l/scrfd_2.5g.hef"]="scrfd_2.5g_h8l.hef"
)

if [ $# -ne 1 ]; then
    echo "Usage: $0 <directory>"
    exit 1
fi

dir=$1
mkdir -p $dir

for url in "${!networks[@]}"; do
    filename="${networks[$url]}"
    wget -nv -O "$dir/$filename" "$url"
done
