#!/bin/bash
set -e

networks=(
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_deeplabv3plus.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_efficientdet_lite0_pp.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_efficientnet_bo.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_efficientnet_lite0.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_efficientnetv2_b0.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_efficientnetv2_b1.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_efficientnetv2_b2.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_higherhrnet_coco.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_inputtensoronly.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_mnasnet1.0.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_mobilenet_v2.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_mobilevit_xs.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_mobilevit_xxs.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_nanodet_plus_416x416_pp.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_nanodet_plus_416x416.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_posenet.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_regnetx_002.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_regnety_002.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_regnety_004.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_resnet18.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_shufflenet_v2_x1_5.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_squeezenet1.0.rpk
    https://github.com/raspberrypi/imx500-models/raw/main/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk
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
