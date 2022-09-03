#! /bin/bash

datum=$(date +%Y%m%d_%H%M%S)
PNAME="libcamera-detect" # Process to Watch
load="$(/usr/bin/ps -C ${PNAME} -o %cpu | tail -1)"
echo "step 1"
b=10 # CPU Useage for compare
int=${load%.*}
echo "step 2"
re='^[0-9]+$' # only numbers regex
if ! [[ $int =~ $re ]] # Test for NOT Number
then
   load2=1
   echo "step 3"
fi
if [[ $int =~ $re ]]
then
   load2=$int
   echo "step 4"
fi

echo "step 5"
echo "step 6"
if [ $load2 -lt $b ]
then
  process="$(pidof libcamera-detect | wc -w)"
   if [ $process == 0 ];
   then
     echo "step 7"
     /usr/local/bin/libcamera-detect -n -t 0 -o "%04d.jpg" --saturation 0.2 --contrast 1.3 --hflip --vflip --lores-width 450 --lores-height 300 --gap 10 --framerate 30 --verbose 0 --post-process-file /home/your_path/object_detect_tf.json
   else
     echo "stop 8"
     killall libcamera-detect
   fi
fi
echo "$datum" "step 9"
