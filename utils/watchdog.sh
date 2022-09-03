#! /bin/bash

datum=$(date +%Y%m%d_%H%M%S)
PNAME="libcamera-detect"
load="$(/usr/bin/ps -C ${PNAME} -o %cpu | tail -1)"
echo "step 1" "$load"  >> /home/thomasd/cam8start.txt
b=10
int=${load%.*}
echo "step 2" "$int"  >> /home/thomasd/cam8start.txt
re='^[0-9]+$'

if ! [[ $int =~ $re ]]
then
   load2=1
   echo "step 3" "$load2"  >> /home/thomasd/cam8start.txt
fi
if [[ $int =~ $re ]]
then
   load2=$int
   echo "step 4" "$load2"  >> /home/thomasd/cam8start.txt
fi

echo "step 5" "$load2"  >> /home/thomasd/cam8start.txt
echo "step 6" "$int"
if [ $load2 -lt $b ]
then
  process="$(pidof libcamera-detect | wc -w)"
   if [ $process == 0 ];
   then
     echo "step 7" "$int" "$process " "start" >> /home/thomasd/cam8start.txt
     /usr/local/bin/libcamera-detect -n -t 0 -o "%04d.jpg" --saturation 0.2 --contrast 1.3 --hflip --vflip --lores-width 450 --lores-height 300 --gap 10 --framerate 30 --verbose 0 --post-process-file /home/thomasd/object_detect_tf.json
   else
     echo "stop 8" "$process " "stop" >> /home/thomasd/cam8start.txt
     killall libcamera-detect
   fi
fi
echo "$datum" "step 9" "$load2" "$process " "---------------------------------------------" >> /home/thomasd/cam8start.txt
