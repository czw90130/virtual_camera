#!/bin/bash 

sudo modprobe v4l2loopback devices=2

for((i=0;i<20;i++));do
    dv="/dev/video$i"
    if [ ! -e "$dv" ] ;then
	break
    fi
done

echo "/dev/video$(($i-2))" > $(cd "$(dirname "$0")"; pwd)/../data/yuyv_camera.txt
echo "/dev/video$(($i-1))" > $(cd "$(dirname "$0")"; pwd)/../data/yuy2_camera.txt
