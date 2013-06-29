#!/bin/bash 

yuyv=`grep /dev $(cd "$(dirname "$0")"; pwd)/../data/yuyv_camera.txt | cut -d = -f2` &&
yuy2=`grep /dev $(cd "$(dirname "$0")"; pwd)/../data/yuy2_camera.txt | cut -d = -f2` &&

gst-launch v4l2src device=$yuyv ! ffmpegcolorspace ! video/x-raw-yuv,format=\(fourcc\)YUY2 ! v4l2sink device=$yuy2
