#!/bin/bash
roslaunch semantic_label_publisher default_pytorch.launch &
echo "Launching semantic_label_publisher"
sleep 7
rosbag play '/home/cair/backup/rapyuta3/carto_filtered.bag'  --clock &
sleep 1.9
rosbag play '/home/cair/backup/rapyuta3/opt.bag' --clock /trajectory_0:=in_tf
