#!/bin/bash -e
release_num=$(lsb_release -r --short)
echo $release_num

if [ $release_num == "18.04" ]
then
  catkin_make --source cv_bridge_1804 --build build/cv_bridge
else
  catkin_make --source cv_bridge_2004 --build build/cv_bridge
fi

catkin_make --source sv-msgs --build build/msgs
catkin_make --source sv-srvs --build build/srvs
catkin_make --source sv-rosapp --build build/rosapp
source devel/setup.bash
