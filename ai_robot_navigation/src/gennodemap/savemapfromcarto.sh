#! /bin/bash

#wait until call save
$(yad --width 500 --entry --title "wait to save map" \
    --button="gtk-ok:0" \
    --button="gtk-close:1" \
    --entry-text \
    "when you want to save map, click ok; otherwise ctrl+c")

mapfolder=$1
echo ${mapfolder}map.pbstream
mkdir ${mapfolder}
rosservice call /write_state ${mapfolder}map.pbstream true
mkdir ${mapfolder}Topomap

# generate topomap file
echo 'generate topomap'
cd ${HOME}/catkin_ws/src/ai_robot/ai_robot_navigation/src/gennodemap
mkdir Input
mkdir Result
cd src
python save_mappoint_trajs.py
cd ../build
./gennodemap

cp ${HOME}/catkin_ws/src/ai_robot/ai_robot_navigation/src/gennodemap/Input/* ${mapfolder}Input/
cp ${HOME}/catkin_ws/src/ai_robot/ai_robot_navigation/src/gennodemap/Result/* ${mapfolder}Topomap/

# get the last pose of trajectory
echo 'get the last pose of trajectory(not done)'
#cd ${mapfolder}
#${HOME}/catkin_ws/build_isolated/huawei_nav_traj_fusion/traj_fusion -E -p ${mapfolder}map.pbstream
echo 'generate node map finish!!'
