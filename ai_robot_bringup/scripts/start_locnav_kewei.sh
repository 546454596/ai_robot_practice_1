#!/usr/bin/env bash

_ifup() {
  ret=$(ip link show "$1" up) && [[ -n ${ret} ]]  
}

vision="false"
quanergy="false"
velodyne="false"
hokuyo="false"
zed_point_cloud="false"
point_cloud_topic="/zed/zed_node/point_cloud/cloud_registered"

# Choose localization method
loc_method=$(yad --width 500 --entry --title "Choose localization method" \
	--button="gtk-ok:0" \
	--button="gtk-close:1" \
	--entry-text \
	"carto_2d" "carto_3d" "orb_slam")

case "${loc_method}" in
  carto_2d*)
    hokuyo="true";;
  carto_3d*)
    quanergy="true"
    velodyne="true"
    ;;
  orb_slam*)
    vision="true";;
esac

echo "Localization method: ${loc_method}"

# Choose map
pushd "${HOME}/Record/${loc_method}"
state=$?
mapdir=$(yad --width=800 --height=400 \
	--title="Select map folder" --file-selection --directory)
echo "Map directory: ${mapdir}"
[ ${state} -eq 0 ] && popd >/dev/null

# Choose obstacle avoid method
sensor=$(yad --width 500 --entry --title "Choose Obatacle Avoid Sensor" \
	--button="gtk-ok:0" \
	--button="gtk-close:1" \
	--entry-text \
	"hokuyo" "quanergy" "zed_point_cloud" "test")

case ${sensor} in
  hokuyo*)
    threed="false"
    hokuyo="true"
    point_cloud_topic="sensors/lidar/points"
    ;;
  quanergy*)
    threed="true"
    quanergy="true"
    point_cloud_topic="sensors/lidar/points"
    ;;
  zed_point_cloud*)
    threed="true"
    vision="true"
    point_cloud_topic="/sensors/stereo_cam/point_cloud/cloud_registered"
    ;;
  test*)
    threed="false"
    sensor="hokuyo"
    ;;
esac

paramfile="params_"${sensor}".yaml"

echo "Sensor: ${sensor}"
echo "Parameter filename: ${paramfile}"

# Choose agv
agv=$(yad --width 500 --entry --title "Choose AGV" \
	--button="gtk-ok:0" \
	--button="gtk-close:1" \
	--entry-text \
	"ranger" "jiaolong" "p3at" "scout" "autolabor" "test")

echo "AGV: ${agv}"

if ! _ifup(can0); then
  sudo ip link set can0 up type can bitrate 500000
fi

roslaunch ai_robot_bringup start_locnav_kewei.launch map_path:=${MapFolder}/ init_pose_file:=${MapFolder}/traj0_endpose.txt keyf:=${MapFolder}/Topomap/denKeyfPos.txt keyfrelation:=${MapFolder}/Topomap/denKeyfPosRelation.txt mappoint:=${MapFolder}/Topomap/MapPointsPos.txt pointToPointFilePath:=${MapFolder}/waypoints.txt threeD:=${threed} paramfile:=${paramfile} agv:=${agv} loc_method:=${loc_method} vision:=${vision} velodyne:=${velodyne} hokuyo:=${hokuyo} quanergy:=${quanergy} point_cloud_topic:=${point_cloud_topic}
