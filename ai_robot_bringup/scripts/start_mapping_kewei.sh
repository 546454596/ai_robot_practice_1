#!/usr/bin/env bash

_ifup() {
  ret=$(ip link show "$1" up) && [[ -n ${ret} ]]  
}

vision="false"
quanergy="false"
velodyne="false"
hokuyo="false"

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
    quanergy="true" ;;
  orb_slam*)
    vision="true" ;;
esac

echo "Localization method: ${loc_method}"

# Choose map
pushd "${HOME}/Record/${loc_method}" &>/dev/null
state=$?
mapdir=$(yad --width=800 --height=400 \
	--title="Select map folder" --file-selection --directory)
echo "Map directory: ${mapdir}"
[ ${state} -eq 0 ] && popd >/dev/null

[[ ! -d "${mapdir}/Input" ]] && mkdir -p "${mapdir}/Input"
[[ ! -d "${mapdir}/Topomap" ]] && mkdir -p "${mapdir}/Topomap"

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

roslaunch ai_robot_bringup start_mapping_kewei.launch threeD:=${threed} paramfile:=${paramfile} agv:=${agv} loc_method:=${loc_method} vision:=${vision} velodyne:=${velodyne} hokuyo:=${hokuyo} quanergy:=${quanergy} map_path:=${mapdir}/
