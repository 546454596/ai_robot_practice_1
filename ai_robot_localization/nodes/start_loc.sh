#!/bin/bash

# Check yad installation
if command -v yad &>/dev/null; then
  echo "'yad' is not available. Run 'sudo apt-get install yad' to intall it."
  exit 1
fi

pushd ${HOME}/Records/map

map_folder="$(yad --width=800 --height=400 --title='Select a map folder' --file-selection \
  --directory)"
echo "'${map_folder}' selected"

popd

roslaunch ai_robot_localization carto_loc_huawei.launch load_state_filename:=${map_folder}/map.pbstream init_pose_file:=${map_folder}/traj0_endpose.txt
