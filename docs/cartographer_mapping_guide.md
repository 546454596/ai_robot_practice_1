# 如何使用 Cartographer 构建地图

## 编译拓扑地图生成程序

- 进入 `~/catkin_ws/src/ai_robot_laser_core/ai_robot_navigation/src/gennodemap` 目录
- 执行 `build.sh` 脚本。如果执行成功，会生成 `build/gennodemap` 可执行文件，用于生成拓扑地图。

## 构建栅格地图和拓扑地图

进入 `~/catkin_ws/src/ai_robot_laser_core/ai_robot_bringup/scripts` 目录，执行 `bash start_mapping_kewei.sh`。该脚本会打开一个对话窗口，按照提示依次选择：

- 选择定位方式 : `carto_2d`
- 选择地图存储目录：例如 `~/Downloads/Map/20230627`。
- 选择机器人底盘: `ranger`（kewei室内机器人）

脚本启动成功后，使用手柄控操控机器人按预设路径移动，建图脚本会自动构建栅格地图。当构建完成后，点击弹出窗口中的 `OK`，即可将栅格地图保存到先前选择的地图存储目录中，文件格式为 `pbstream`，例如 `~/Downloads/Map/20230627/map.pbstream`。

同时，该脚本会自动在地图存储目录下创建 `Input` 和 `Topomap` 两个子目录。其中：

- `Input` 目录存放 `MapPoints.txt` 文件（环境地图3D点）和 `trajectory.txt`文件（基于 keyframe 的 pose node）。
- `Topomap` 目录用于存放拓扑地图。

## 更多细节

在 `ai_robot_bringup` 包中， `scripts/start_mapping_kewei.sh` 脚本最后会调用 `launch/start_mapping_kewei.launch`。该 launch 文件除了启动建图所需的必要组件，还会启动 `~/catkin_ws/src/ai_robot_laser_core/ai_robot_navigation/src/gennodemap/savemapfromcarto.sh` 脚本，该脚本负责自动保存栅格地图，构建拓扑地图，并把拓扑地图数据文件复制到地图存储目录下。通过阅读该脚本的源码，可进一步了解自动化建图背后的每一个步骤。
