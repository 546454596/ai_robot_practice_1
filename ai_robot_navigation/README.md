# AI robot brain
## introduction
* This is a modified version of old version(including hover_px4 and HSICNav). It is to provide a more flexible frame for all platform(ground robot or drone) and all task for robot. Thus its input and output are simplified. This version should be work with [remote_controller](https://github.com/icesit/remote_controller.git).
## schema
1. Get controller message(the controller message is origin message including each button pressed/released and rocker value from -1/right to +1/left).
2. If controller button is pressed, brain stop what is doing now and set new task according to the button; If rocker is pushed, robot move according to value; If no control message and no task, robot stop at this position; If no control message but task exists, do task. All the above will give a present control order(usually is velocity or accelerated velocity).
3. Control pose according to control order.
## how to add new task
1. add the task name and task number to the #define in AIBrain.h.
2. choose a button to start your task and set call back function.
3. add a function to class AIBrain for your task like what is done in AIBrain.think(). When this task is started, it will run your function repeatedly. Actuator is in PosControl.
4. add a judgement to your function when it should end and return to hover mode.
## how to change platform
1. replace the drone class(including .h, .cpp and the definition in PosControl).
2. check the actuator(all the drone->**() function).
## high AI function(task)
* path planning and obstacle avoid.(to be done)
## usage
* Set your ip as the old version;
* Put it in catkin_ws/src;
* catkin_make;
* roslaunch.
##how to run the whole system
* pioneer3-at as an example
1. turn on sensors including imu(mavros px4.launch), stereo image(drone_sensor start_stereo03.launch).
2. turn on rosaria(rosaria RosAria), ai_robot(start.launch) and remote controller(remote_controller remote_controller).
3. turn on orb-slam(see xue's version).
* if you want to test find path
4. set destination with
> rosrun ai_robot setDestination x y z

5. press btn2(generally btn B in betop joystick).
6. if it doesn't work as you desire, move any direction bar or just push btn1(btn a).
## example--run the gazebo simulation
* build the needed program including [sjtu_drone](https://github.com/icesit/sjtu_drone) and [orb-slam2](https://github.com/icesit/ORB_SLAM2) with saveload function;
* roslaunch sjtu_drone with the map(if launch doesn't work, please run node one by one) and orb-slam, then use keyboard to run the drone all around the map, orb-slam will generate mappoint and keyframe txt file;
* build octree and make a folder named Result in its root, run the testkeyframe then three txt appear in Result folder, copy the txt files to ai_robot/findpath/;
* calibrate the RT between slam and gtpose and set in the AIBrain init function;
* build [remotecontroller](https://github.com/icesit/remote_controller) and this ai_robot and then run;
## auther
* Xue Wuyang
* Miao Ruihang(old version)
