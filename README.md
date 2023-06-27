# Final project - Fondamenti di robotica
#### Gruppo B - Usama Aarif (217696) Mauro Meneghello (217564) Luca Boschiero (217460)

The report of the project, the presentation and some video are inside the **report&media** folder.


## Instruction for setting up the enviroment
**!!!IMPORTANT!!!**
This software component works only with ros and locosim, so make sure you cloned the [locosim folder](https://github.com/mfocchi/locosim) and set up the ros workspace and locosim enviroment before using it!

Clone this repository inside **locosim/robot_control/lab_exercices**.

Place the world file in **locosim/ros_impedance_controller/worlds/** and the models inside **locosim/ros_impedance_controller/worlds/models**. Make sure you have also the tavolo folder inside models.

Then move the **msg/detection_msgs** and **yolov5_ros** folders in the **ros_ws/src** folder. (you can also clone yolov5_ros from the repository [here](https://github.com/mats-robotics/yolov5_ros) ). Move these directories and not only copy them, otherwise you will get an error.

Finally download the **last.pt** file [here](https://drive.google.com/file/d/1LOnxKGTqSHdLwhOvP8nNwq1wYvvzkmGX/view?usp=drive_link) and move it into the **yolov5_ros** folder.

## How to build?

```
cd ~/ros_ws
catkin_make install
source ~/.bashrc
source devel/setup.bash
```


## How to run?

1. 	Launch ur5generic.py:
	```
	python3 -i ~/ros_ws/src/locosim/robot_control/lab_exercises/robotica/src/ur5_generic.py
	```
	This will load the robot and start Yolo to detect the lego blocks.
	**ATTENTION!!** The **ur5generic.py** in our repo is similar to the **ur5generic.py** inside **locosim/robot_control/lab_exercices/lab_palopoli** folder, but it is not the same! 
	
2. 	Launch vision.py:
	```
	python3 -i ~/ros_ws/src/locosim/robot_control/lab_exercises/robotica/src/vision.py
	```
	This will start the service that publishes the information about the detected blocks.
	**ATTENTION!!** In order to launch **vision.py** correctly, modify the file at **line 28** copying the absolute path to your models folder here:
	<br>
 	https://github.com/lucaboschiero/robotica/blob/85763b0d19a4b632aadeafc3a7a164c4c07d26e5/src/vision.py#L28C10-L28C10
	<br>
 	now recompile with:
	```
	cd ~/ros_ws
	catkin_make install
	```
	and launch it.
	
4. 	Lauch the ros node custom_joint_publisher.cpp:
	```
	rosrun robotica motion
	```
	
