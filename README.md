# Final project - Fondamenti di robotica
##Gruppo B - Usama Aarif (217696) Mauro Meneghello (217564) Luca Boschiero (217460)

## Instruction for setting up the enviroment
**!!!IMPORTANT!!!**
This software component works only with ros and locosim, so make sure you cloned the [locosim folder](https://github.com/mfocchi/locosim) and set up the ros workspace and locosim enviroment before using it!

Clone this repository inside locosim/robot_control/lab_exercices.

Then download our custom world file (here) and the models (here). Place the world file in locosim/ros_impedance_controller/worlds/ and the models inside locosim/ros_impedance_controller/worlds/models. Make sure you have also the tavolo folder inside models.

## How to build?


## How to run?

1. 	Launch ur5generic.py:
	```
		python3 -i ~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/ur5_generic.py
	```
	This will load the robot and start Yolo to detect the lego blocks.
	**ATTENTION!!** The ur5generic.py in our repo is similar to the ur5generic.py inside locosim/robot_control/lab_exercices/lab_palopoli folder, but it is not the same! 
	
2. 	Launch vision.py:
	```
		python3 -i ~/ros_ws/src/locosim/robot_control/lab_exercises/lab_palopoli/vision.py
	```
	This will start the service that publishes the information about the detected blocks.
	
3.      Lauch the ros node custom_joint_publisher.cpp:
	```
		rosrun lab_palopoli custom_joint_publisher
	```
	
