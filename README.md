# autonomous_landing_uav

This work was developed by Isabel Castelo during her Master Thesis under direct supervision of Prof. Pedro Lima and Dr. Meysam Basiri. To read more about this work, click on the follwing [link](https://drive.google.com/file/d/17AfR2s2Ql5n2FO_m81a-wonHWeZszbup/view?usp=sharing "Thesis Link").

## Hardware

* DJI F550 Hexacopter
* Jetson Xavier NX
* Pixhawk
* Realsense 435


## Dependencies

* MAVROS
* OpenCV
* Fiducials package (https://github.com/UbiquityRobotics/fiducials.git)

NOTES: In the Fiducials package, you must include the **aruco_detect3.launch** file in the launch directory of the Fiducials package. This file can be found in this repository.



## Aruco Markers

The pattern that we used for this test has 4 aruco markers (IDs: 55, 168, 227 and 946). The pattern is included in this repository.



## Code Description

##### fsm.py

This is the main file where it's implemented the high level Smach. The code works as follow: when a marker is tracked, the state changes to the landing sequence state. It will only exit this state, if the UAV has landed sucessfully or if it stops tracking the marker. 

##### follow_UGV.py

When the state machine enters the landing sequence state, it creates an instance of the class "Publisher". This class stores all the measurements for the currently observed markers ("/fiducials_transforms" topic) which after some processing, are used as inputs of the PID controller.

##### PID.py

Three independent PI controllers were implemented for the the movement on each axis.


NOTE: On pages 38 and 39 of the report, one can find a detailed description of the code.




## Launch Instructions

Run the following launch file to run the mavros node, the realsense node, the aruco detection node, and the necessary transforms:
```
$ roslaunch transforms.launch
```

On a separate terminal run the python code with the proposed implementation:
```
$ python fsm.py
```

NOTE: To exit this Smach, you must type `CTRL + Z` and then run `kill -9 [NUMBER OF THE PROCESS]`




## Bags

During the tests, we recorded some bags which might become helpful at some point. Here's the [link](https://drive.google.com/drive/folders/1uaRyFXZu-y_0YEwd8CXSCPzH8fAMMLzc?usp=sharing "Thesis Link").
