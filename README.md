# autonomous_landing_uav

This work was developed by Isabel Castelo during her Master Thesis under direct supervision of Prof. Pedro Lima and Meysam Basiri. To read more about this work, click on the follwing [link](https://drive.google.com/file/d/17AfR2s2Ql5n2FO_m81a-wonHWeZszbup/view?usp=sharing "Thesis Link").

## Hardware

* DJI F550 Hexacopter
* Jetson Xavier NX
* Hex Cube Black
* Realsense 435


## Dependencies

* MAVROS
* OpenCV
* Fiducials package (https://github.com/UbiquityRobotics/fiducials.git)

NOTES: In the Fiducials package, one must include the **aruco_detect3.launch** file in the launch directory of the Fiducials package. This file can be found in this repository.



## Aruco Markers

The pattern used in this test has 4 aruco markers (IDs: 55, 168, 227 and 946). The pattern is included in this repository.



## Code Description

##### fsm.py

This is the main file where it's implemented the high level Smach. 

The code works as follows: when the camera tracks a marker, the current state changes to the landing sequence state and initialises the landing. The state machine only exits this state if it stops tracking the markers.

##### follow_UGV.py

When the state machine enters the landing sequence state, it creates an instance of the class "Publisher". This class stores all the measurements for the currently observed markers ("/fiducials_transforms" topic) and, after some processing, uses them to get the input of the PID controller.

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


## Test Description

The pilot should drive the UAV safely off the ground to a 5-8 meters height. Once the UAV is stable in this position, the pilot can switch the flight mode to GUIDED. This step will allow the execution of the state machine and thus initialize the landing sequence. 
As soon as the UAV lands on the markers, the pilot should get the control back of the drone for safety reasons.

The landing sequence can be tested multiple times in a row without the need to relaunch the code since the PID variables restart every time the flight mode exits the GUIDED mode.


## Bags

During the tests, we recorded some bags which might become helpful at some point. Here's the [link](https://drive.google.com/drive/folders/1uaRyFXZu-y_0YEwd8CXSCPzH8fAMMLzc?usp=sharing "Thesis Link").
