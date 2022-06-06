# autonomous_landing_uav

This work was developed by Isabel Castelo during her Master Thesis under direct supervision of Prof. Pedro Lima and Dr. Meysam Basiri. To read more about this work, click on the follwing [link](https://drive.google.com/file/d/17AfR2s2Ql5n2FO_m81a-wonHWeZszbup/view?usp=sharing "Thesis Link").

## Hardware

* DJI F550 Hexacopter
* Pixhawk
* Realsense 435


## Dependencies

* MAVROS
* OpenCV
* Fiducials package (https://github.com/UbiquityRobotics/fiducials.git)

NOTES: In the Fiducials package, you must include the **aruco_detect3.launch** file in the launch directory of the Fiducials package. This file can be found in this repository.



## Aruco Markers

The pattern that we used for this test has 4 aruco markers (IDs: 55, 168, 227 and 946). The pattern is included in this repository.



## Test Description




## Running the code

Run the following launch file to run thr mavros node, the realsense node, the aruco detection node, and the necessary transforms:
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
