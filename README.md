# unitree_inspection
This repository has code and launch files for running autonomous navigation, SLAM, and machine learning Optical Character Recognition for industrial inspection with the Unitree Go1.

Use `ros2 launch unitree_inspection autonomous_inspection.launch.py` to launch all required nodes for a demo where the Go1 navigates around its map, avoiding obstacles it encounters along the way. The Go1's goal is to reach inspection points predefined as 2D poses with a text label in the [environment parameters yaml file](unitree_inspection/config/environment_params.yaml). Once an inspection point has been reached, the Go1 will sweep up and down to find text it recognizes as a text label for the next inspection point. It will then update its goal, navigate to the next point, and inspect again. The Go1 will continue to do this until it is told to return to the origin of the map by a text label's corresponding 2D pose.

## Demo Video
Here's a video of the demo in action:
TODO


## Dependencies
To build the packages in this repository, several dependencies are required. Use the following commands to create a workspace `ws` with a properly built project:
```
mkdir -p ws/src
cd ws/src
git clone git@github.com:ngmor/unitree_inspection.git
cd ..
vcs import < src/unitree_inspection/inspection.repos
cd src/rslidar_sdk_ros2
git submodule init
git submodule update
cd ../..
colcon build
```

## Packages
## unitree_inspection / unitree_inspection_interfaces
TODO
### Launch Files
TODO
### Nodes
TODO

## unitree_ocr / unitree_ocr_interfaces
TODO
### Launch Files
TODO
### Nodes
TODO


## East Detection Model
https://github.com/oyyd/frozen_east_text_detection.pb

## Text Recognition Models
https://github.com/opencv/opencv/blob/master/doc/tutorials/dnn/dnn_text_spotting/dnn_text_spotting.markdown

## OpenCV OCR Example
https://github.com/opencv/opencv/blob/master/samples/dnn/text_detection.cpp