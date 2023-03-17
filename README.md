# unitree_inspection
This repository has code and launch files for running autonomous navigation, SLAM, and machine learning Optical Character Recognition for industrial inspection with the Unitree Go1.

Use `ros2 launch unitree_inspection autonomous_inspection.launch.py` to launch all required nodes for a demo where the Go1 navigates around its map, avoiding obstacles it encounters along the way. The Go1's goal is to reach inspection points predefined as 2D poses with a text label in the [environment parameters yaml file](unitree_inspection/config/environment_params.yaml). Once an inspection point has been reached, the Go1 will sweep up and down to find text it recognizes as a text label for the next inspection point. It will then update its goal, navigate to the next point, and inspect again. The Go1 will continue to do this until it is told to return to the origin of the map by a text label's corresponding 2D pose.

See the portfolio post for this project [here](https://ngmor.github.io/projects/legged-autonomous-inspection/).

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
This package handles the high level control of the Go1 for this project. This high level logic is mainly contained in the `inspection` node.

### Launch Files
Use `ros2 launch unitree_inspection ${launch_file_name}` --show-args to view arguments for each launch file.

- `autonomous_inspection.launch.py` - launches all nodes required for this project, including navigation nodes, OCR nodes, and the high level control node.

- `inspection.launch.py` - launches only the OCR nodes and the high level control node so that navigation nodes can be launched separately.

### Nodes
#### inspection
This node handles high level logic for control of the Go1 for this project. It interfaces with Nav2 to command the Go1 to visit different inspection points and processes the detected text at each inspection point.

##### Services
- `go_to_inspection_point` - sets the navigation goal to the location of the input text label and begins navigation sequence. This is the service called to start this node's sequence.
- `inspect_for_text` - commands the Go1 to sweep up and down, searching for text. This stops when text in the expected text label list is found.
- `sweep_pitch` - commands the Go1 to sweep up and down in the pitch direction. This functionality is used for searching for text at inspection points.
- `stop_sweep` - cancels a sweep.
- `reset_rpy` - resets the Go1 body's roll pitch and yaw to 0, returning to default standing position.
- `unitree_nav_to_pose` - testing service for commanding the Go1 to a specific pose.
- `untree_cancel_nav` - cancels all navigation goals.

#### sweep
This node only handles the sweeping for text functionality. Its functionality has been incorporated into the `inspection` node so it is not documented further here.

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