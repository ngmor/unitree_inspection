# unitree_inspection
This repository has code and launch files for running autonomous navigation, SLAM, and machine learning Optical Character Recognition for industrial inspection with the Unitree Go1.

Use `ros2 launch unitree_inspection autonomous_inspection.launch.py` to launch all required nodes for a demo where the Go1 navigates around its map, avoiding obstacles it encounters along the way. The Go1's goal is to reach inspection points predefined as 2D poses with a text label in the [environment parameters yaml file](unitree_inspection/config/environment_params.yaml). Once an inspection point has been reached, the Go1 will sweep up and down to find text it recognizes as a text label for the next inspection point. It will then update its goal, navigate to the next point, and inspect again. The Go1 will continue to do this until it is told to return to the origin of the map by a text label's corresponding 2D pose.

See the portfolio post for this project [here](https://ngmor.github.io/projects/legged-autonomous-inspection/).

## Demo Video
Here's a video of the demo in action:

[Inspection Demo](https://user-images.githubusercontent.com/113186159/225857165-a1586f8f-e21b-4c1d-9634-d020e9593030.mp4)

For a higher quality, [watch this video on YouTube](https://www.youtube.com/watch?v=onxPLXfmMvo).


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
Use `ros2 launch unitree_inspection ${launch_file_name} --show-args` to view arguments for each launch file.

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
This package handles Optical Character Recognition (OCR) with the Unitree Go1's cameras.

This OCR is performed using EAST text detection and CRNN text recognition machine learning models. These models are not included in this repository, and must be downloaded and placed in the `models` folder of this package separately. For more information, see [here](unitree_ocr/models/README.md).

### Launch Files
Use `ros2 launch unitree_ocr ${launch_file_name} --show-args` to view arguments for each launch file.

- `text_detection_subscriber.launch.py` - launches the `text_detection_subscriber` node with arguments and remappings necessary to detect text from images published by the Go1's head front camera.

### Nodes
#### text_detection_subscriber
This node subscribes to a camera topic, runs the OCR models on it, and then publishes the detected text. In the `unitree_ocr_interaces/Detections` message that it publishes, counts are provided for both the number of consecutive frames any text is detected and the number of consecutive frames any particular text is detected. This can be used to help filter out transient detections that sometimes occur.

### OCR Library
The OCR library contains a class called `TextDetector` which wraps up the EAST/CRNN detection functionality. It is based on [OpenCV's code](https://github.com/opencv/opencv/blob/master/samples/dnn/text_detection.cpp).

After building this repository with `colcon build`, documentation for this OCR library can be found by opening `ws/build/unitree_ocr/html/index.html` in a browser.