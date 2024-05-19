# TECHIN517-Manipulation

## Instructions for Camera Setup D435i
Refer to this link for installing dependencies, librealsense2 and the build. 
https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide?_ga=2.42442656.40329647.1712875214-856519986.1712454558

1) Everything is pretty much covered , check if the patching is done correctly by running this command:

`sudo dmesg | tail -n 50` 
>The log should indicate that a new _uvcvideo_ driver has been registered.  
Refer to [Troubleshooting](#troubleshooting-installation-and-patch-related-issues) in case of errors/warning reports.

2) check the camera by running : 
`realsense-viewer` 

3) For ros interface installation, follow these instructions from point **NUMBER 4**
https://zhaoxuhui.top/blog/2020/09/09/intel-realsense-d435i-installation-and-use.html 

# Ros_sam Setup Guide

This part provides a detailed guide on setting up the `ros_sam` package in a ROS environment. Please follow the steps below to ensure a proper installation.

## Prerequisites

Ensure that you have a working ROS installation and that `catkin` is properly set up on your system. Additionally, you will need `git` and `pip` installed to handle repository cloning and Python packages installation.

## Installation Steps

### 1. Clone the ros_sam Repository

Navigate to the `src` directory of your catkin workspace and clone the `ros_sam` repository using the following command:

```bash
cd ~/catkin_ws/src  # Replace '~/catkin_ws' with your actual catkin workspace path
git clone https://github.com/robot-learning-freiburg/ros_sam.git
```

### 2. Download Model Checkpoints

Navigate to the [Segment-Anything Model Checkpoints](https://github.com/facebookresearch/segment-anything#model-checkpoints) page on GitHub. Download the required checkpoint files and place them in the `ros_sam/model` directory within your workspace.

### 3. Install the Segment-Anything Library

Use pip to install the `segment-anything` library directly from its GitHub repository:

```bash
pip install git+https://github.com/facebookresearch/segment-anything.git
```

### 4. Clone the rqt_image_view_seg Repository
Clone the `rqt_image_view_seg` repository into your catkin workspace's src directory:
```bash
git clone https://github.com/ipab-slmc/rqt_image_view_seg.git ~/catkin_ws/src/
```

### 5. Build the Packages
Clean your workspace and build the specific packages using catkin build:

```bash
cd ~/catkin_ws
catkin clean # Option
catkin build ros_sam rqt_image_view_seg
```


### 6. Copy Scripts
Copy the necessary script files from the `ros_sam/scripts` directory to the designated scripts folder in your workspace:

```bash
cp ~/catkin_ws/src/ros_sam/scripts/* ~/catkin_ws/src/ros_sam/scripts/
```

### Post-Installation

Ensure to source your catkin workspace to apply all changes:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Running the System

Open separate terminals and execute the following commands in each to run the system components:

- Start ROS core:
```bash
roscore
```
- Launch the RealSense camera:
```bash
roslaunch realsense2_camera rs_camera.launch
```
- Run the SAM nodes:
```bash
rosrun ros_sam sam_node.py
rosrun ros_sam node_subs.py # Alternatively, you can use sam_fix_p.py / sam_grid_p.py
```

- Launch the GUI:
```bash
roslaunch ros_sam gui_test.launch
```
You can use `rqt_plot` to monitor various topics and nodes:

```bash
rosrun rqt_plot rqt_plot
```
On the GUI page, choose `/camera/color/image_raw` for input and `/masked_images` or `/segmented/composite_image` for the output to see the segmented images.

### Common Error Solutions

If you encounter errors during execution, here are some solutions:

- Missing Dependencies: Use rosdep to install any missing dependencies: 
```bash
rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

- GUI RuntimeError: If you get a ValueError: `PyCapsule_GetPointer called with incorrect name`, check the following resource for a [solution](https://www.reddit.com/r/Veusz/comments/do9xmm/faq_pycapsule_getpointer_called_with_incorrect/).

- Control_transfer returned error: If you are receiving the control_transfer returned warning only several times during launch and then they stop, or they occur one or two times a minute after launch, then this warning can be safely ignored. [link](https://github.com/IntelRealSense/realsense-ros/issues/1510)


# Ros_yolo8 Setup Guide

## Clone the ros_yolo Repository

Navigate to the `src` directory of your catkin workspace and clone the `ultralytics_ros` repository using the following command:

```bash
cd ~/catkin_ws/src  # Navigate to the source directory of your catkin workspace
GIT_LFS_SKIP_SMUDGE=1 git clone -b noetic-devel https://github.com/Alpaca-zip/ultralytics_ros.git
```

After cloning, install the required Python dependencies:

```bash
pip install -r ~/catkin_ws/src/ultralytics_ros/requirements.txt
```
Use rosdep to install any additional dependencies:

```bash
cd ~/catkin_ws
rosdep install -r -y -i --from-paths .
```
Build the cloned repository using catkin build:
```bash
catkin build
```

## Running the System
Refer to the Run section of the `README` in the ultralytics_ros repository. Set the `input_topic` parameter to `/camera/color/image_raw` when running the node. Commands will vary based on the specific launch files and nodes available in the ultralytics_ros package.

Example use:
```bash
roslaunch ultralytics_ros tracker.launch debug:=true input_topic:=/camera/color/image_raw yolo_model:=best.pt
```

# Aruco setup

## clone the aurco repo
Navigate to the `src` directory of your catkin workspace and clone the `aurco` repository using the following command, remember to switch to `noetic-devel` before cloning:
```
git clone https://github.com/pal-robotics/aruco_ros.git
```
After cloning, navigate to the `aruco_ros` directory and `git checkout 3.1.4`.

Remember to `catkin build` and `source devel/setup.bash` at the end at the `catkin_ws` directory.

## checking aruco marker position
start realsense camera node
```
roslaunch realsense2_camera rs_camera.launch
```
Start the single.launch file
```
roslaunch aruco_ros single.launch
```
Use image_view to observe the recognition effect
```
rosrun image_view image_view image:=/aruco_single/result
```
Check the posture
```
rostopic echo /aruco_single/pose
```
The `MarkerID` is 582, and the `Markersize` is 0.034m, which can be changed in the `single.launch` file.

## transformation pipeline
start realsense camera node
```
roslaunch realsense2_camera rs_camera.launch
```
launch the yolo
```
roslaunch ultralytics_ros tracker.launch debug:=true input_topic:=/camera/color/image_raw yolo_model:=epoch300.pt
```
Start the single.launch file
```
roslaunch aruco_ros single.launch
```
Use image_view to observe the recognition effect
```
rosrun image_view image_view image:=/aruco_single/result
```
rosrun the pipeline script
```
rosrun realsense2_camera pipeline.py
```
launch kinova arm
```
roslaunch kortex_driver kortex_driver.launch arm:=gen3_lite ip_address:=10.18.2.230
```

## yolo with inorbit

socks ID = 80
bottle ID = 39
blanket ID = 81
