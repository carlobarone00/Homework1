# 🤖 Armando Gazebo Package

## 📘 About
This package contains the code implementation of the Homework1
 
---

## 🛠️ Build & Installation
Make sure the following dependencies are installed:

### ➡️ Installation

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher
sudo apt update
sudo apt install ros-humble-urdf-launch
```
Set the following environment variable to correctly locate the meshes (if it hasn't been added to your Dockerfile): 

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/src/armando_description/meshes
```

---

## 🚀 Usage

To launch the `armando_display.launch.py` file run the armando_display.launch.py file to visualize the robot in Rviz:

```bash
ros2 launch armando_description armando_display.launch.py
```

Run the armando_world.launch.py file to visualize the robot in Gazebo and set the choice_controll argument to start the desired controller:

* choice_controll = 0: starts the position_controller.

* choice_controll = 1: starts the joint_trajectory_controller.


To launch the position_controller, execute:

```bash
ros2 launch armando_gazebo armando_world.launch.py choice_controll:=0
```

To launch the joint_trajectory_controller, execute:

```bash
ros2 launch armando_gazebo armando_world.launch.py choice_controll:=1
```

To view which controllers are currently active, run the rqt_controller_manager tool in a separate terminal (first-time users should install it):

```bash
sudo apt-get install ros-<distro>-rqt-controller-manager
ros2 run rqt_controller_manager rqt_controller_manager
```
Finally, to verify that the image topic is correctly published, use rqt_image_view:

```bash
ros2 run rqt_image_view rqt_image_view
```


