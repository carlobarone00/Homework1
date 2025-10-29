# 🤖 Armando Controller Package

## 📘 About
This package contains a **ROS C++ node** named `arm_controller_node.cpp`, which implements the following functionalities:

* A **Subscriber** to the `/joint_states` topic to print the current joint positions of the robot to the console.
* A **Publisher** designed to sequentially write **four position commands** onto the `/position_controller/command` topic, specifically when the **position\_controller** is active.
* A **Publisher** designed to write the **same four position commands**  onto the `/joint_trajectory_controller/joint_trajectory` topic, specifically when the **joint\_trajectory\_controller** is active.
