# Ultrasonic Mobile Robot

## Table of Contents
- [Project Overview](#project-overview)
- [System Compatibility](#system-compatibility)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Project Overview
Integrated robotic system combining:
- **Autonomous Navigation**  
  Environment mapping & HTTP-based path planning
- **Human Pose Estimation**  
  Real-time body keypoint detection for HRI
- **Robotic Arm Control**  
  Precise motion planning with MoveIt

## System Compatibility
```bash
Ubuntu 18.04 + ROS Melodic
Ubuntu 20.04 + ROS Noetic
```

## Installation
```bash
git clone git@github.com:Geo-JTao/Ultrasonic-mobile-robot.git
mv Ultrasonic-mobile-robot Ultrasonic-mobile-robot_ws
cd Ultrasonic-mobile-robot_ws
```

### Install dependencies
```bash
# Edit install_ros_packages.sh first,We need ros-noetic for example
bash ./src/install_ros_packages.sh
```

### Build workspace
```bash
catkin build rm_msgs && source devel/setup.bash
catkin build && source devel/setup.bash
```

## Usage
Running the Project​ After successfully compiling all packages, you can start the project as follows:

1、Launch Basic Drivers and MoveIt Server.
Open the first terminal and run the following command to launch the basic drivers and the MoveIt server.

2、 Launch Navigation, Pose Detection, and Robotic Arm Control​.
Open the second terminal and run the following command to launch the navigation, human pose detection, and robotic arm control functions.

```bash
./server.sh
./client.sh​
```



​