# Robotiq FANUC Interface

Modbus interface to allow communication between a Robotiq gripper and a Fanuc robotic arm

## Table of Contents

* [Installation](#installation)
* [Usage](#usage)
* [History](#history)
* [Credits](#credits)
* [License](#license)

## <a name="installation"></a>Installation

1. Install [robotiq ROS package](http://wiki.ros.org/robotiq).

  This package provides ROS drivers for the  Robotiq Adaptive Grippers.

2. Install [modbus ROS package](http://wiki.ros.org/modbus).

  This package stack provides a wrapper from the modbus TCP communication to standardized ROS messages

3. Download the [RobotiqFANUCInterface](https://github.com/rarrais/RobotiqFANUCInterface.git) repository to the src folder of your catkin workspace.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rarrais/RobotiqFANUCInterface.git
  ```
  
4. Build your code. Assuming your catkin workspace is located in ~/catkin_ws:

  ```bash
  cd ~/catkin_ws
  catkin_make
  ```

## <a name="usage"></a> Usage

1. Initialize the **robotiq ROS modbus node**. Make sure to substitute the gripper IP address.

  ```bash
  rosrun robotiq_s_model_control SModelTcpNode.py <gripper_ip_address>
  ```
  
  The default robotic gripper IP address is 192.168.1.11. To connect with this IP address execute the following command:

  ```bash
  rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11
  ```

2. Initialize the **robotiq fanuc interface ROS modbus node**. Make sure to substitute the robot IP address.

  ```bash
  rosrun robotiq_fanuc_interface fanuc_interface.py _ip:=<robot_ip_address>
  ```
  
3. It is possible to control the gripper using the robot Modbus registers. There are two bytes for controlling the gripper and two bytes for getting the status of the gripper.

  **Commands**: Set the first two registers (starting at 1) to control the initialization and movement of the gripper. 
  
  | Initialise Gripper [1] | 2nd bit | 1st bit |      Command     |
  |:----------------------:|:-------:|:-------:|:----------------:|
  |            0           |    0    |    0    |    Do Nothing    |
  |            1           |    0    |    1    |    Do Nothing    |
  |            2           |    1    |    0    |   Reset Gripper  |
  |            3           |    1    |    1    | Activate Gripper |
  
  | Control Gripper [2] | 2nd bit | 1st bit |    Command    |
  |:-------------------:|:-------:|:-------:|:-------------:|
  |          0          |    0    |    0    |   Do Nothing  |
  |          1          |    0    |    1    |   Do Nothing  |
  |          2          |    1    |    0    |  Open Gripper |
  |          3          |    1    |    1    | Close Gripper |
  
  NOTE: To perform movements on the gripper, make sure to set the 2nd bit of *Initialise Gripper [1]* to 0.  

  
    **Status**: Read the third and forth registers to get information on the status and movement of the gripper. 
  
  | Gripper Status [3] | 2nd bit | 1st bit |       Status       |
  |:------------------:|:-------:|:-------:|:------------------:|
  |          0         |    0    |    0    |    Gripper Reset   |
  |          1         |    0    |    1    | Gripper Activating |
  |          2         |    1    |    0    |  Gripper Activated |
  |          3         |    1    |    1    |      Not Used      |
  
  | Gripper Movement [4] | Moving (0) /  Stopped (1) | Open (0) / Close (1) |       Status      |
  |:--------------------:|:-------------------------:|:--------------------:|:-----------------:|
  |           0          |             0             |           0          |  Moving and Open  |
  |           1          |             0             |           1          | Moving and Closed |
  |           2          |             1             |           0          |  Stopped and Open |
  |           3          |             1             |           1          | Stopped and Close |
  
  


## <a name="history"></a>History

* **June 6, 2016**: Version 1.0 released. Tested with the S-Model of the Robotiq Gripper and with a fake modbus slave, simulating the robot.

## <a name="credits"></a>Credits

* Developed by [Rafael Arrais](https://github.com/rarrais). 
* Robotiq ROS Package by [Shaun Edwards](https://github.com/shaun-edwards). 
* Modbus ROS Package by [Sven Bock](https://github.com/sven-bock). 
