# FANUC Robot Configuration

Follow the following guide to configure the TCP / IP Interface, the Modbus protocol and the internal data arrangement. Refer to the FANUC Modbus TCP Interface Operators Manual for further information.

## TCP/IP

1. Press MENUS.
2. Select Setup.
3. Press F1, [TYPE], and select Host Comm. Next, select TCP / IP. You will see a screen similar to the following.

![0](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/0.png)

Note: Make sure to alter the Robot IP Address to match your network.

## Modbus Settings

1. Press MENUS.
2. Select I/O.
3. Press F1, [TYPE], and select Modbus TCP. You will see a screen similar to the following.

![1](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/1.png)


## Group I/O

1. Press MENUS.
2. Select I/O.
3. Press F1, [TYPE], and select Group. You will see a screen similar to the following.

![2](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/2.png)

4. Press F2, CONFIG, and configure the following group outputs.

![3](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/3.png)

5. Press F3, IN / OUT, and perform the same configuration for group inputs.

![4](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/4.png)

Note: You can press Next and then F2, VERIFY, to check if the options selected are valid.

## Digital I/O

1. Press MENUS.
2. Select I/O.
3. Press F1, [TYPE], and select Digital. You will see a screen similar to the following.

![5](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/5.png)

4. Press F2, CONFIG, and configure the following digital outputs.

![6](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/6.png)

5. Press F3, IN / OUT, and perform the same configuration for digital inputs.

![7](https://raw.githubusercontent.com/rarrais/RobotiqFANUCInterface/master/doc/7.png)

## Application Example

1. Import the TESTJOB.tp file. This file presents an example of operation. Please refer to the README.md on the root of this repository for information on how the bits are mapped. 


