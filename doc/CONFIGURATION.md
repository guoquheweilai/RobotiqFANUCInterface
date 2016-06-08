# FANUC Robot Configuration

Follow the Modbus TCP Interface Operators Manual to define the TCP/IP Interface and the Modbus protocol. Refer to the following steps for configuring the internal data arrangement. 

## TCP/IP

1. Press MENUS.
2. Select Setup.
3. Press F1, [TYPE], and select Host Comm. Next, select TCP / IP. You will see a screen similar to the following.

Note: Make sure to alter the Robot IP Address to match your network.

## Modbus Settings

1. Press MENUS.
2. Select I/O.
3. Press F1, [TYPE], and select Modbus TCP. You will see a screen similar to the following.
4. 



## Group I/O

1. Press MENUS.
2. Select I/O.
3. Press F1, [TYPE], and select Group. You will see a screen similar to the following.

2

4. Press F2, CONFIG, and configure the following group outputs.

3

5. Press F3, IN / OUT, and perform the same configuration for group inputs.

4

Note: You can press Next and then F2, VERIFY, to check if the options selected are valid.

## Digital I/O

1. Press MENUS.
2. Select I/O.
3. Press F1, [TYPE], and select Digital. You will see a screen similar to the following.

5

4. Press F2, CONFIG, and configure the following digital outputs.

6

5. Press F3, IN / OUT, and perform the same configuration for digital inputs.

7

## Application Example

1. Import the TESTJOB.tp file. This file presents an example of operation. Please refer to the README.md on the root of this repository for information on how the bits are mapped. 


