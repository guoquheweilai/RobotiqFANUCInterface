#!/usr/bin/env python

import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg

NUM_REGISTERS = 2
ADDRESS_READ_START = 1
ADDRESS_WRITE_START = 3

pub_gripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)
pub_robot = rospy.Publisher("modbus_wrapper/output", HoldingRegister,queue_size=500)

def showUpdatedRegisters(msg):

    rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))

    command = outputMsg.SModel_robot_output();

    initializeGripper = msg.data[0]
    controlGripper = msg.data[1]

    # # Reset Gripper
    # if initializeGripper == 2:
    #     command.rACT = 0

    # # Initialize Gripper
    # if initializeGripper == 3:
    #     command.rACT == 1
    #     command.rGTO == 1
    #     command.rSPA == 255
    #     command.rFRA == 150

    # Open Gripper
    if controlGripper == 2:
        command.rPRA = 0
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 55 # Speed
        command.rFRA = 50 # Force

    # Close Gripper
    if controlGripper == 3:
        command.rPRA = 255
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 55 # Speed
        command.rFRA = 50 # Force

    pub_gripper.publish(command)

def statusInterpreter(status):

    # Variable Initialization
    gripperStatus = 0
    gripperMovement = 0

    # Gripper Reset
    if (status.gACT == 0) and (status.gIMC == 0):
        gripperStatus = 0

    # Gripper Activating
    if (status.gACT == 1) and (status.gIMC == 1) and (status.gSTA == 0):
        gripperStatus = 1

    # Gripper Activated
    if (status.gACT == 1) and (status.gIMC == 3):
        gripperStatus = 2

    # Gripper Open
    if (status.gPOA < 50) and (status.gPOB < 50) and (status.gPOC < 50):
        handClosed = False

    # Gripper Closed
    if (status.gPOA > 200) and (status.gPOB > 200) and (status.gPOC > 200):
        handClosed = True

    # Gripper Moving
    if (status.gDTA == 0) and (status.gDTA == 0) and (status.gDTA == 0):
        handMoving = True

    # Gripper Stopped
    if (status.gDTA == 3) and (status.gDTA == 3) and (status.gDTA == 3):
        handMoving = False


    # Hand Moving and Opening
    if handMoving and not handClosed:
        gripperMovement = 0

    # Hand Moving and Closing
    if handMoving and handClosed:
        gripperMovement = 1

    # Hand Stoppend and Open
    if not handMoving and not handClosed:
        gripperMovement = 2

    # Hand Stopped and Closed
    if not handMoving and handClosed:
        gripperMovement = 3


    output = HoldingRegister()
    output.data = [gripperStatus, gripperMovement]
    
    rospy.loginfo("Updating Robot Registers")

    pub_robot.publish(output)

if __name__=="__main__":

    rospy.init_node("modbus_client")
    host = "172.16.7.4"
    port = 502

    if rospy.has_param("~ip"):
        host =  rospy.get_param("~ip")
    else:
        rospy.loginfo("For not using the default IP %s, add an arg e.g.: '_ip:=\"192.168.0.199\"'",host)
    if rospy.has_param("~port"):
        port =  rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=1234'",port)

    # setup modbus client    
    modclient = ModbusWrapperClient(host, port=port, rate=1, reset_registers=False, sub_topic="modbus_wrapper/output", pub_topic="modbus_wrapper/input")
    modclient.setReadingRegisters(ADDRESS_READ_START, NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START, NUM_REGISTERS)
    rospy.loginfo("Setup complete")
    
    # start listening to modbus and publish changes to the rostopic
    modclient.startListening()
    rospy.loginfo("Listener started")

    # Create a listener that show us a message if anything on the readable modbus registers change
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")

    sub_fanuc = rospy.Subscriber("modbus_wrapper/input", HoldingRegister, showUpdatedRegisters, queue_size=500)
    sub_robotiq = rospy.Subscriber("SModelRobotInput", inputMsg.SModel_robot_input, statusInterpreter, queue_size=500) 

    # Spins ROS
    rospy.spin()    
   
    