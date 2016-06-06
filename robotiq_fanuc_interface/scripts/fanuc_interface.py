#!/usr/bin/env python

import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister
from robotiq_s_model_control.msg import _SModel_robot_output  as outputMsg
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg

NUM_REGISTERS = 1
ADDRESS_READ_START = 1
ADDRESS_WRITE_START = 3

pub_gripper = rospy.Publisher('SModelRobotOutput', outputMsg.SModel_robot_output, queue_size=10)
pub_robot = rospy.Publisher("modbus_wrapper/output", HoldingRegister,queue_size=500)

def showUpdatedRegisters(msg):
    
    rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))

    command = outputMsg.SModel_robot_output();

    initializeGripper = msg.data[0]
    controlGripper = msg.data[1]

    if initializeGripper == 1:
        # Initialize Gripper
        pass

    if controlGripper == 1:
        # Open Gripper
        command.rPRA = 0
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150


    if controlGripper == 2:
        # Close Gripper
        command.rPRA = 255
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    pub_gripper.publish(command)

def statusInterpreter(status):

    if (status.gPOA < 50) and (status.gPOB < 50) and (status.gPOC < 50):
        # Hand Open
        handClosed = False

    if (status.gPOA > 200) and (status.gPOB > 200) and (status.gPOC > 200):
        # Hand Closed
        handClosed = True

    if (status.gDTA == 0) and (status.gDTA == 0) and (status.gDTA == 0):
        handMoving = True

    if (status.gDTA == 3) and (status.gDTA == 3) and (status.gDTA == 3):
        handMoving = False

    if not handMoving and not handClosed:
        gripperStatus = 0

    if not handMoving and handClosed:
        gripperStatus = 1

    if handMoving and not handClosed:
        gripperStatus = 2

    if handMoving and handClosed:
        gripperStatus = 3


    output = HoldingRegister()
    output.data = [gripperStatus]
    
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
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setReadingRegisters(ADDRESS_READ_START,NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START,NUM_REGISTERS)
    rospy.loginfo("Setup complete")
    
    # start listening to modbus and publish changes to the rostopic
    modclient.startListening()
    rospy.loginfo("Listener started")

    # Create a listener that show us a message if anything on the readable modbus registers change
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")

    sub_fanuc = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=500)
    sub_robotiq = rospy.Subscriber("SModelRobotInput", inputMsg.SModel_robot_input, statusInterpreter, queue_size=500) 

    # Spins ROS
    rospy.spin()    
   
    