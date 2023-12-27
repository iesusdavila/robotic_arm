#!/usr/bin/env python

import roslib;

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from robotiq_3f_gripper_control.msg import Robotiq3FGripperRobotOutput
from time import sleep

from sensor_msgs.msg import JointState

pub = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)
command = Robotiq3FGripperRobotOutput();

def genCommand(char, command):
    """Update the command according to the character entered by the user."""

    if char == 'a':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150

    if char == 'r':
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 0

    if char == 'c':
        command.rPRA = 255

    if char == 'o':
        command.rPRA = 0

    if char == 'b':
        command.rMOD = 0

    if char == 'p':
        command.rMOD = 1

    if char == 'w':
        command.rMOD = 2

    if char == 's':
        command.rMOD = 3

    # If the command entered is a int, assign this value to rPRA
    try:
        command.rPRA = int(char)
        if command.rPRA > 255:
            command.rPRA = 255
        if command.rPRA < 0:
            command.rPRA = 0
    except ValueError:
        pass

    if char == 'f':
        command.rSPA += 25
        if command.rSPA > 255:
            command.rSPA = 255

    if char == 'l':
        command.rSPA -= 25
        if command.rSPA < 0:
            command.rSPA = 0

    if char == 'i':
        command.rFRA += 25
        if command.rFRA > 255:
            command.rFRA = 255

    if char == 'd':
        command.rFRA -= 25
        if command.rFRA < 0:
            command.rFRA = 0

    return command


def askForCommand(command):
    """Ask the user for a command to send to the gripper."""

    currentCommand = 'Simple 3F gripper Controller\n-----\nCurrent command:'
    currentCommand += ' rACT = ' + str(command.rACT)
    currentCommand += ', rMOD = ' + str(command.rMOD)
    currentCommand += ', rGTO = ' + str(command.rGTO)
    currentCommand += ', rATR = ' + str(command.rATR)
    ##    currentCommand += ', rGLV = ' + str(command.rGLV)
    ##    currentCommand += ', rICF = ' + str(command.rICF)
    ##    currentCommand += ', rICS = ' + str(command.rICS)
    currentCommand += ', rPRA = ' + str(command.rPRA)
    currentCommand += ', rSPA = ' + str(command.rSPA)
    currentCommand += ', rFRA = ' + str(command.rFRA)

    # We only show the simple control mode
    ##    currentCommand += ', rPRB = ' + str(command.rPRB)
    ##    currentCommand += ', rSPB = ' + str(command.rSPB)
    ##    currentCommand += ', rFRB = ' + str(command.rFRB)
    ##    currentCommand += ', rPRC = ' + str(command.rPRC)
    ##    currentCommand += ', rSPC = ' + str(command.rSPC)
    ##    currentCommand += ', rFRC = ' + str(command.rFRC)
    ##    currentCommand += ', rPRS = ' + str(command.rPRS)
    ##    currentCommand += ', rSPS = ' + str(command.rSPS)
    ##    currentCommand += ', rFRS = ' + str(command.rFRS)

    print(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += 'b: Basic mode\n'
    strAskForCommand += 'p: Pinch mode\n'
    strAskForCommand += 'w: Wide mode\n'
    strAskForCommand += 's: Scissor mode\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'

    strAskForCommand += '-->'

    return raw_input(strAskForCommand)

def callback(data):
    
    for x in range(len(data.name)):
        if(data.name[x]=="finger_1_joint_1"):
            C=data.position[x]*(255/1.22)
            command.rPRC = int(C)
        elif(data.name[x]=="finger_2_joint_1"):
            B=data.position[x]*(255/1.22)
            command.rPRB = int(B)
        elif(data.name[x]=="finger_middle_joint_1"):
            A=data.position[x]*(255/1.22)
            command.rPRA = int(A)
        pub.publish(command)


def publisher():
    """Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic."""

    rospy.init_node('Robotiq3FGripperSimpleController')

    command.rACT = 0
    pub.publish(command)
    rospy.sleep(1) 
    command.rACT = 1
    command.rMOD = 0
    command.rGTO = 1
    # command.rSPA = 255
    # command.rFRA = 150
    pub.publish(command)
    rospy.sleep(10) 
    
    command.rICF = 1
    pub.publish(command)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        rospy.Subscriber('joint_states', JointState, callback)

        rospy.spin()

if __name__ == '__main__':
    publisher()
