#!/usr/bin/env python

import roslib

roslib.load_manifest('robotiq_3f_gripper_control')
import rospy
from robotiq_3f_gripper_control.msg import Robotiq3FGripperRobotOutput

from robotiq_3f_gripper_control.msg import _Robotiq3FGripperRobotInput  as inputMsg
from robotiq_3f_gripper_control.msg import _Robotiq3FGripperRobotOutput as outputMsg


from time import sleep

from sensor_msgs.msg import JointState




# pub = rospy.Publisher('Robotiq3FGripperRobotOutput', outputMsg.Robotiq3FGripperRobotOutput)
# command = Robotiq3FGripperRobotOutput();




class Robotiq3FGripperROSConnector:

    def __init__(self):
        
        self.robot_name = "ROBOTIQ-3F"

        # Initialize ROS node
        rospy.init_node('robotiq3f_driver_py')
        self.rate = rospy.Rate(20) # hz

        # Publish current robot state
        self.joint_state_pub = rospy.Publisher('Robotiq_3F/joint_states', JointState, queue_size=10)
        # self.indy_state_pub = rospy.Publisher("/indy/status", GoalStatusArray, queue_size=10)
        # self.control_state_pub = rospy.Publisher("/feedback_states", FollowJointTrajectoryFeedback, queue_size=1)

        # Subscribe desired joint position
        self.joint_execute_plan_sub = rospy.Subscriber("/Robotiq3FGripperRobotInput", inputMsg.Robotiq3FGripperRobotInput, self.read_joint_status, queue_size=1)
        # self.joint_execute_plan_sub = rospy.Subscriber("/ABB/joint_states", JointState, self.read_robot_joint_status, queue_size=1)

        # # Subscribe desired joint position
        # self.joint_execute_plan_sub = rospy.Subscriber("/joint_path_command", JointTrajectory, self.execute_plan_result_cb, queue_size=1)

        # # Subscribe command
        # self.execute_joint_state_sub = rospy.Subscriber("/indy/execute_joint_state", JointState, self.execute_joint_state_cb, queue_size=1)
        # self.stop_sub = rospy.Subscriber("/stop_motion", Bool, self.stop_robot_cb, queue_size=1)
        # self.set_motion_param_sub = rospy.Subscriber("/indy/motion_parameter", Int32MultiArray, self.set_motion_param_cb, queue_size=1)

        # Misc variable
        self.joint_state_list = []

        self.joint_robot_name_list = []
        self.joint_robot_state_list = []
        # self.execute = False
        # self.vel = 3
        # self.blend = 5



    def __del__(self):
        self.indy.disconnect()
    
    def read_robot_joint_status(self, msg):
        self.joint_robot_name_list= msg.name
        self.joint_robot_state_list = msg.position
        
    def read_joint_status(self, msg):
        self.joint_state_list = []
        
        self.joint_state_list.append(msg.gPOA*(1.22-0.05)/255)
        # self.joint_state_list.append(msg.gPOB*(1.22-0.05)/255)
        # self.joint_state_list.append(msg.gPOC*(1.22-0.05)/255)
        # self.joint_state_list.append(msg.gPOS*(-0.18+0.19)/255)


    def joint_state_publisher(self):
        
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()

        if self.robot_name == 'ROBOTIQ-3F':
            joint_state_msg.name = ['finger_1_joint_1', 'finger_2_joint_1', 'finger_middle_joint_1', "palm_finger_1_joint"]
        else:
            joint_state_msg.name = ['finger_joint']

        joint_state_msg.position = self.joint_state_list 
        joint_state_msg.velocity = []
        joint_state_msg.effort = []


        # control_state_msg = FollowJointTrajectoryFeedback()
        # control_state_msg.header.stamp = rospy.Time.now()
        # if self.robot_name == 'ROBOTIQ-3F':
        #     control_state_msg.joint_names = ['joint1', 'joint2', 'joint3']
        # else:
        #     control_state_msg.joint_names = ['joint0']

        # control_state_msg.actual.positions = utils_transf.degs2rads(self.indy.get_joint_pos())
        # control_state_msg.desired.positions = utils_transf.degs2rads(self.indy.get_joint_pos())
        # control_state_msg.error.positions = [0 for i in control_state_msg.joint_names]


        self.joint_state_pub.publish(joint_state_msg)
        # self.control_state_pub.publish(control_state_msg)



    def run(self):

        while not rospy.is_shutdown():
            # self.current_robot_status = self.indy.get_robot_status()
            self.joint_state_publisher()
            # self.robot_state_publisher()

            # if self.execute:
            #     self.execute = False
            #     if self.current_robot_status['busy']:
            #         continue
            #     if self.current_robot_status['direct_teaching']:
            #         continue
            #     if self.current_robot_status['collision']:
            #         continue
            #     if self.current_robot_status['emergency']:
            #         continue
            #     if self.current_robot_status['ready']:
            #         self.move_robot()

        # self.indy.disconnect()









# def genCommand(char, command):
#     """Update the command according to the character entered by the user."""

#     if char == 'a':
#         command = Robotiq3FGripperRobotOutput();
#         command.rACT = 1
#         command.rGTO = 1
#         command.rSPA = 255
#         command.rFRA = 150

#     if char == 'r':
#         command = Robotiq3FGripperRobotOutput();
#         command.rACT = 0

#     if char == 'c':
#         command.rPRA = 255

#     if char == 'o':
#         command.rPRA = 0

#     if char == 'b':
#         command.rMOD = 0

#     if char == 'p':
#         command.rMOD = 1

#     if char == 'w':
#         command.rMOD = 2

#     if char == 's':
#         command.rMOD = 3

#     # If the command entered is a int, assign this value to rPRA
#     try:
#         command.rPRA = int(char)
#         if command.rPRA > 255:
#             command.rPRA = 255
#         if command.rPRA < 0:
#             command.rPRA = 0
#     except ValueError:
#         pass

#     if char == 'f':
#         command.rSPA += 25
#         if command.rSPA > 255:
#             command.rSPA = 255

#     if char == 'l':
#         command.rSPA -= 25
#         if command.rSPA < 0:
#             command.rSPA = 0

#     if char == 'i':
#         command.rFRA += 25
#         if command.rFRA > 255:
#             command.rFRA = 255

#     if char == 'd':
#         command.rFRA -= 25
#         if command.rFRA < 0:
#             command.rFRA = 0

#     return command


# def askForCommand(command):
#     """Ask the user for a command to send to the gripper."""

#     currentCommand = 'Simple 3F gripper Controller\n-----\nCurrent command:'
#     currentCommand += ' rACT = ' + str(command.rACT)
#     currentCommand += ', rMOD = ' + str(command.rMOD)
#     currentCommand += ', rGTO = ' + str(command.rGTO)
#     currentCommand += ', rATR = ' + str(command.rATR)
#     ##    currentCommand += ', rGLV = ' + str(command.rGLV)
#     ##    currentCommand += ', rICF = ' + str(command.rICF)
#     ##    currentCommand += ', rICS = ' + str(command.rICS)
#     currentCommand += ', rPRA = ' + str(command.rPRA)
#     currentCommand += ', rSPA = ' + str(command.rSPA)
#     currentCommand += ', rFRA = ' + str(command.rFRA)

#     # We only show the simple control mode
#     ##    currentCommand += ', rPRB = ' + str(command.rPRB)
#     ##    currentCommand += ', rSPB = ' + str(command.rSPB)
#     ##    currentCommand += ', rFRB = ' + str(command.rFRB)
#     ##    currentCommand += ', rPRC = ' + str(command.rPRC)
#     ##    currentCommand += ', rSPC = ' + str(command.rSPC)
#     ##    currentCommand += ', rFRC = ' + str(command.rFRC)
#     ##    currentCommand += ', rPRS = ' + str(command.rPRS)
#     ##    currentCommand += ', rSPS = ' + str(command.rSPS)
#     ##    currentCommand += ', rFRS = ' + str(command.rFRS)

#     print(currentCommand)

#     strAskForCommand = '-----\nAvailable commands\n\n'
#     strAskForCommand += 'r: Reset\n'
#     strAskForCommand += 'a: Activate\n'
#     strAskForCommand += 'c: Close\n'
#     strAskForCommand += 'o: Open\n'
#     strAskForCommand += 'b: Basic mode\n'
#     strAskForCommand += 'p: Pinch mode\n'
#     strAskForCommand += 'w: Wide mode\n'
#     strAskForCommand += 's: Scissor mode\n'
#     strAskForCommand += '(0-255): Go to that position\n'
#     strAskForCommand += 'f: Faster\n'
#     strAskForCommand += 'l: Slower\n'
#     strAskForCommand += 'i: Increase force\n'
#     strAskForCommand += 'd: Decrease force\n'

#     strAskForCommand += '-->'

#     return raw_input(strAskForCommand)

    

# def callback(data):
    
#     for x in range(len(data.name)):
#         if(data.name[x]=="finger_1_joint_1"):
#             C=data.position[x]*(255/1.22)
#             command.rPRC = int(C)
#         elif(data.name[x]=="finger_2_joint_1"):
#             B=data.position[x]*(255/1.22)
#             command.rPRB = int(B)
#         elif(data.name[x]=="finger_middle_joint_1"):
#             A=data.position[x]*(255/1.22)
#             command.rPRA = int(A)
#         pub.publish(command)


def main():
    """Main loop which requests new commands and publish them on the Robotiq3FGripperRobotOutput topic."""


    # command.rACT = 0
    # pub.publish(command)
    # rospy.sleep(1) 
    # command.rACT = 1
    # command.rMOD = 0
    # command.rGTO = 1
    # # command.rSPA = 255
    # # command.rFRA = 150
    # pub.publish(command)
    # rospy.sleep(10) 
    
    # command.rICF = 1
    # pub.publish(command)
    # rospy.sleep(1)

    # while not rospy.is_shutdown():
    #     rospy.Subscriber('joint_states', JointState, callback)

    #     rospy.spin()
    t = Robotiq3FGripperROSConnector()
    t.run()

if __name__ == '__main__':
    main()
