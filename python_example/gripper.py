from __future__ import print_function

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

import actionlib
import rospy

rospy.init_node('gripper_test_client')

action_name = rospy.get_param('~action_name', 'command_robotiq_action')
print(action_name)

robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
robotiq_client.wait_for_server()
print("Client test: Starting sending goals")

goal = CommandRobotiqGripperGoal()
goal.emergency_release = False
goal.stop = False
goal.position = 0.01
goal.speed = 0.5
goal.force = 5.0

# Sends the goal to the gripper.
robotiq_client.send_goal(goal)
# Block processing thread until gripper movement is finished, comment if waiting is not necesary.
print(robotiq_client.wait_for_result())
rospy.sleep(rospy.Duration(0.5))

goal = CommandRobotiqGripperGoal()
goal.emergency_release = False
goal.stop = False
goal.position = 0.1
goal.speed = 1.0 #0.5
goal.force = 5.0

# Sends the goal to the gripper.
robotiq_client.send_goal(goal)
# Block processing thread until gripper movement is finished, comment if waiting is not necesary.
print(robotiq_client.wait_for_result())
rospy.sleep(rospy.Duration(0.5))

print(robotiq_client.get_result())
rospy.sleep(rospy.Duration(0.5))
