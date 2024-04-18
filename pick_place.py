#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3_lite'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3_lite

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import copy
import os
from kortex_driver.srv import *
from kortex_driver.msg import *

from utils import *


class PickAndPlace(object):
  """PickAndPlace"""
  def __init__(self):
    # TO AVOID ISSUES WITH NAMESPACES
    os.system ("export ROS_HOSTNAME='/my_gen3_lite/'")
    # Initialize the node
    super(PickAndPlace, self).__init__()
    # moveit_commander.roscpp_initialize("/my_gen3_lite/joint_states")
    rospy.init_node('example_move_it_trajectories')
    moveit_commander.roscpp_initialize(sys.argv)

    try:
      self.is_gripper_present = rospy.get_param("/my_gen3_lite/is_gripper_present", False)
      print("Gripper found")
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param("/my_gen3_lite/gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        print("NO gripper found")
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param("/my_gen3_lite/degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("/my_gen3_lite/robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3_lite/")
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, robot_description = "/my_gen3_lite/robot_description", ns="/my_gen3_lite/")
      self.display_trajectory_publisher = rospy.Publisher('/my_gen3_lite/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, robot_description = "/my_gen3_lite/robot_description",ns="/my_gen3_lite/")

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True
    send_gripper_command_full_name = '/' + rospy.get_param('~robot_name', 'my_gen3_lite') + '/base/send_gripper_command'
    rospy.wait_for_service(send_gripper_command_full_name)
    self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, planned_path1, planning_time, error_code)= arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, config_pose, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    arm_group.set_joint_value_target(config_pose)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)
    print(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def plan_cartesian_path(self, waypoints, scale=1):
    move_group = self.arm_group

    # waypoints = []

    # wpose = move_group.get_current_pose().pose
    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0  # waypoints to follow  # eef_step
    )  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    move_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      print("Gripper moved")
      return val
    except:
      print("Failed to move gripper")
      return False 
    
  def activate_gripper(self, gripper_position):
    rospy.loginfo("Activating gripper using kortex driver.")
    try:
      req = SendGripperCommandRequest()
      finger = Finger()
      finger.finger_identifier = 0
      finger.value = gripper_position
      req.input.gripper.finger.append(finger)
      req.input.mode = GripperMode.GRIPPER_POSITION

      self.send_gripper_command(req)
    except rospy.ServiceException:
      rospy.logerr("Failed to activate the gripper")
    else:
      rospy.loginfo("Gripper activated successfully")
      return True

  def add_box(self, size, pos, name):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = self.robot.get_planning_frame()
    pose.pose.orientation.w = 0
    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    pose.pose.position.z = pos[2]
    self.scene.add_box(name, pose, size=size)
    
  

def main(target_pose, final_pose, gripper_ratio):
  pnp = PickAndPlace()

  # For testing purposes
  success = pnp.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass

  if success:

    rospy.loginfo("Added cartesian path functions")
    
    middle_point = average_pose(target_pose, final_pose)
    
    waypoints = [target_pose]
    #print(pnp.get_cartesian_pose())
    # pnp.add_box((0.05, 0.6, 0.25), (0.3, -0.3, 0.125), 'wall')
    # pnp.add_box((0.075, 0.075, 0.25), (0.2, -0.2, 0.125), 'stack')
    pnp.add_box((0.7, 0.7, 0.05), (0.25, -0.25, -0.05), 'floor')
    
    # open gripper
    pnp.reach_named_position("home")
    
    pnp.activate_gripper(0)
    
    # Way point method
    # plan, fraction = pnp.plan_cartesian_path(waypoints)
    # rospy.loginfo(plan)
    # rospy.loginfo(fraction)
    # pnp.execute_plan(plan)

    # Reach method
    start_time = time.time()
    pnp.reach_cartesian_pose(target_pose, tolerance=0.015, constraints=None)
    time.sleep(0.1)
    pnp.activate_gripper(gripper_ratio)
    time.sleep(0.1)
    
    # --Way point--
    # pnp.reach_cartesian_pose(middle_point, tolerance=0.02, constraints=None)
    # time.sleep(0.1)

    pnp.reach_cartesian_pose(final_pose, tolerance=0.02, constraints=None)
    pnp.activate_gripper(0)
    end_time = time.time()
    pnp.reach_named_position("home")
    rospy.loginfo(f"Time: {end_time - start_time}")
    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  picked_obj = 'bottle'
  # transformation equation
  # x′ = 0.05208x
  # y′ = 0.058y+0.003
  z_axis = 0
  if picked_obj == 'sock':
    # (8.64, -3.5)
    z_axis = 0.02
    target_position = (0.45, -0.20, z_axis)
    target_orientation = (-0.00013, 0.999829, 0.002574, 0.018335) # Quaternion orientation (x, y, z, w)
    gripper_ratio = 0.55
  elif picked_obj == 'bottle':
    z_axis = 0.125
    target_position = (0.45, -0.20, z_axis)
    target_orientation = (-0.00013, 0.999829, 0.002574, 0.018335) # Quaternion orientation (x, y, z, w)
    gripper_ratio = 0.80

  target_pose = set_pose(target_position, target_orientation)

  # (0, -2.3)
  final_position = (0.26, -0.345, z_axis)
  final_orientation = (-0.00013, 0.999829, 0.002574, 0.018335)
  final_pose = set_pose(final_position, final_orientation)

  main(target_pose, final_pose, gripper_ratio)

    # end effector pointed down
    # target_pose.orientation.x = -0.00013  # X coordinate
    # target_pose.orientation.y = 0.999829  # Y coordinate
    # target_pose.orientation.z = 0.002574  # Z coordinate
    # target_pose.orientation.w = 0.018335  # Z coordinate