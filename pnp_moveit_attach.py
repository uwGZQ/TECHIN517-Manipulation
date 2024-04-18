#!/usr/bin/env python
import math
import tf.transformations as tf
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
import copy
import os
from kortex_driver.srv import *
from kortex_driver.msg import *

CUBE_LOCATIONS = []

class PickAndPlace(object):
    """PickAndPlace"""
    def __init__(self):
        # TO AVOID ISSUES WITH NAMESPACES
        os.system("export ROS_HOSTNAME='/my_gen3_lite/'")
        # Initialize the node
        super(PickAndPlace, self).__init__()
        moveit_commander.roscpp_initialize("/my_gen3_lite/joint_states")
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param("/my_gen3_lite/is_gripper_present", False)
            print("Gripper found")
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param("/my_gen3_lite/gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                print("NO gripper found")
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param("/my_gen3_lite/degrees_of_freedom", 6)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("/my_gen3_lite/robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3_lite/")
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite/")
            self.display_trajectory_publisher = rospy.Publisher('/my_gen3_lite/move_group/display_planned_path',
                                                                moveit_msgs.msg.DisplayTrajectory,
                                                                queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, robot_description="/my_gen3_lite/robot_description", ns="/my_gen3_lite/")

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
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
        (success_flag, planned_path1, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(planned_path1, wait=True)

    def reach_joint_angles(self, config_pose, tolerance):
        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions:
            rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        arm_group.set_joint_value_target(config_pose)

        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions:
            rospy.loginfo(p)
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
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.arm_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.00)  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
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
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
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

    def add_object_to_scene(self, size, pos, name):
        # Define the pose of the box
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.robot.get_planning_frame()
        pose.pose.orientation.w = 1.0  # No rotation
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]

        # Add the box as a collision object to the planning scene
        self.scene.add_box(name, pose, size=size)
        

    def attach_object_to_gripper(self, obj_name, touch_links):
        # Attach the object to the end effector link
        eef_link = "end_effector_link"
        self.scene.attach_box(eef_link, obj_name, touch_links=touch_links)

        # Wait for the scene to update
        if not self.wait_for_scene_update(obj_name):
            #rospy.logerr("Failed to attach the object to the gripper")
            return False

        return True

    def detach_object_from_gripper(self, obj_name):
        # Detach the object from the end effector link
        eef_link = "end_effector_link"
        self.scene.remove_attached_object(eef_link, name=obj_name)

        # Wait for the scene to update
        if not self.wait_for_scene_update(obj_name, timeout=2):
            #rospy.logerr("Failed to detach the object from the gripper")
            return False

        return True

    def wait_for_scene_update(self, obj_name, timeout=2):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the object is still attached to the end effector
            attached_objects = self.scene.get_attached_objects([obj_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the object is in the scene
            is_known = obj_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if not is_attached and not is_known:
                return True

            # Sleep to give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning, then we timed out
        return False

    def clean(self):
        self.scene.remove_world_object()
        pass


def main():
    pnp = PickAndPlace()

    # For testing purposes
    success = pnp.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Added cartesian path functions")

        waypoints = []
        #print(pnp.get_cartesian_pose())

        pnp.clean()
        pnp.add_object_to_scene((0.4, 0.3, 0.01), (0.342, -0.4, 0.15), 'cube')
        pnp.add_object_to_scene((1, 1, 0.008), (0, -0.25, -0.025), 'floor')
        #pnp.add_object_to_scene((0.05, 0.4, 0.1), (0, -0.25, 0.025), 'wall')
        #pnp.detach_object_from_gripper('cube')
        # open gripper
        time.sleep(1)
        pnp.reach_named_position("home")

        pnp.activate_gripper(0)
        # Get the pose of the cube in the MoveIt reference frame
        cube_pose = pnp.scene.get_object_poses(["cube"])["cube"]

        # Define the desired pose for the arm
        arm_pose = geometry_msgs.msg.Pose()

        # Assuming the object's pose is already in the arm's reference frame,
        # set the arm's pose directly using the object's pose
        arm_pose.position.x = 0.342
        arm_pose.position.y = -0.3
        arm_pose.position.z = 0.15

        arm_pose.orientation = cube_pose.orientation
        
        waypoint1 = geometry_msgs.msg.Pose()
        waypoint1.position.x = 0.342
        waypoint1.position.y = -0.2
        waypoint1.position.z = 0.14
        
        #waypoints.append(waypoint1)
        # Define the orientation in Euler angles (in degrees)
        roll = 90   # Rotation around the x-axis
        pitch = 90 # Rotation around the y-axis
        yaw = 0   # Rotation around the z-axis

        # Convert Euler angles to quaternion
        quat = tf.quaternion_from_euler(math.radians(roll),
                                                math.radians(pitch),
                                                math.radians(yaw))

        # Set the orientation of the pose
        arm_pose.orientation.x = quat[0]
        arm_pose.orientation.y = quat[1]
        arm_pose.orientation.z = quat[2]
        arm_pose.orientation.w = quat[3]
        # Reach the desired Cartesian pose

        waypoint1.orientation.x = quat[0]
        waypoint1.orientation.y = quat[1]
        waypoint1.orientation.z = quat[2]
        waypoint1.orientation.w = quat[3]
        #waypoints.append(arm_pose)
        #waypoints.append(waypoint1)
        
        #plan = pnp.plan_cartesian_path(waypoints)
        pnp.reach_cartesian_pose(waypoint1, tolerance=0.05, constraints=None)
        #pnp.execute_plan(plan)
        time.sleep(1)
        
        
        
        pnp.reach_cartesian_pose(arm_pose, tolerance=0.01, constraints=None)

        
        time.sleep(1)
        pnp.attach_object_to_gripper('cube', touch_links=["right_finger_dist_link", "left_finger_dist_link"])
        time.sleep(0.4)
        pnp.activate_gripper(0.85)
        
        time.sleep(1)
        
        
        final_pose = geometry_msgs.msg.Pose()
        final_pose.position.x = -0.30  # X coordinate
        final_pose.position.y = 0.2  # Y coordinate
        final_pose.position.z = 0.4  # Z coordinate

        # Quaternion orientation (x, y, z, w)
        final_pose.orientation.x = 0 # X coordinate
        final_pose.orientation.y = 1  # Y coordinate
        final_pose.orientation.z = 0  # Z coordinate
        final_pose.orientation.w = 0  # Z coordinate
        
        waypoint2 = geometry_msgs.msg.Pose()
        waypoint2.position.x = -0.1
        waypoint2.position.y = -0.3
        waypoint2.position.z = 0.4
        waypoint2.orientation.x = 0
        waypoint2.orientation.y = 1
        waypoint2.orientation.z = 0
        waypoint2.orientation.w = 0
        
        #waypoints = []
        #waypoints.append(final_pose)
        #waypoints.append(waypoint1)
        
        #pnp.plan_cartesian_path(waypoints)
        #plan = pnp.plan_cartesian_path(waypoints)
        
        #pnp.execute_plan(plan)
        pnp.reach_cartesian_pose(waypoint1, tolerance=0.01, constraints=None)
        time.sleep(1)
        #pnp.reach_cartesian_pose(waypoint2, tolerance=0.05, constraints=None)
        #time.sleep(1)

        pnp.reach_cartesian_pose(final_pose, tolerance=0.02, constraints=None)
        time.sleep(2)
        pnp.activate_gripper(0)
        
        pnp.detach_object_from_gripper('cube')
        time.sleep(0.5)
        pnp.reach_named_position("home")

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == '__main__':
    main()

