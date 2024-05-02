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
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

CUBE_LOCATIONS = []

class PickAndPlace(object):
    """PickAndPlace"""
    def __init__(self):
        # TO AVOID ISSUES WITH NAMESPACES
        os.system("export ROS_HOSTNAME='/my_gen3_lite/'")
        # Initialize the node
        super(PickAndPlace, self).__init__()
        moveit_commander.roscpp_initialize("/my_gen3_lite/joint_states")
        rospy.init_node('pnp')

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
        if constraints:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)
        print(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        success = arm_group.go(wait=True)

        # Clear constraints to avoid affecting other movements
        arm_group.clear_path_constraints()
        return success

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
        # Ensure there are points in the trajectory to execute
        if not plan.joint_trajectory.points:
            rospy.logerr("Plan contains no points.")
            return False

        # Use the MoveIt function to compute the time parameterization for the plan
        move_group = self.arm_group
        if not move_group.compute_cartesian_path(plan.joint_trajectory, True):
            rospy.logerr("Failed to compute time parameterization.")
            return False

        # Execute the trajectory
        success = move_group.execute(plan, wait=True)
        if not success:
            rospy.logerr("Failed to execute plan.")
        return success
    
    def log_trajectory_points(self, plan):
        rospy.loginfo("Logging trajectory points and their timestamps:")
        for i, point in enumerate(plan.joint_trajectory.points):
            rospy.loginfo("Point {}: time_from_start = {:.2f} sec".format(i, point.time_from_start.to_sec()))



    def reach_gripper_position(self, relative_position):
        if not self.gripper_group:
            rospy.logerr("Gripper group is not initialized.")
            return False

        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            rospy.loginfo("Gripper moved")
            return val
        except Exception as e:
            rospy.logerr("Failed to move gripper: %s", str(e))
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
        rospy.sleep(0.2)
        rospy.loginfo(f"{name} added to the scene at {pos} with size {size}")

    def attach_object_to_gripper(self, obj_name, eef_link, touch_links):
        known_objects = self.scene.get_known_object_names()
        print(known_objects)
        if obj_name not in known_objects:
            rospy.logerr(f"Object {obj_name} does not exist in the planning scene.")
            return False

        #eef_link = "left_finger_dist_link"
        self.scene.attach_box(eef_link, obj_name, touch_links=touch_links)
        rospy.sleep(1)
        return self.wait_for_scene_update(obj_name, is_known=True, is_attached=True)

    def detach_object_from_gripper(self, obj_name):
        # Detach the object from the end effector link
        eef_link =  "tool_frame" #"left_finger_dist_link",
        self.scene.remove_attached_object(eef_link, name=obj_name)

        # Wait for the scene to update
        if not self.wait_for_scene_update(obj_name, timeout=2):
            #rospy.logerr("Failed to detach the object from the gripper")
            return False

        return True

    def wait_for_scene_update(self, obj_name, is_known=False, is_attached=False, timeout=4):
        start = rospy.get_time()
        while rospy.get_time() - start < timeout and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([obj_name])
            is_currently_attached = len(attached_objects.keys()) > 0
            is_currently_known = obj_name in self.scene.get_known_object_names()
            
            if (is_known == is_currently_known) and (is_attached == is_currently_attached):
                return True
            
            rospy.sleep(0.1)
        return False


    def clean(self):
        self.scene.remove_world_object()
        pass
    def clear(self, obj_name):
        self.scene.remove_world_object(obj_name)
        pass
    def print_allowed_collision_matrix(self):
        rospy.wait_for_service('/get_planning_scene')
        try:
            get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
            request = GetPlanningSceneRequest()
            request.components.components = request.components.ALLOWED_COLLISION_MATRIX
            response = get_planning_scene(request)

            acm = response.scene.allowed_collision_matrix
            print("Allowed Collision Matrix:")
            print("Default Entry Values: ", acm.default_entry_values)
            print("Default Entry Names: ", acm.default_entry_names)
            print("Entries:")
            for i, entry in enumerate(acm.entry_names):
                print("  Entry Name: ", entry)
                print("  Allowed Collisions: ", acm.entry_values[i].enabled)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # def set_collision(self, object_name, allowed_collision):
    #     """
    #     Modify collision settings for a specified object.
        
    #     :param object_name: Name of the object to modify.
    #     :param allowed_collision: Boolean, set to True if collision should be ignored.
    #     """
    #     # Get the current allowed collision matrix
    #     acm = self.scene.get_allowed_collision_matrix()
        
    #     # Update the collision matrix for the object
    #     if allowed_collision:
    #         acm.default_entry_names.append(object_name)
    #         acm.default_entry_values.append(True)
    #     else:
    #         if object_name in acm.default_entry_names:
    #             idx = acm.default_entry_names.index(object_name)
    #             acm.default_entry_values[idx] = False
        
    #     # Apply the updated collision matrix
    #     self.scene.apply_collision_matrix(acm) 
 

    # Rest of your function code...
    def perform_pick_and_place(self, object_name, pick_and_place_params):
        

        params = pick_and_place_params.get(object_name, {})
        if not params:
            raise ValueError(f"No custom parameters found for object {object_name}")
        pick_pose = geometry_msgs.msg.Pose()
        place_pose = geometry_msgs.msg.Pose()
        # Use the custom parameters
        pick_position = params['pick_pose']
        place_position = params['place_pose']
        approach_distance = params.get('approach_distance')
        approach_axis = params.get('approach_axis')
        retract_distance = params.get('retract_distance')
        rpy = params.get("rpy")
        elink = params.get("elink")
        grip = params.get("grip")
        attached_objects = self.scene.get_attached_objects()
    
        if attached_objects:
            rospy.loginfo("Detaching all currently attached objects.")
            for obj_name in attached_objects.keys():
                self.detach_object_from_gripper(obj_name)
                rospy.loginfo(f"Detached {obj_name} from gripper.")
        else:
            rospy.loginfo("No objects are currently attached to the gripper.")

        quat = tf.quaternion_from_euler(math.radians(rpy[0]), math.radians(rpy[1]), math.radians(rpy[2]))
        print(quat)
        pick_pose.position.x = pick_position[0]
        pick_pose.position.y = pick_position[1]
        pick_pose.position.z = pick_position[2]
        pick_pose.orientation.x = quat[0]
        pick_pose.orientation.y = quat[1]
        pick_pose.orientation.z = quat[2]
        pick_pose.orientation.w = quat[3]

        # For simplicity, we'll assume the place_pose is just a translation without rotation
        place_pose.position.x = place_position[0]
        place_pose.position.y = place_position[1]
        place_pose.position.z = place_position[2]
        place_pose.orientation.x = 0
        place_pose.orientation.y = 1
        place_pose.orientation.z = 0
        place_pose.orientation.w = 0
        
        
        # place_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z, pick_pose.orientation.w = quat
        # place_pose.orientation.x, place_pose.orientation.y, place_pose.orientation.z, place_pose.orientation.w = [0, 1, 0, 0]
        
        orientation_constraint = self.create_orientation_constraint()
        
        # Approach phase
        #self.detach_object_from_gripper(object_name)
        approach_pose = copy.deepcopy(pick_pose)
        current_value = getattr(approach_pose.position, approach_axis)
        if current_value < 0:
            setattr(approach_pose.position, approach_axis, getattr(approach_pose.position, approach_axis) + approach_distance)
        else:
            setattr(approach_pose.position, approach_axis, getattr(approach_pose.position, approach_axis) - approach_distance)
        self.activate_gripper(0) 
        self.reach_cartesian_pose(approach_pose, tolerance=0.01, constraints=None)

        # Pick phase
        
        self.reach_cartesian_pose(pick_pose, tolerance=0.01, constraints=None)
        print('reaching to pick pose')   
        time.sleep(0.5)
        self.attach_object_to_gripper(object_name,eef_link=elink, touch_links=["right_finger_dist_link", "left_finger_dist_link", "tool_frame",'right_finger_prox_link', 'left_finger_prox_link'])
        self.activate_gripper(grip)  
        time.sleep(1)
        
        
        pick_pose.position.z += 0.03
        #quat = tf.quaternion_from_euler(math.radians(90), math.radians(90), math.radians(0))
        #pick_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z, pick_pose.orientation.w = quat
        self.reach_cartesian_pose(pick_pose, tolerance=0.01, constraints=None)
    
           
        
        # Retract phase
        retract_pose = copy.deepcopy(pick_pose)
        #retract_pose.position.approach_axis += retract_distance
        latest_value = getattr(retract_pose.position, approach_axis)
        if latest_value < 0:
            setattr(retract_pose.position, approach_axis, getattr(retract_pose.position, approach_axis) + retract_distance)
        else:
            setattr(retract_pose.position, approach_axis, getattr(retract_pose.position, approach_axis) - retract_distance)
        #self.activate_gripper(0) 
        
        
        self.reach_cartesian_pose(retract_pose, tolerance=0.01, constraints=None)
        #self.activate_gripper(0.85)
        constraints = moveit_msgs.msg.Constraints()
        constraints.orientation_constraints.append(orientation_constraint)

        # Move to place position
        approach_place_pose = copy.deepcopy(place_pose)
        approach_place_pose.position.z += approach_distance
        
        
        self.reach_cartesian_pose(approach_place_pose, tolerance=0.05, constraints=constraints)
        
        # Place phase
        self.reach_cartesian_pose(place_pose, tolerance=0.05, constraints=None)
        self.activate_gripper(0) 
        self.detach_object_from_gripper(object_name)
        
        # Final retract phase
        final_retract_pose = copy.deepcopy(place_pose)
        final_retract_pose.position.z += retract_distance
        self.reach_cartesian_pose(final_retract_pose, tolerance=0.01, constraints=None)
        self.clear(object_name)
        self.arm_group.clear_path_constraints()


    def create_orientation_constraint(self):
        # Convert Euler angles to quaternion
        quat = tf.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(180))

        # Create orientation constraint
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header.frame_id = self.arm_group.get_planning_frame()
        orientation_constraint.link_name = self.arm_group.get_end_effector_link()
        print(orientation_constraint)
        #orientation_constraint.orientation.x = 0
        orientation_constraint.orientation.y = 1
        #orientation_constraint.orientation.z = 0
        #orientation_constraint.orientation.w = 0
        orientation_constraint.absolute_x_axis_tolerance = 0.5
        orientation_constraint.absolute_y_axis_tolerance = 0.5
        orientation_constraint.absolute_z_axis_tolerance = 0.5
        orientation_constraint.weight = 1

        return orientation_constraint


def main():
    pnp = PickAndPlace()
    if pnp.is_init_success:
        rospy.loginfo("Node initialization successful, proceeding with operations.")

        
        waypoints = []
        #print(pnp.get_cartesian_pose())
        pnp.clean()
        
        

        #Environment Dynamic Object
        pnp.add_object_to_scene((0.1, 0.2, 0.135), (0.221, -0.4, 0.075), 'stand1')
        pnp.add_object_to_scene((0.1, 0.2, 0.135), (0.463, -0.4, 0.075), 'stand2')
        pnp.add_object_to_scene((1, 2, 0.049), (0.5, 0, -0.025), 'floor')

        #Environment Dynamic Object
        pnp.add_object_to_scene((0.05, 0.05, 0.150), (0.363, 0.1, 0.075), 'bottle')
        pnp.add_object_to_scene((0.3, 0.19, 0.001), (0.342, -0.4, 0.165), 'blanket1')
        pnp.add_object_to_scene((0.05, 0.05, 0.05), (0.342, 0, 0.025), 'sock')
        pnp.add_object_to_scene((0.3, 0.19, 0.001), (0.342, -0.4, 0.12), 'blanket2')
        #pnp.add_object_to_scene((1, 2, 0.049), (0.5, 0, 0.9), 'roof')
        #pnp.add_object_to_scene((0.05, 0.4, 0.1), (0, -0.25, 0.025), 'wall')
        #pnp.detach_object_from_gripper('cube')
        # open gripper
        time.sleep(1)
        pnp.reach_named_position("home")

        #pnp.activate_gripper(0)
        planning_scene = PlanningScene()

        known_objects = pnp.scene.get_known_object_names()
        print("Known collision objects in the scene:")
        for obj in known_objects:
            print(obj)
        # Get the pose of the objects in the MoveIt reference frame
        blanket1_pose = pnp.scene.get_object_poses(["blanket1"])["blanket1"]
        print(blanket1_pose)
        bottle_pose = pnp.scene.get_object_poses(["bottle"])["bottle"]
        print(bottle_pose)
        sock_pose = pnp.scene.get_object_poses(["sock"])["sock"]
        print(bottle_pose)
        blanket2_pose = pnp.scene.get_object_poses(["blanket2"])["blanket2"]
        print(blanket2_pose)
        # Perform pick and place for the blanket
        pick_pose = geometry_msgs.msg.Pose()
        place_pose = geometry_msgs.msg.Pose()
        
        

        pick_and_place_params = {
        'blanket1': {'pick_pose': (blanket1_pose.position.x, blanket1_pose.position.y + 0.08, blanket1_pose.position.z + 0.04), 
                    'place_pose': (-0.30, 0.2, 0.2), 
                    'approach_distance': 0.1, 
                    'approach_axis': 'y',
                    'retract_distance': 0.1,
                    'rpy': [-0.04, 72.48, -90.03],
                    'elink' : 'tool_frame',
                    'grip' : 0.9},
        'bottle': {'pick_pose': (bottle_pose.position.x, bottle_pose.position.y, bottle_pose.position.z + 0.05), 
                    'place_pose': (-0.30, 0.2, 0.2), 
                    'approach_distance': 0.15, 
                    'approach_axis': 'z',
                    'retract_distance': 0.2,
                    'rpy': (180, 0, 180),
                    'elink' : 'tool_frame',
                    'grip' : 0.75},
        'blanket2': {'pick_pose': (blanket2_pose.position.x, blanket2_pose.position.y + 0.08, blanket2_pose.position.z + 0.04), 
                    'place_pose': (-0.30, 0.2, 0.2), 
                    'approach_distance': 0.1, 
                    'approach_axis': 'y',
                    'retract_distance': 0.1,
                    'rpy': (-0.04, 72.48, -90.03),
                    'elink' : 'tool_frame',
                    'grip' : 0.9},
        }
        
        

        # # Define poses
        # pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = (blanket_pose.position.x, blanket_pose.position.y + 0.08, blanket_pose.position.z + 0.04)
        # place_pose.position.x, place_pose.position.y, place_pose.position.z = (-0.30, 0.2, 0.2)
        # quat = tf.quaternion_from_euler(math.radians(-0.04), math.radians(72.48), math.radians(-90.03))
        # pick_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z, pick_pose.orientation.w = quat
        # place_pose.orientation.x, place_pose.orientation.y, place_pose.orientation.z, place_pose.orientation.w = [0, 1, 0, 0]
        
        # Execute pick and place
        pnp.perform_pick_and_place('blanket1', pick_and_place_params)
        pnp.reach_named_position("home")

        # # Blanket 2 PnP
        # pnp.add_object_to_scene((0.3, 0.19, 0.001), (0.342, -0.4, 0.1), 'blanket')
        # blanket_pose = pnp.scene.get_object_poses(["blanket"])["blanket"]
        # pick_pose = geometry_msgs.msg.Pose()
        # place_pose = geometry_msgs.msg.Pose()
        # # Define poses
        # pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = blanket_pose.position.x, blanket_pose.position.y + 0.08, blanket_pose.position.z + 0.04
        # place_pose.position.x, place_pose.position.y, place_pose.position.z = -0.30, 0.2, 0.2
        # quat = tf.quaternion_from_euler(math.radians(-0.04), math.radians(72.48), math.radians(-90.03))
        # pick_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z, pick_pose.orientation.w = quat
        # place_pose.orientation.x, place_pose.orientation.y, place_pose.orientation.z, place_pose.orientation.w = [0, 1, 0, 0]
        
        # Execute pick and place
        pnp.perform_pick_and_place('blanket2', pick_and_place_params)
        pnp.reach_named_position("home")

        pnp.perform_pick_and_place('bottle', pick_and_place_params)
        pnp.reach_named_position("home")

        rospy.spin()


if __name__ == '__main__':
    main()

