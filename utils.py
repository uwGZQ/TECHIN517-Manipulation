import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint
import tf

def set_pose(position, orientation):
    pose = geometry_msgs.msg.Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose

def average_pose(pose1, pose2, z_delta=0.02):
    # Create a new pose object for the middle
    middle_pose = geometry_msgs.msg.Pose()

    # Average the positions
    middle_pose.position.x = (pose1.position.x + pose2.position.x) / 2
    middle_pose.position.y = (pose1.position.y + pose2.position.y) / 2
    middle_pose.position.z = (pose1.position.z + pose2.position.z) / 2 + z_delta

    # Average the orientations (simple average, not slerp)
    middle_pose.orientation.x = (pose1.orientation.x + pose2.orientation.x) / 2
    middle_pose.orientation.y = (pose1.orientation.y + pose2.orientation.y) / 2
    middle_pose.orientation.z = (pose1.orientation.z + pose2.orientation.z) / 2
    middle_pose.orientation.w = (pose1.orientation.w + pose2.orientation.w) / 2

    return middle_pose


def rpy_to_quaternion(roll, pitch, yaw):
    quaternion = Quaternion()
    # Convert roll, pitch, and yaw to quaternion
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)

def add_orientation_constraint(roll, pitch, yaw):
    # Create an OrientationConstraint
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = "base_link"  # Set the reference frame
    orientation_constraint.link_name = "gripper_link"  # Set the link to which the constraint applies
    orientation_constraint.orientation = rpy_to_quaternion(roll, pitch, yaw)

    orientation_constraint.orientation.w = 1
    orientation_constraint.absolute_x_axis_tolerance = 0.2
    orientation_constraint.absolute_y_axis_tolerance = 0.2
    orientation_constraint.absolute_z_axis_tolerance = 3.14
    orientation_constraint.weight = 1.0  # Set the weight of the constraint

    # Create a Constraints message and add the OrientationConstraint
    constraints = Constraints()
    constraints.orientation_constraints.append(orientation_constraint)

    return constraints
