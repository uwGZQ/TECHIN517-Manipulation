import geometry_msgs.msg

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