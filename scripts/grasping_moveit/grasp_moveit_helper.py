import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix


def create_pose_stamped(ref_frame, stamp, pos, orientation):
    posest = PoseStamped()
    posest.header.frame_id = ref_frame
    posest.header.stamp = stamp
    posest.pose.position.x = pos[0]
    posest.pose.position.y = pos[1]
    posest.pose.position.z = pos[2]
    quat = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    posest.pose.orientation.x = quat[0]
    posest.pose.orientation.y = quat[1]
    posest.pose.orientation.z = quat[2]
    posest.pose.orientation.w = quat[3]
    return posest


def create_alt(posest, orientation):
    posest_new = PoseStamped()
    posest_new.header.frame_id = posest.header.frame_id
    posest_new.header.stamp = rospy.Time.now()
    posest_new.pose.position = posest.pose.position
    quat = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    posest_new.pose.orientation.x = quat[0]
    posest_new.pose.orientation.y = quat[1]
    posest_new.pose.orientation.z = quat[2]
    posest_new.pose.orientation.w = quat[3]
    return posest_new


def determine_pos_orient(pos, orient, distance_vector):
    # ensure np
    pos = np.array(pos)
    orient = np.array(orient)
    distance_vector = np.array(distance_vector)

    # relation between camera vector space and cobot vector space
    rotation_matrix = np.array([[0, -1, 0],[-1, 0, 0], [0, 0, -1]])
    
    # translate and rotate position and orientation
    pos_new = rotation_matrix @ pos + distance_vector
    orient_new = rotation_matrix @ orient

    return pos_new, orient_new


def add_plane(scene):
    scene.remove_world_object("table_plane")

    plane_size = 10  # Size of the plane (width and depth)
    plane_pose = PoseStamped()
    plane_pose.header.frame_id = "joint1"
    plane_pose.pose.orientation.w = 1.0  # Set to a neutral orientation
    plane_pose.pose.position.x = 0.0  # Centered
    plane_pose.pose.position.y = 0.0
    plane_pose.pose.position.z = 0.04  # Slightly above the robot's base

    scene.add_box("table_plane", plane_pose, size=(10, 10, 0.01)) 
    rospy.sleep(2)


def init_mycobot():
    baud = rospy.get_param("~baud", 115200)
    mc = None
    try:
        port = rospy.get_param("~port", "/dev/ttyACM0")
        mc = MyCobot(port, baud)
    except Exception:
        rospy.loginfo("could not find ttyACM0")
        try:
            port = rospy.get_param("~port", "/dev/ttyACM1")
            mc = MyCobot(port, baud)
        except Exception:
            rospy.loginfo("could not find ttyACM1")
    return mc