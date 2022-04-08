
import py_pubsub.subscriber_member_function as pubsub
import rclpy
import sys
import time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from foxy_nav2_navigator.nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
# sudo apt install ros-foxy-tf-transformations
# sudo pip3 install transforms3d
# q = tf_transformations.quaternion_from_euler(r, p, y)
# r, p, y = tf_transformations.euler_from_quaternion(quaternion)


#import follow-me

if __name__ == '__main__':
    pubsub.main()





    



