

import rclpy
import sys
import time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
# sudo apt install ros-foxy-tf-transformations
# sudo pip3 install transforms3d
# q = tf_transformations.quaternion_from_euler(r, p, y)
# r, p, y = tf_transformations.euler_from_quaternion(quaternion)


#import follow-me
#from py_pubsub.py_pubsub.subscriber_member_function import MinimalSubscriber, main

#import navigation2 libraries
from foxy_nav2_navigator.nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

# code to get robot position info and transform them into the map referential


#------------------------------------------------------------------------------------------------------
#got ros1 (kinetic) function to transform base_footprint to map coordinates
#need to get it for ros2 to get the initial pose in the map referential and go back to it after follow_me has ended.
#problems when importing follow_me code (does not acces it, module not found)
#------------------------------------------------------------------------------------------------------
def current_time():
    return round(time.time()*1000)




#set initial pose on map

#launch follow me
#main()
#break follow-me loop after (3sec?) of non following



# si vitesse mvt == 0 pour 3 sec -> breack
#amcl get pose service
# code
#launch follow me
# every time a non null speed order is given t =timt.time
# if time - t ==3 break
#go to pose code


#go back to initial pose when stop following smtg




while(1): #a implementer dans le follow me qui fait office de boucle
    
    timestamp = current_time()
    position actuel = p
    if new p == p and t - old t >=3
        break ou reste code nav
    

    



