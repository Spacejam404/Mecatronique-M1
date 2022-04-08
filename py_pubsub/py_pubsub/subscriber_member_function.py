# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from time import time

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from foxy_nav2_navigator.nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.lin_vel = 0
        self.rot_vel = 0
        self.publisher= self.create_publisher(Twist,'cmd_vel' , 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def goTo(self):
        navigator = BasicNavigator()
        navigator.changeMap('~$HOME/anthony/B2__map.yaml')
        navigator.waitUntilNav2Active()
        goal_poses = []
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 9.44
        goal_pose.pose.position.y = -8.45
        goal_pose.pose.position.z = 0.20
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_poses.append(goal_pose)

        navigator.goThroughPoses(goal_poses)
    
        result = navigator.getResult()
    
        if result == NavigationResult.SUCCEEDED:
            navigator.lifecycleShutdown()
            return'Goal succeeded!'
        elif result == NavigationResult.CANCELED:
            return'Goal was canceled!'
        elif result == NavigationResult.FAILED:
            return'Goal failed!'
        else:
            return'Goal has an invalid return status!'

   

    def listener_callback(self, msg):
       # self.get_logger().info ('I heard: "%s"' % msg)
        self.get_logger().info ('Distances : ' "%s" %msg.ranges)
        #self.get_logger().info ('Range max : ' "%s" %msg.ranges)
        self.get_logger().info ('angle min : ' "%s" %msg.angle_min)
        self.get_logger().info ('angle max : ' "%s" %msg.angle_max)    
        self.get_logger().info ('temps entre 2 scans : ' "%s" %msg.scan_time)  
        self.get_logger().info ('time_increment  : ' "%s" %msg.time_increment)  
        sum_x =0 
        sum_y =0
        i = -1
        tour =0
        r_min = 1
        r_max =3
        x_centre = 0
        
        msg_twist =Twist()


        for r in msg.ranges :
            i=i+1 
            theta = msg.angle_min +msg.angle_increment *i
            thetadeg = math.degrees(theta)
            print('Theta ={}, r ={}, thetadeg = {}'.format(theta,r,thetadeg))
            if 300<thetadeg or thetadeg<60 :
                if r>r_min and r<r_max :
                    x= r * math.cos(theta)
                    y= r * math.sin(theta)
                    sum_x = sum_x +x
                    sum_y = sum_y +y
                    tour +=1
        if tour == 0 :
            msg_twist.linear.x = 0.0
            msg_twist.angular.z = 0.0
        else :

            bari_x = sum_x / tour
            bari_y = sum_y /tour
            print('bari_x ={}, bari_y ={}'.format(bari_x, bari_y))
            print('time ={}'.format(msg.scan_time))

            x_centre = (r_max + r_min)/2

            delta_x = bari_x -x_centre
            delta_y = bari_y 

            self.lin_vel = delta_x *1
            self.rot_vel = delta_y *0.5

            msg_twist.linear.x = self.lin_vel 
            msg_twist.angular.z = self.rot_vel 
        
            print('delta_x ={}, delta_y ={}'.format(delta_x,delta_y))    
        print('lin_vel ={}, rot_vel ={}'.format(self.lin_vel, self.rot_vel))    
        self.publisher.publish(msg_twist)
        
        
        if self.lin_vel != 0 or self.rot_vel != 0: 
            timer = time()
            if i!=0:
                i=0
        else:
            timer = time()
            
            if i == 0:
                oldtime = timer
                i =1
            if timer - oldtime >= 3:
                MinimalSubscriber.goTo()
                
             #This should end the "Follow-me" part of the code after 3sec of inactivity
            #should call a navigation to position fucntion 
            #position final tel antho --> go to coordonate


        

         
        



       
        #twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        #twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        #pub.publish(twist)

        #Follow me
        # If valeur_absolue_bari_x ou bari_y > 0.5m alors 
        # Robot avance de r vers theta_moy

    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# change some parameters to avoid collisions w/ obstacles in sight