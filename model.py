from gettext import dpgettext
import sys
import motor
import math
from constants import *


class Model(object):
    """
    Represents the robot's state 
    """

    def __init__(self):
        # Distance between the wheels
        self.l = L
        # Wheel radius
        self.r = R

        self.x = 0
        self.y = 0
        self.alpha = 0

        self.x_goal = 0
        self.y_goal = 0
        self.alpha_goal = 0

        self.m1 = motor.Motor()
        self.m2 = motor.Motor()

        self.acc = 0
        self.speed_acc = 0
        self.mode = 1

    def __repr__(self):
        s = "current : {} {} {}".format(self.x, self.y, self.theta)
        s = s + "\ngoal    : {} {} {}".format(self.x_goal, self.y_goal, self.theta_goal)
        s = s + "\nmotors    : {} {}".format(self.m1, self.m2)
        s = s + "acc={}, speed_acc={}, mode={}".format(
            self.acc, self.speed_acc, self.mode
        )
        return s

    def ik(self, linear_speed, rotational_speed):
        """Given the linear speed and the rotational speed, 
        returns the speed of the wheels in a differential wheeled robot
        
        Arguments:
            linear_speed {float} -- Linear speed (m/s)
            rotational_speed {float} -- Rotational speed (rad/s)
        
        Returns:
            float -- Speed of motor1 (m/s), speed of motor2 (m/s)
        """
        
        m1_speed = linear_speed - rotational_speed * (L/2)
        m2_speed = linear_speed + rotational_speed * (L/2)
        return m1_speed, m2_speed

    def dk(self):
        """Given the speed of each of the 2 motors (m/s), 
        returns the linear speed (m/s) and rotational speed (rad/s) of a differential wheeled robot
        
        Keyword Arguments:
            m1_speed {float} -- Speed of motor1 (m/s) (default: {None})
            m2_speed {float} -- Speed of motor2 (default: {None})
        
        Returns:
            float -- linear speed (m/s), rotational speed (rad/s)
        """
        
        
        linear_speed = (self.m1.speed + self.m2.speed)/2
        rotation_speed = (self.m1.speed - self.m2.speed)/self.l
        return linear_speed, rotation_speed

    def update(self, dt):
        """Given the current state of the robot (speeds of the wheels) and a time step (dt), 
        calculates the new position of the robot.
        The speed of the wheels are assumed constant during dt.
        
        Arguments:
            dt {float} -- Travel time in seconds
        """
        # Going from wheel speeds to robot speed
        linear_speed, rotation_speed = self.dk()
        dp = linear_speed * dt
        # TODO
        if rotation_speed !=0:
            dx = (dp/self.alpha) * math.sin(self.alpha) #distance (m)
            dy = (dp/self.alpha) * (1-math.cos(self.alpha)) #distance parcourue (m)

        else:
            dx = dp
            dy = 0
        # dp = linear_speed * dt  (distance)
        # alpha = rotational_speed * dt (distance)
        x_m = dx * math.cos(self.alpha) - dy * math.sin(rotation_speed * dt)
        y_m = dx * math.sin(self.alpha) + dy * math.cos(rotation_speed * dt)
        dAlpha = rotation_speed * dt
        # Updating the robot position
        self.x = self.x + x_m  
        self.y = self.y + y_m  
        self.alpha = self.alpha + dAlpha  

