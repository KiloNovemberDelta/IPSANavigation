#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class Robot:
    def __init__(self):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0 #Sonar distance

        '''Listener and publisher'''

        rospy.Subscriber("/sensor/sonar_front", Range, self.callbacksonar)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        self.pub_velocity() #Run the publisher once

    def callbacksonar(self,data):
        self.sonar = data.range

    def get_sonar(self):
        return self.sonar

    def set_speed_angle(self,speed,angle):
        self.speed = speed
        self.angle = angle
        self.pub_velocity()

    def pub_velocity(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = self.angle

        self.cmd_vel_pub.publish(cmd_vel)

def run_demo():
    '''Main loop'''
    robot = Robot()
    while True:
        #Write here your strategy..

        print("SONAR VALUE : ")
        print(robot.get_sonar())

        velocity = 0
        angle = 0
        sonar = float(robot.get_sonar())


        #Finishing by publishing the desired speed. DO NOT TOUCH.
        robot.set_speed_angle(velocity,angle)
        rospy.sleep(0.5)

if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous = True)

    run_demo()
