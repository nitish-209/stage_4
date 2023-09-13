#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from math import pi


class FollowWall():

    def __init__(self):
        rospy.init_node('follow_wall')
        
        #velocity publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        
        self.sider = 0.0
        self.sidel = 0.0
        self.front = 0.0
        self.sidem = 0.0
        self.back = 0.0
        #laser subscriber
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_msg = []

        self.ctrl_c = False
        self.rate = rospy.Rate(1)


    def publish_in_cmdvel(self):
        self.vel_pub.publish(self.cmd)


    def laser_callback(self,msg):
        self.laser_msg = msg.ranges
        front = (sum(msg.ranges[355:])+sum(msg.ranges[:6]))/10
        self.front = round(front, 2)
        sider = sum(msg.ranges[269:272])/3
        self.sider = round(sider,6)
        sidel = sum(msg.ranges[89:92])/3
        self.sidel = round(sidel,6)
        self.sidem = round(min(msg.ranges),6)
        back = sum(msg.ranges[179:182])/3
        self.back = round(back,6)

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.vel_pub.publish(self.cmd)


    def move_straight(self,vel,t):
        self.cmd.linear.x = vel
        self.cmd.angular.z = 0
        i = 0
        self.vel_pub.publish(self.cmd)
        rospy.sleep(t)
        self.stop_robot()

    def move_front(self,vel):
        self.cmd.linear.x = vel
        self.cmd.angular.z = 0
        self.vel_pub.publish(self.cmd)
    
    def turn90(self,cw):
        self.cmd.linear.x = 0
        if cw == 'cw':
            self.cmd.angular.z = -pi/2/5
        else:
            self.cmd.angular.z = pi/2/5
        self.vel_pub.publish(self.cmd)
        rospy.sleep(5)
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.vel_pub.publish(self.cmd)

    def turn(self,cw):
        self.cmd.linear.x = 0
        if cw == 'cw':
            self.cmd.angular.z = -0.1
        else:
            self.cmd.angular.z = 0.1
        self.vel_pub.publish(self.cmd)
        

if __name__=='__main__':
    fw = FollowWall()

    try:
        rospy.sleep(1)
        while fw.front>0.5 or fw.front==0.0:
            fw.move_front(0.1)
        fw.stop_robot()
        while abs(fw.sidel-fw.sidem)>0.01:
            print(fw.sidel-fw.sidem)
            fw.turn('cw')
        fw.stop_robot()
        fw.move_straight(0.5,5)
        while fw.back!= fw.sidem:
            print(fw.back,fw.sidem)
            fw.turn('acw')
        fw.stop_robot()
        while fw.front>0.4:
            fw.move_front(0.2)
        fw.turn90('cw')
        fw.move_straight(0.2,2)
    except rospy.ROSInterruptException:
        pass
'''        while 1:
            print(fw.sidel)
            print(fw.sidem)
            rospy.sleep(1)
'''


#pid wallfolloer try 1:
'''        while fw.front>0.3:
            if fw.sidem>0.3:
                print('1',fw.sidem)
                fw.cmd.linear.x = 0.2
                fw.cmd.angular.z = (fw.sidem-0.3)
                fw.publish_in_cmdvel()
            elif fw.sidem<0.3:
                print('2',fw.sidem)
                fw.cmd.linear.x = 0.2
                fw.cmd.angular.z = (fw.sidem-0.3)
                fw.publish_in_cmdvel()
            else:
                print('3',fw.sidem)
                fw.cmd.linear.x = 0.1
                fw.cmd.angular.z = 0.0
                fw.publish_in_cmdvel()
        fw.stop_robot()
'''


