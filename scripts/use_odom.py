#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time
from math import pi

class UseOdom():

    def __init__(self):
        rospy.init_node('use_odom')

        #velocity publisher
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()

        #Odometry subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.odom_data = Odometry()
        self.z = 0.0
        self.w = 0.0
        self.check_odom_ready = False

        self.ctrl_c = False
        self.rate = rospy.Rate(1)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_pub.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()


    def odom_callback(self,msg):
        self.odom_data = msg
        self.check_odom_ready = True
        self.z = self.odom_data.pose.pose.orientation.z
        self.w = self.odom_data.pose.pose.orientation.w

    def stop_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def facedown(self):
        rospy.sleep(1)
        while (self.w-0.0>0.01):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -0.2
            self.vel_pub.publish(self.cmd)
        self.stop_robot()

    def faceright(self):
        rospy.sleep(2)
        print(self.w)
        while (abs(self.w-0.7075))>0.01:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.2
            self.vel_pub.publish(self.cmd)
        self.stop_robot()

    def move_front(self,vel):
        self.cmd.linear.x = vel
        self.cmd.angular.z = 0
        self.vel_pub.publish(self.cmd)

    def move_straight_t(self,vel,t):
        time.sleep(1)
        self.cmd.linear.x = vel
        self.cmd.angular.z = 0
        self.publish_once_in_cmd_vel()
        time.sleep(t)
        self.stop_robot()



#-->Stage 4 top cylinder to bottom cylinder

if __name__=='__main__':
    uo = UseOdom()
    try:
        uo.move_straight_t(0.1,10)
        uo.facedown()
        uo.move_straight_t(0.03,2)
        uo.move_straight_t(0.1,26)
        uo.faceright()
        uo.move_straight_t(0.03,2)
        uo.move_straight_t(0.1,27)
        uo.facedown()
        uo.move_straight_t(0.03,2)
        uo.move_straight_t(0.1,6)
    except rospy.ROSInterruptException:
        pass
    


#--> Reading odom orientation data:
# if __name__=='__main__':
#     uo = UseOdom()
#     try:    
#         rospy.sleep(1)

#         while 1:
#             print(uo.z,uo.w,"\n")
#             rospy.sleep(1)
#     except rospy.ROSInterruptException:
#         pass

