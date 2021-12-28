#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
from math import pi

def move_square():
    vel=0.1
    print('square')
    rospy.init_node('twist_publisher')
    twist1 = Twist()
    twist2 = Twist()
    start = time.time()
    twist1.angular.z = 0
    twist1.linear.x = vel
    twist2.angular.z = vel
    twist2.linear.x = 0.0
    pub = rospy.Publisher('/rviz/moveit/', Twist, queue_size=1)
    r = rospy.Rate(1)
    t0 = time.time()
    for i in range(1):
        t0 = time.time()
        while vel * (time.time() - t0) < 0.6:
            pub.publish(twist1)
            r.sleep()
        t0 = time.time()
        while vel* (time.time() - t0) < pi / 2:
            pub.publish(twist2)
            r.sleep()

    pub.publish(Twist())


if __name__ == "__main__":
    move_square()
    print('1')