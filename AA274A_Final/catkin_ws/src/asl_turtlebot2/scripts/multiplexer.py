#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Multiplexer:
    def __init__(self):
        rospy.init_node("multiplexer", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/cmd_vel_nav", Twist, self.nav_callback)
        rospy.Subscriber("/cmd_vel_teleop", Twist, self.teleop_callback)

        self.cmd_vel_nav = Twist()
        self.cmd_vel_teleop = Twist()
    
    def nav_callback(self, data):
        self.cmd_vel_nav.linear = data.linear
        self.cmd_vel_nav.angular = data.angular
    
    def teleop_callback(self, data):
        self.cmd_vel_teleop.linear = data.linear
        self.cmd_vel_teleop.angular = data.angular
    
    def main(self):
        while not rospy.is_shutdown():
            use_teleop = rospy.get_param("~teleop", False)

            if use_teleop:
                self.cmd_vel_pub.publish(self.cmd_vel_teleop)
            else:
                self.cmd_vel_pub.publish(self.cmd_vel_nav)

if __name__=="__main__":
    try:
        multiplexer = Multiplexer()
        multiplexer.main()
    except rospy.ROSInterruptException:
        pass
