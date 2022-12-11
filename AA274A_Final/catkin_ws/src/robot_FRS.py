#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point32, Polygon
from nav_msgs.msg import Path
from FRS.msg import FRS

class robot_FRS:
    """
    This node handles calculating the robot's forward reachable set (FRS).
    It takes in planned trajectory and current location.
    It publishes the FRS, represented by a 2D zonotope (aka a square) at each point in the planned trajectory.
    """

    def __init__(self):
        rospy.init_node("robot_FRS",anonymous=True)
        self.v_max = .2
        self.error = .2
        self.traj = None
        self.ers = np.zeros((4,2))
        self.robotfrs = FRS()
        self.FRS_pub = rospy.Publisher("/frs",FRS,queue_size=10)
        rospy.Subscriber("/cmd_smoothed_path",type,self.traj_callback)
        #rospy.Subscriber("/topic",type,pos_callback)
    
    def traj_callback(self, smooth_traj):
        self.traj = np.zeros((len(smooth_traj.poses),2))
        for i in range(len(smooth_traj.poses)):
            self.traj[i,0] = smooth_traj.poses.pose.position.x
            self.traj[i,1] = smooth_traj.poses.pose.position.y

    # def pos_callback(self, pos):
        # The goal with this would be to estimate the robot's error. For now, let's just set it constant.
        # pass

    def calc_ERS(self):
        # center is 0,0
        # betas are just error
        # use 2 generators to get a square
        gen = np.array([[1,0],[0,1]])
        beta_gen = np.vstack(gen*self.error,gen*-self.error)
        for i in range(4):
            for j in range(np.shape(beta_gen[i:])):
                self.ers[i,:] = beta_gen[i]+beta_gen[i+j]
    
    def calc_FRS(self):
        self.calc_ERS()
        centers = np.array(self.traj)
        for c in centers:
            frs_i = c+self.ers
            data = Polygon()
            for pt in frs_i:
                data.points.append(Point32(pt[0],pt[1],0))
            self.robotfrs.polygons.append(data)
        self.FRS_pub.publish(self.robotfrs)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.calc_FRS()
            rate.sleep()

if __name__ == '__main__':
    my_frs = robot_FRS()
    my_frs.run()

