import typing as T

import numpy as np
from utils import wrapToPi
import rospy
from std_msgs.msg import Float32

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1: float, k2: float, k3: float,
                 V_max: float = 0.5, om_max: float = 1) -> None:
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

        self.V_max = V_max
        self.om_max = om_max

        # Define publishers
        self.rho_pub = rospy.Publisher("/controller/rho",Float32,queue_size=10)
        self.alpha_pub = rospy.Publisher("/controller/alpha",Float32,queue_size=10)
        self.del_pub = rospy.Publisher("/controller/delta",Float32,queue_size=10)

    def load_goal(self, x_g: float, y_g: float, th_g: float) -> None:
        """ Loads in a new goal position """
        self.x_g = x_g
        self.y_g = y_g
        self.th_g = th_g

    def compute_control(self, x: float, y: float, th: float, t: float) -> T.Tuple[float, float]:
        """
        Inputs:
            x,y,th: Current state
            t: Current time (you shouldn't need to use this)
        Outputs:
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
        self.k1 = 1.0; self.k2 = 0.4; self.k3 = 0.4
        rho = np.sqrt((self.x_g - x)**2 + (self.y_g - y)**2)
        beta = np.arctan2(self.y_g - y, self.x_g - x)            # beta = alpha + theta = delta + theta_goal
        alpha = wrapToPi(beta - th)
        delta = wrapToPi(beta - self.th_g)

        V = self.k1 * rho * np.cos(alpha)
        om = self.k2*alpha + self.k1 * np.sin(alpha) * np.cos(alpha) / alpha * (alpha + self.k3*delta)

        # Publish control inputs
        self.rho_pub.publish(Float32(rho))
        self.alpha_pub.publish(Float32(alpha))
        self.del_pub.publish(Float32(delta))
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om
