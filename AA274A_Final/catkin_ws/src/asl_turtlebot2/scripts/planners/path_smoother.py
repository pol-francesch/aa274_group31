import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, k, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        k (int): The degree of the spline fit.
            For this assignment, k should equal 3 (see documentation for
            scipy.interpolate.splrep)
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        t_smoothed (np.array [N]): Associated trajectory times
        traj_smoothed (np.array [N,7]): Smoothed trajectory
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    path = np.array(path)

    # Create nominal time
    nominal_time = np.zeros((path.shape[0],))
    for n in range(1, path.shape[0]):
        nominal_time[n] = np.linalg.norm(path[n,:] - path[n-1,:]) / V_des + nominal_time[n-1]

    # Determine cubic coefficients
    x_spline = scipy.interpolate.splrep(nominal_time, path[:,0], k = k, s = alpha)
    y_spline = scipy.interpolate.splrep(nominal_time, path[:,1], k = k, s = alpha)

    # Create smoothed time
    t_smoothed = np.arange(0, nominal_time[-1], dt)

    # Find splines
    x_d   = scipy.interpolate.splev(t_smoothed, x_spline, der=0)
    xd_d  = scipy.interpolate.splev(t_smoothed, x_spline, der=1)
    xdd_d = scipy.interpolate.splev(t_smoothed, x_spline, der=2)

    y_d   = scipy.interpolate.splev(t_smoothed, y_spline, der=0)
    yd_d  = scipy.interpolate.splev(t_smoothed, y_spline, der=1)
    ydd_d = scipy.interpolate.splev(t_smoothed, y_spline, der=2)

    theta_d = np.arctan2(yd_d, xd_d)
    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return t_smoothed, traj_smoothed
