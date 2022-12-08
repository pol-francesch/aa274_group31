import os
import typing as T

import numpy as np
from scipy.integrate import odeint  # type: ignore
from six.moves import cPickle as pickle #for performance


def car_dyn(x: np.ndarray, t: float, ctrl: np.ndarray, noise: np.ndarray) -> T.List[float]:
    u_0 = ctrl[0] + noise[0]
    u_1 = ctrl[1] + noise[1]
    dxdt = [u_0 * np.cos(x[2]),
            u_0 * np.sin(x[2]),
            u_1]
    return dxdt

def simulate_car_dyn(
    x_0: float,
    y_0: float,
    th_0: float,
    times: T.List[float],
    controller: T.Optional[T.Any] = None,
    actions: T.Optional[np.ndarray] = None,
    noise_scale: float = 0.
) -> T.Tuple[np.ndarray, np.ndarray]:
    """
    inputs: x_0,y_0,th_0 (floats) initial state
            times (list len N) sequence of times at which to apply control
            controller: controller object to use to compute feedback control
            actions: (np.array shape: N-1, 2) list of actions to apply
            noise_scale: (float) standard deviation of control noise

            if controller is provided, simulates feedback control by calling
                controller.compute_control(x,y,th,t) at each time step
            otherwise, if the array actions is specified, they are applied open loop

            (one of controller or actions must be specified)

    outputs: states (np.array shape (N, 3)) sequence of [x,y,th] state vectors
             ctrl (np.array shape (N-1, 2)) sequence of [V, om] control vectors
    """

    feedback = False
    if controller:
        feedback = True
    elif actions is None:
        print("Either provide a controller or a sequence of open loop actions")
        raise Exception

    x = np.array([x_0, y_0, th_0]) # vector of x,y,th
    N = len(times)
    states = np.zeros([N,3])
    noise = noise_scale*np.random.randn(N,2) # control noise
    ctrl = np.zeros([N-1, 2]) # vector of V, om
    for i,t in enumerate(times[:-1]):
        # log current state
        states[i,:] = x

        # compute control
        if feedback:
            V, om = controller.compute_control(x[0], x[1], x[2], t)
        elif actions is not None:
            V = actions[i,0]
            om = actions[i,1]

        ctrl[i,0] = V
        ctrl[i,1] = om

        # apply control and simulate forward
        d_state = odeint(car_dyn, x, [t, times[i+1]], args=(ctrl[i,:], noise[i,:]))
        x = d_state[1,:]

    # log final state
    states[-1,:] = x

    return states, ctrl


def wrapToPi(a: T.Union[T.List[float], np.ndarray]) -> T.Union[T.List[float], np.ndarray]:
    if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
        return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
    return (a + np.pi) % (2*np.pi) - np.pi

def get_folder_name(filename: str) -> str:
    return '/'.join(filename.split('/')[:-1])

def maybe_makedirs(path_to_create: str) -> None:
    """This function will create a directory, unless it exists already,
    at which point the function will return.
    The exception handling is necessary as it prevents a race condition
    from occurring.
    Inputs:
        path_to_create - A string path to a directory you'd like created.
    """
    try:
        os.makedirs(path_to_create)
    except OSError:
        if not os.path.isdir(path_to_create):
            raise

def save_dict(di_: T.Dict[str, T.Any], filename_: str) -> None:
    maybe_makedirs(get_folder_name(filename_))
    with open(filename_, 'wb') as f:
        pickle.dump(di_, f)

def load_dict(filename_: str) -> T.Dict[str, T.Any]:
    with open(filename_, 'rb') as f:
        ret_di = pickle.load(f)
    return ret_di
