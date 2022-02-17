"""
Demo for o80_pam.PositionController
The (pressure controlled) robot is requested to return to a specified position
after random motion.
Required before running:
  pam_mujoco position_control
"""

from typing import Sequence,Tuple
import random
import time
from datetime import datetime
import math
import matplotlib.pyplot as plt
import o80
import pam_interface
import o80_pam
import pam_mujoco
from lightargs import BrightArgs,FileExists


ROBOT_SEGMENT_ID = "robot"
MUJOCO_ID = "position_control"

## gains for simulated robot ##
#KP = [0.007,0.4,3.0,0.8]
#KI=[0.002,0.0001,0.0,0.005]
#KD = [0.007,0.01,0.00,0.004]
#NDP = [0.1,-0.3,0.05,0.2]

## gains for real robot (pamy1) ##
#KP = [ 0.8 , -3.0  , 1.2,  -1.0 ]
#KI = [ 0.015 , -0.25 , 0.02 ,-0.05 ]
#KD = [ 0.04 , -0.09 , 0.09 , -0.09 ]
#NDP = [ -0.3 , -0.5 , -0.34 , -0.48 ]

## gains for real robot (pamy2) ##
KP = [ 0.4 , 0.0 , 0.0,  0.0 ]
KI = [ 0.0 , 0.0 , 0.0 , 0.0 ]
KD = [ 0.0 , 0.0 , 0.0 , 0.0 ]
NDP = [ 0.5 , 0.5 , 0.5 , 0.5] 


# gains, advice:
# starting plotting the pressure values,
# i.e. running o80_plotting
# is helpful to tune NDP


TIME_STEP = 0.01 # seconds
QD_DESIRED = [0.7,0.7,0.7,0.7] # radian per seconds
PI4 = math.pi/4.
Q_TARGET = [-PI4,-PI4,0,0] # target position in radian

    
def _get_handle() -> pam_mujoco.MujocoHandle :
    """
    Configure mujoco to be realtime, to have graphics,
    and to setup a pressure controlled robot
    """
    burst_mode=False
    accelerated_time=False
    control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
    graphics = True
    robot = pam_mujoco.MujocoRobot(ROBOT_SEGMENT_ID, control=control)
    handle = pam_mujoco.MujocoHandle(
        MUJOCO_ID, robot1=robot, graphics=graphics, accelerated_time=accelerated_time,
        burst_mode=burst_mode
    )
    return handle


def _get_interface_and_frontend(segment_id:str) -> Tuple[o80_pam.o80Pressures,o80_pam.FrontEnd]:
    """
    Returns interface and frontend to the robot.
    If segment_id is empty: mujoco simulated robot
    For real robot: segment_id must be the segment_id for
    the robot backend.
    """
    if segment_id:
        # segment id: i.e. real robot
        frontend = o80_pam.FrontEnd(segment_id)
        interface = o80_pam.o80Pressures(segment_id,frontend=frontend)
        return (interface,frontend)

    handle = _get_handle()
    frontend = handle.frontends["robot"]
    interface = handle.interfaces["robot"]
    return (interface,frontend)


def _plot_trajectories(
        pam_config: pam_interface.Configuration,
        interface:o80_pam.o80Pressures,
        q_desired: Sequence[float], ):
    """
    
    """
    fdir = "/home/vberenz/Desktop/pid_position_tuning/"
    fig = plt.figure()
    now = datetime.now()
    
    interface,frontend = _get_interface_and_frontend("real_robot")
    start_iteration = frontend.latest().get_iteration()

    _,_,q_current,_ = interface.read()
    controller = o80_pam.PositionController(q_current,q_desired,QD_DESIRED,pam_config,
                                            KP,KD,KI,NDP,TIME_STEP)
    
    des_q_traj, des_dq_traj = controller.get_trajectories()

    q_traj,dq_traj = [[] for _ in range(4)],[[] for _ in range(4)]

    while controller.has_next():
        _,_,q,qd = interface.read()
        pressures = controller.next(q,qd)
        interface.set(pressures,duration_ms=int(TIME_STEP*1e3),wait=True)
        _,__,angles,___ = interface.read()
        for dof in range(4):
            q_traj[dof].append(angles[dof])
    
    def _plot_des_vs_applied(axes,dof,des_q_traj,q_traj):
        axes[0][dof].plot(des_q_traj[dof])
        axes[1][dof].plot(q_traj[dof])
        
    fig = plt.figure()
    axes = fig.subplots(2,4)
    for dof in range(4):
        _plot_des_vs_applied(axes,dof,des_q_traj,q_traj)

    plt.show()


def _run(segment_id: str, pam_config_file_path: str):
    """
    runs 3 iterations of 1) robot going to random posture,
    2) robot going to specified Q_TARGET
    """
    interface,frontend = _get_interface_and_frontend(segment_id)
    pam_config = pam_interface.JsonConfiguration(pam_config_file_path)
    
    _plot_trajectories(
        pam_config,
        interface,
        Q_TARGET
    )

def _config() -> BrightArgs :
    """
    Configuration dialog
    """
    config = BrightArgs("pam position control demo")
    config.add_option("segment_id",
                      "real_robot",
                      "segment_id of the real robot, empty string for simulated robot",
                      str)
    config.add_option("pam_config_file",
                      pam_interface.Pamy2DefaultConfiguration.get_path(),
                      "file specifying the max and min pressures for all muscles",
                      str,
                      [FileExists])
    change_all = False
    config.dialog(change_all)
    print()
    return config


if __name__ == "__main__":

    config = _config()
    _run(config.segment_id,config.pam_config_file)
    
    
