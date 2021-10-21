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
import math
import pam_interface
import o80_pam
import pam_mujoco
from lightargs import BrightArgs,FileExists


ROBOT_SEGMENT_ID = "robot"
MUJOCO_ID = "position_control"
KP = [0.02,0.03,0.01,0.03]
KI = [0.00075,0.0002,0.00125,0.0005]
KD = [0.000,0.001,0.000,0.000]
NDP = [.5,1.,.5,.4]
TIME_STEP = 0.01 # seconds
QD_DESIRED = [0.43,0.174,0.349,0.349] # radian per seconds
PI4 = math.pi/4.
Q_TARGET = [PI4]*4


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


def _go_to(
        pam_config: pam_interface.Configuration,
        interface:o80_pam.o80Pressures,
        q_desired: Sequence[float], ):
    """
    Apply a position controller to reach the
    desired position
    """
    
    _,_,q_current,_ = interface.read()
    
    controller = o80_pam.PositionController(q_current,q_desired,QD_DESIRED,pam_config,
                                            KP,KD,KI,NDP,TIME_STEP)

    while controller.has_next():
        _,_,q,qd = interface.read()
        pressures = controller.next(q,qd)
        interface.set(pressures,duration=o80.Duration_us.milliseconds(int(TIME_STEP*1e3)),wait=True)
    

def _random_posture(interface: o80_pam.o80Pressures,
                    min_pressure: int=12000, max_pressure: int=20000):
    """
    set the robot to a random posture
    """

    
    posture = [ (random.randrange(min_pressure,max_pressure),
                 random.randrange(min_pressure,max_pressure)) for dof in range(4) ]
    interface.set(posture,duration_ms=2000,wait=True)
    
        
def _run(segment_id: str, pam_config_file_path: str):
    """
    runs 3 iteration of 1) robot going to random posture,
    2) robot going to specified Q_TARGET
    """
    interface,frontend = _get_interface_and_frontend(segment_id)
    pam_config = pam_interface.JsonConfiguration(pam_config_file_path)
    

    for _ in range(3):

        print("going to random pressures")
        _random_posture(interface)
        time.sleep(2)
        print("going to target position")
        _go_to(pam_config,interface,Q_TARGET)
        time.sleep(2)
        

def _config() -> BrightArgs :

    config = BrightArgs("pam position control demo")
    config.add_option("segment_id",
                      "",
                      "segment_id of the real robot, empty string for simulated robot",
                      str)
    config.add_option("pam_config_file",
                      pam_interface.DefaultConfiguration.get_path(),
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
    
    
