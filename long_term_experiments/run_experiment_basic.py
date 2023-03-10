from ast import Return
import o80
import pam_interface
import o80_pam
import pam_mujoco
from argparse import ArgumentParser
from datetime import datetime
import numpy as np
import math
import time
import random
import pickle

import matplotlib.pyplot as plt


real_robot = False
pressures_min = [13000, 16500, 14500, 15000, 13000, 16500, 14500, 15000]
pressures_max = [25000, 21500, 21000, 25000, 25000, 25000, 21000, 25000]
pressures_mid = [int((pressures_min[i]+pressures_max[i])/2) for i in range(8)]
pressures_range = [int(pressures_max[i]-pressures_min[i]) for i in range(8)]
pressures_half_range = [int(pressures_max[i]-pressures_min[i])/2 for i in range(8)]

def init_robot():
    if not real_robot:
        mujoco_id = "p_control"
        robot_segment_id = "robot"
        burst_mode = False
        accelerated_time = False

        def _get_handle():

            control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
            graphics = True
            pamy = pam_mujoco.RobotType.PAMY1
            robot = pam_mujoco.MujocoRobot(pamy, robot_segment_id, control=control)
            handle = pam_mujoco.MujocoHandle(
                mujoco_id, robot1=robot, graphics=graphics,
                accelerated_time=accelerated_time,
                burst_mode=burst_mode
            )
            return handle, robot.json_control_path

        # getting the handle and the json file used to configure
        # the robot (to get min and max pressures)
        handle, robot_config_path = _get_handle()

        # frontend to robot
        frontend = handle.frontends[robot_segment_id]
        robot_config = pam_interface.JsonConfiguration(robot_config_path)
    else:
        frontend = o80_pam.FrontEnd("real_robot")
        robot_config = pam_interface.Pamy2DefaultConfiguration(False)
    return frontend, robot_config


def open_loop_motion(duration, n, pressures):
    # run repetitive hitting motions
    for i in range(n):
        # creating a command locally. The command is *not* sent to the robot yet.
        for pressure in pressures:
            pressureAGO = pressure[0:4]
            pressureANT = pressure[4:8]
            frontend.add_command(pressureAGO, pressureANT,
                                o80.Duration_us.milliseconds(duration),
                                o80.Mode.QUEUE)
        # sending the command to the robot, and waiting for its completion.
        frontend.pulse_and_wait()

def move_to_list_of_pressures(slow=True):
    position_list = [[-1, 1.0  * 1.0, -1, 1], [1, 0.8  * 1.0, 1, -1], [1, 0  * 1.0, -1, 1], [-1, -0.7  * 1.0, 1, -1],
                     [1, 0.8  * 1.0, 1, 1], [-1, 0.8  * 1.0, -1, -1], [-1, -0.7  * 1.0, 1, 1], [1, -0.8  * 1.0, -1, -1],
                    [-1, 0.8  * 1.0, 0, 1], [1, 0.8  * 1.0, 0, -1], [1, -0.2  * 1.0, 0, 0.5], [-1 * 0.9, -0.6  * 1.0, 0, -0.8], [0, -0.6  * 1.0, 0, -0.8]]
    if slow:
        duration = 2000
        multiplier = 1.0
    else:
        duration = 700
        multiplier = 0.7    # make motion less agressive

    for position in position_list:
        pressures = np.multiply(position + [-p for p in position], pressures_half_range) * multiplier + np.array(pressures_mid)
        pressures = pressures.astype(int)
        pressures = [x.tolist() for x in pressures]
        open_loop_motion(duration, 1, [pressures])

def move_to_min_pressures():
    pressures = [pressures_min]
    open_loop_motion(2000, 1, pressures)

def move_to_mid_pressures(factor = 1.0, duration=3000):
    pressures = [int(pressures_mid[i] * (1-factor) * (i<4) + pressures_mid[i]) for i in range(8)]
    pressures = [pressures]
    open_loop_motion(3000, 1, pressures)

def start_logging(logger, idx=0):
    if logger and logger!=None:
        print("stop logger")
        logger.stop() 
    segment_id = "real_robot"
    file_path = "/tmp/data_" + str(idx)
    frequency = 500
    logger = o80_pam.Logger(segment_id,file_path,frequency=frequency)
    print("start logger")
    logger.start() # spawn a process which starts dumping observations into the file
    return logger

def stop_logging(logger):
    print("stop logger")
    logger.stop()
    return logger

if __name__ == "__main__":

    # init variables and lists
    logger = None
    frontend, robot_config = init_robot()
    should_exit = False
    idx_list = []

    # start logging
    logger = start_logging(logger)

    # execute target pressure trajectory
    it_start = frontend.latest().get_iteration()
    move_to_list_of_pressures(slow=True)
    it_end = frontend.latest().get_iteration()
    idx_list.append((it_start, it_end))
        
    # save indices
    file_path = "/tmp/data_" + str("0")
    with open (file_path+"_idx", "wb") as f:
        pickle.dump(idx_list, f)

    # stop logging and apply min pressures
    logger = stop_logging(logger)
    move_to_min_pressures()
    

print("done")