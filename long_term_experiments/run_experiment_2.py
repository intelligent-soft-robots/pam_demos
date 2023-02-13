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

import matplotlib.pyplot as plt


from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_thermal_imaging import BrickletThermalImaging


real_robot = True
pressures_min = [13000, 16500, 14500, 15000, 13000, 16500, 14500, 15000]
pressures_max = [25000, 21500, 21000, 25000, 25000, 25000, 21000, 25000]
pressures_mid = [int((pressures_min[i]+pressures_max[i])/2) for i in range(8)]
pressures_range = [int(pressures_max[i]-pressures_min[i]) for i in range(8)]

#was: pressures_mid = [20000, 19500, 17000, 19000, 22000, 21500, 17000, 19000]  #was: pressures_mid = [20000, 18250, 17000, 19000, 20500, 20500, 17000, 19000]
#was: pressures_half_range = [5000, 3250, 4000, 6000, 4500, 4500, 4000, 6000]


def init_robot():
    if not real_robot:
        mujoco_id = "position_control"
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


def move_to_zero_position():
    pressures = [[24555, 23606, 20856, 24056, 20228, 20192, 25019, 22000]]
    time.sleep(0.05)
    open_loop_motion(500, 1, pressures)
    
    pressures_current = frontend.latest().get_observed_pressures()
    print(pressures_current)
    #pressures = [[20500, 20000, 20000, 25000, 20000, 20500, 20000, 20000]]
    #open_loop_motion(500, 1, pressures)



# def move_to_position():




#     def reset_check(self, observation):
        
#         print(observation.joint_positions)

        
#         reset_counter = 0
#         while not self.check_init_joint_postions(observation.joint_positions):
#             self._do_natural_reset()
#             observation = self._create_observation()

#             # try reset 5 times
#             reset_counter += 1
#             if reset_counter>=5:
#                 input()

#             # check if 2. dof gets stuck on wrong side after reset
#             if observation.joint_positions[2]<0.1:
#                 self.move_second_dof_to_positive_angle_joint_pos()


#         # log reset position and pressures to file
#         with open("init_joint_pos.txt", "a") as f:
#             f.write(str(observation.joint_positions)+',\n')
#         with open("init_pressures.txt", "a") as f:
#             f.write(str(self._hysr_config.starting_pressures)+',\n')


#     def check_init_joint_postions(self, pos):
#         # calculate error
#         err = [pos[i]-self._hysr_config.reference_posture[i] for i in range(4)]
#         err_capped = np.array([max([min([e, 0.1]), -0.1]) for e in err])

#         # update starting pressures
#         self._hysr_config.starting_pressures = list(list(np.int_(ps) for ps in np.array(self._hysr_config.starting_pressures)*np.swapaxes([1+err_capped*abs(err_capped), [1,1,1,1]], 0, 1)))

#         #check if starting position within boundaries
#         max_err = [0.1, 0.35, 0.25, 0.5]
#         if any([abs(err[i])>max_err[i] for i in range(4)]):
#             print("_______reset____________")
#             print("init error to large...")
#             print("init pos:", pos, "reference:", self._hysr_config.reference_posture)
#             return False
#         return True

#     def move_second_dof_to_positive_angle_joint_pos(self):
#         print("___ Move 2. dof to other side ______")
#         target_pressures = list(list(np.int_(ps) for ps in np.array(self._hysr_config.starting_pressures)*np.swapaxes([[1, 0.6, 1, 1], [1,1.3,1,1]], 0, 1)))
#         self._move_to_pressure(target_pressures)



def move_to_list_of_pressures():
    position_list = [[-1, 1.0  * 0.2, -1, 1], [1, 0.8  * 0.2, 1, -1], [1, 0  * 0.2, -1, 1], [-1, -0.4  * 0.2, 1, -1],
                     [1, 0.8  * 0.2, 1, 1], [-1, 0.8  * 0.2, -1, -1], [-1, -0.4  * 0.2, 1, 1], [1, -0.8  * 0.2, -1, -1],
                    [-1, 0.8  * 0.2, 0, 1], [1, 0.8  * 0.2, 0, -1], [1, -0.2  * 0.2, 0, 0.5], [-1, -0.6  * 0.2, 0, -0.8]]
    for position in position_list:
        print(position)
        pressures = np.multiply(position + [-p for p in position], pressures_half_range) * 1.0 + np.array(pressures_mid)
        pressures = pressures.astype(int)
        pressures = [x.tolist() for x in pressures]
        print(pressures)
        open_loop_motion(2000, 1, [pressures])

def move_to_random_pressures():
    rel_diff = [random.random() for i in range(4)]
    rel_start = [random.random() * (1-d) for d in rel_diff]
    higher = [random.random()>0.5 for i in range(4)]
    rel_values = [(rel_start[i] + rel_diff[i]*higher[i]) if i<4 else (rel_start[i-4] + rel_diff[i-4]*(1-higher[i-4])) for i in range(8)]
    target_p = [[int(pressures_min[i] + pressures_range[i]*rel_values[i]) for i in range(8)]]
    print(target_p)
    open_loop_motion(3000, 1, target_p)
    time.sleep(1)

def move_to_min_pressures():
    print(pressures_min)
    pressures = [pressures_min]
    obs = frontend.latest()
    print("-----", obs.get_iteration())
    open_loop_motion(2000, 1, pressures)

def move_to_mid_pressures():
    pressures = [pressures_mid]
    time.sleep(0.1)
    obs = frontend.latest()
    print("-----", obs.get_iteration())
    open_loop_motion(3000, 1, pressures)
    obs = frontend.latest()
    print("-----", obs.get_iteration())


def start_logging(logger, idx):
    if logger and logger!=None:
        print("stop logger")
        logger.stop() 
        print(".")
    segment_id = "real_robot"
    file_path = "/tmp/long_term_" + str(idx)
    frequency = 500
    logger = o80_pam.Logger(segment_id,file_path,frequency=frequency)
    print("start logger")
    logger.start() # spawn a process which starts dumping observations into the file
    print(".")
    return logger

def random_freq_targets():
    t = np.arange(1000)

    n = np.zeros((1000,), dtype=complex)

    n_freq = 200
    n[0:n_freq] = [1] * n_freq

    s = np.fft.ifft(n)

    plt.plot(t, s.real, label='real')

    plt.plot(t, s.imag, '--', label='imaginary')

    plt.legend()

    plt.show()




def stop_logging(logger):
    print("stop logger")
    logger.stop()
    print(".")
    return logger

if __name__ == "__main__":
    # logger = None
    # frontend, robot_config = init_robot()
    # move_to_mid_pressures()
    # move_to_min_pressures()
    # for k in range(10):
    #     if k%1000==0:
    #         logger = start_logging(logger,k)

    random_freq_targets()
    


# if __name__ == "__main__":
#     # parser = ArgumentParser()
#     # parser.add_argument("data", type=str)
#     # args = parser.parse_args()
#     # print(args.data)

#     logger = None
#     frontend, robot_config = init_robot()

#     # pressures = [[16000, 12000, 14000, 14000, 14000, 14000, 14000, 14000]]
#     # time.sleep(0.1)
#     # print("-----")
#     # open_loop_motion(2000, 1, pressures)
#     # print("-----")

#     # for i in range(10):
#     #     move_to_random_pressures()

#     # logger = start_logging(logger, 0)
#     # print(pressures_mid)
#     # pressures = [pressures_mid]
#     # time.sleep(0.1)
#     # obs = frontend.latest()
#     # print("-----", obs.get_iteration())
#     # open_loop_motion(3000, 1, pressures)
#     # obs = frontend.latest()
#     # print("-----", obs.get_iteration())

#     # logger = start_logging(logger, 1)
#     print(pressures_min)
#     pressures = [pressures_min]
#     obs = frontend.latest()
#     print("-----", obs.get_iteration())
#     open_loop_motion(3000, 1, pressures)
#     # obs = frontend.latest()
#     # print("-----", obs.get_iteration())

#     # logger = stop_logging(logger)


#     # n = 1
#     # for i in range(n):
#         #print("------", i+1, "/ " + str(n) + " --------")
#         #fast_hitting_motions(3, log=False)
        
#         #move_to_list_of_pressures()
#         # input()
#         #fast_hitting_motions(3)
#         #move_to_list_of_positions()
#         #move_to_list_of_pressures()
#         #repeatability_motion3()
#         #move_to_zero_position()
#         # time.sleep(0.1)
#         # capture_image()
        
        
#     print("done")