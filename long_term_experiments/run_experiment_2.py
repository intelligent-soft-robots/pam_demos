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



real_robot = True
pressures_min = [13000, 16500, 14500, 15000, 13000, 16500, 14500, 15000]
pressures_max = [25000, 21500, 21000, 25000, 25000, 25000, 21000, 25000]
pressures_mid = [int((pressures_min[i]+pressures_max[i])/2) for i in range(8)]
pressures_range = [int(pressures_max[i]-pressures_min[i]) for i in range(8)]
pressures_half_range = [int(pressures_max[i]-pressures_min[i])/2 for i in range(8)]

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



def move_to_list_of_pressures(slow=True):
    if slow:
        position_list = [[-1, 1.0  * 1.0, -1, 1], [1, 0.8  * 1.0, 1, -1], [1, 0  * 1.0, -1, 1], [-1, -0.7  * 1.0, 1, -1],
                     [1, 0.8  * 1.0, 1, 1], [-1, 0.8  * 1.0, -1, -1], [-1, -0.7  * 1.0, 1, 1], [1, -0.8  * 1.0, -1, -1],
                    [-1, 0.8  * 1.0, 0, 1], [1, 0.8  * 1.0, 0, -1], [1, -0.2  * 1.0, 0, 0.5], [-1 * 0.9, -0.6  * 1.0, 0, -0.8], [0, -0.6  * 1.0, 0, -0.8]]
                    # [1 * 0.8, -0.6  * 1.0, 0, -0.8], [-1 * 0.8, -0.6  * 1.0, 0, -0.8], [0, -0.6  * 1.0, 0, -0.8]]
        duration = 2000
        multiplier = 1.0
    else:
        position_list = [[1 * 0.8, -0.6  * 1.0, 0, -0.8], [-1 * 0.8, -0.6  * 1.0, 0, -0.8], [0, -0.6  * 1.0, 0, -0.8]]
        position_list = [[-1, 1.0  * 1.0, -1, 1], [1, 0.8  * 1.0, 1, -1], [1, 0  * 1.0, -1, 1], [-1, -0.7  * 1.0, 1, -1],
                     [1, 0.8  * 1.0, 1, 1], [-1, 0.8  * 1.0, -1, -1], [-1, -0.7  * 1.0, 1, 1], [1, -0.8  * 1.0, -1, -1],
                    [-1, 0.8  * 1.0, 0, 1], [1, 0.8  * 1.0, 0, -1], [1, -0.2  * 1.0, 0, 0.5], [-1 * 0.9, -0.6  * 1.0, 0, -0.8], [0, -0.6  * 1.0, 0, -0.8]]
                    # [1 * 0.8, -0.6  * 1.0, 0, -0.8], [-1 * 0.8, -0.6  * 1.0, 0, -0.8], [0, -0.6  * 1.0, 0, -0.8]]
        duration = 700
        multiplier = 0.7

    for position in position_list:
        print(position)
        pressures = np.multiply(position + [-p for p in position], pressures_half_range) * multiplier + np.array(pressures_mid)
        pressures = pressures.astype(int)
        pressures = [x.tolist() for x in pressures]
        open_loop_motion(duration, 1, [pressures])
        

def move_to_random_pressures(rel_diff_between_min_and_max = None, rel_min_value = None, is_agonist_higher = None):
    if rel_diff_between_min_and_max is None:
        rel_diff_between_min_and_max = [random.random() for i in range(4)]
    if rel_min_value is None:
        rel_min_value = [random.random() * (1-d) for d in rel_diff_between_min_and_max]
    if is_agonist_higher is None:
        is_agonist_higher = [random.random()>0.5 for i in range(4)]
    move_to_pressures_by_relative_values(rel_diff_between_min_and_max, rel_min_value, is_agonist_higher)
    return (rel_diff_between_min_and_max, rel_min_value, is_agonist_higher)

def move_to_pressures_by_relative_values(rel_diff_between_min_and_max, rel_min_value, is_agonist_higher):
    # print(rel_diff_between_min_and_max, rel_min_value, is_agonist_higher)
    rel_values = [(rel_min_value[i] + rel_diff_between_min_and_max[i]*is_agonist_higher[i]) if i<4 \
        else (rel_min_value[i-4] + rel_diff_between_min_and_max[i-4]*(1-is_agonist_higher[i-4])) for i in range(8)]
    target_p = [[int(pressures_min[i] + pressures_range[i]*rel_values[i]) for i in range(8)]]
    
    open_loop_motion(3000, 1, target_p)
    time.sleep(1)
    return target_p

def calc_pressures_by_relative_values2(rel_diff_between_min_and_max, rel_min_value):
    # print(rel_diff_between_min_and_max, rel_min_value)
    rel_values = [(rel_min_value[i] + rel_diff_between_min_and_max[i]) if i<4 \
        else (rel_min_value[i] - rel_diff_between_min_and_max[i-4]) for i in range(8)]
    target_p = [int(pressures_min[i] + pressures_range[i]*rel_values[i]) for i in range(8)]
    return target_p

def move_to_min_pressures():
    pressures = [pressures_min]
    print("move_to_min_pressures", pressures)
    obs = frontend.latest()
    print("-----", obs.get_iteration())
    open_loop_motion(2000, 1, pressures)
    print("-----", obs.get_iteration())

def move_to_mid_pressures(factor = 1.0, duration=3000):
    pressures = [int(pressures_mid[i] * (1-factor) * (i<4) + pressures_mid[i]) for i in range(8)]
    pressures = [pressures]
    print("move_to_mid_pressures", pressures)
    obs = frontend.latest()
    print("-----", obs.get_iteration())
    open_loop_motion(3000, 1, pressures)
    obs = frontend.latest()
    print("-----", obs.get_iteration())

def move_2dof_right():
    pressures = pressures_mid.copy()
    pressures[1] = int(pressures_min[1] - 0.35*pressures_range[1])
    pressures[5] = int(pressures_min[5] + 0.35*pressures_range[1])
    pressures = [pressures]
    print("move_2dof_right", pressures)
    obs = frontend.latest()
    print("-----", obs.get_iteration())
    open_loop_motion(3000, 1, pressures)
    obs = frontend.latest()
    print("-----", obs.get_iteration())


def start_logging(logger, idx):
    if logger and logger!=None:
        print("stop logger")
        logger.stop() 
    segment_id = "real_robot"
    file_path = "/tmp/long_term_" + str(idx)
    frequency = 500
    logger = o80_pam.Logger(segment_id,file_path,frequency=frequency)
    print("start logger")
    logger.start() # spawn a process which starts dumping observations into the file
    return logger



def generate_multisine_signal(duration, sampling_frequency, max_frequency=10):
    frequencies = [0.1 * n for n in range(10*max_frequency)]
    amplitudes = [1] * len(frequencies)
    phases = [2 * np.pi * np.random.random() for _ in range(len(frequencies))]
    t = np.arange(0, duration, 1.0 / sampling_frequency)
    y = 0
    for idx, freq in enumerate(frequencies):
        y += amplitudes[idx] * np.sin(2 * np.pi * freq * t + phases[idx])
    y = y/max(abs(y)) #/2+0.5
    # plt.plot(t, y)
    # plt.show()
    return y

def generate_miltisine_pressure_trajectory(duration, sampling_frequency, max_frequency=10, max_amplitude = 1.0):
    rel_diff_between_min_and_max = [generate_multisine_signal(duration, sampling_frequency, max_frequency) for _ in range(4)]
    # rel_min_value_unnormalized = [generate_multisine_signal(duration, sampling_frequency, max_frequency) for _ in range(4)]
    amplitude = [1 * max_amplitude for _ in range(4)]
    print(amplitude)
    rand_values = [random.random() for _ in range(4)]
    print(rand_values)
    rel_min_value = [(1 - 1*amplitude[i%4]) * ((rand_values[i%4] * (i<4) + (1-rand_values[i%4]) * (i>=4))) + 0.5*amplitude[i%4]  for i in range(8)]
    

    print(rel_min_value)

    print([rel_diff_between_min_and_max[dof][0] for dof in range(4)])
    # print([(rel_min_value_unnormalized[dof][0]/2+0.5) * (1-abs(rel_diff_between_min_and_max[dof][0])) for dof in range(4)])

    # signal = [move_to_pressures_by_relative_values([abs(rel_diff_between_min_and_max[dof][i]) for dof in range(4)],
    #                                             [(rel_min_value_unnormalized[dof][i]/2+0.5) * (1-abs(rel_diff_between_min_and_max[dof][i])) for dof in range(4)],
    #                                             [rel_diff_between_min_and_max[dof][i] > 0 for dof in range(4)]) \
    #                                          for i in range(len(rel_diff_between_min_and_max[0]))]

    
    signal = [calc_pressures_by_relative_values2([rel_diff_between_min_and_max[dof][i]*0.5*amplitude[dof] for dof in range(4)],
                                                rel_min_value)
                                             for i in range(len(rel_diff_between_min_and_max[0]))]
    # print("--- signal ---")
    # print(signal)
    # t = np.arange(0, duration, 1.0 / sampling_frequency)
    # colors = ['red', 'green', 'blue', 'black']
    # # a = [[(rel_min_value_unnormalized[dof][i]/2+0.5) * (1-abs(rel_diff_between_min_and_max[dof][i])) for dof in range(4)] for i in range(len(rel_diff_between_min_and_max[0]))]
    # for idx in range(8):
    #     plt.plot(t, [s[idx] for s in signal], label=str(idx), color=colors[idx%4], linewidth=2+2*(idx//4))
    #     # plt.plot(rel_diff_between_min_and_max[idx%4], label=str(idx), color=colors[idx%4], linewidth=2+2*(idx//4))
    #     # plt.plot(t, [a_i[idx] for a_i in a], label=str(idx), color=colors[idx%4], linewidth=2+2*(idx//4))
    # plt.legend()
    # plt.show()
    return signal



def stop_logging(logger):
    print("stop logger")
    logger.stop()
    return logger


def move_to_reset_pressures():
    move_2dof_right()
    move_to_mid_pressures(1.0, duration = 200)
    move_to_mid_pressures(0.9, duration=200)
    move_to_mid_pressures(1.1, duration=200)
    move_to_mid_pressures(0.97, duration=200)
    move_to_mid_pressures(1.01, duration=200)
    move_to_min_pressures()
    move_to_mid_pressures(1.0, duration = 3000)

    # check position
    target_position = [0.0,0.90,0.40,0.0]
    obs = frontend.latest()
    positions = obs.get_positions()
    err = [abs(positions[i] - target_position[i])**2 for i in range(4)]
    print("positions: " + str(positions))
    print("err: " + str(err))
    print("err squared:", sum([x**2 for x in err]))
    if sum([x**2 for x in err])>0.1:
        print("ERROR: position not reached")
        input()



list_freq_amplitudes = [(10, 0.25), (7, 0.3), (4, 0.4)]


if __name__ == "__main__":
    logger = None
    frontend, robot_config = init_robot()
    should_exit = False

    idx_reset = []
    idx_random_movement = []
    idx_move_to_random_pressures = []
    idx_fixed_movement_slow = []
    idx_fixed_movement_fast = []
    
    for k in range(9999999999):
        if k%30==0:
            logger = start_logging(logger,k)
            

        for max_frequency, max_amplitude in list_freq_amplitudes:
            it_start = frontend.latest().get_iteration()
            rel_diff_between_min_and_max, rel_min_value, is_agonist_higher = move_to_random_pressures()
            it_end = frontend.latest().get_iteration()
            idx_move_to_random_pressures.append((it_start, it_end))
            
            print("max_frequency: " + str(max_frequency) + ", max_amplitude: " + str(max_amplitude))
            signal = generate_miltisine_pressure_trajectory(10, 500, max_frequency=max_frequency, max_amplitude = max_amplitude)

            it_start = frontend.latest().get_iteration()
            open_loop_motion(2000, 1, [signal[0]])
            it_end = frontend.latest().get_iteration()
            idx_move_to_random_pressures.append((it_start, it_end))

            it_start = frontend.latest().get_iteration()
            open_loop_motion(2, 1, signal)
            it_end = frontend.latest().get_iteration()
            idx_random_movement.append((it_start, it_end))

            it_start = frontend.latest().get_iteration()
            move_to_random_pressures(rel_diff_between_min_and_max, rel_min_value, is_agonist_higher)
            it_end = frontend.latest().get_iteration()
            idx_move_to_random_pressures.append((it_start, it_end))


            it_start = frontend.latest().get_iteration()
            open_loop_motion(2000, 1, [signal[0]])
            it_end = frontend.latest().get_iteration()
            idx_move_to_random_pressures.append((it_start, it_end))

            it_start = frontend.latest().get_iteration()
            open_loop_motion(2, 1, signal)
            it_end = frontend.latest().get_iteration()
            idx_random_movement.append((it_start, it_end))

        it_start = frontend.latest().get_iteration()
        move_to_reset_pressures()
        it_end = frontend.latest().get_iteration()
        idx_reset.append((it_start, it_end))

        it_start = frontend.latest().get_iteration()
        move_to_list_of_pressures(slow=True)
        it_end = frontend.latest().get_iteration()
        idx_fixed_movement_slow.append((it_start, it_end))

        it_start = frontend.latest().get_iteration()
        move_to_reset_pressures()
        it_end = frontend.latest().get_iteration()
        idx_reset.append((it_start, it_end))
        
        it_start = frontend.latest().get_iteration()
        move_to_list_of_pressures(slow=False)
        it_end = frontend.latest().get_iteration()
        idx_fixed_movement_fast.append((it_start, it_end))


        with open("commands", "r") as f:
            commands = f.readlines()
            commands = [c.strip() for c in commands]
            for c in commands:
                if c=="stop":
                    should_exit = True
                    print("stoping...")
                if c=="pause":
                    input("paused... press enter to continue")

        if (k+1)%30==0 or should_exit:
            file_path = "/tmp/long_term_" + str(k)
            with open (file_path+"_idx_reset", "wb") as f:
                pickle.dump(idx_reset, f)
            with open (file_path+"_idx_random_movement", "wb") as f:
                pickle.dump(idx_random_movement, f)
            with open (file_path+"_idx_move_to_random_pressures", "wb") as f:
                pickle.dump(idx_move_to_random_pressures, f)
            with open (file_path+"_idx_fixed_movement_slow", "wb") as f:
                pickle.dump(idx_fixed_movement_slow, f)
            with open (file_path+"_idx_fixed_movement_fast", "wb") as f:
                pickle.dump(idx_fixed_movement_fast, f)

            idx_reset = []
            idx_random_movement = []
            idx_move_to_random_pressures = []
            idx_fixed_movement_slow = []
            idx_fixed_movement_fast = []

        if should_exit:
            break

        

    logger = stop_logging(logger)

    move_to_min_pressures()
    

print("done")






    


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