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


from tinkerforge.ip_connection import IPConnection
from tinkerforge.bricklet_thermal_imaging import BrickletThermalImaging


real_robot = True
pressures_mid = [20000, 18250, 17000, 19000, 20500, 20500, 17000, 19000]
pressures_half_range = [5000, 3250, 4000, 6000, 4500, 4500, 4000, 6000]

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

def init_position_controller(robot_config):
    kp = [0.4309, 1.212, 0.55, .2]                                #[0.06,0.6,0.4,0.03] #[0.05,0.5,0.3,0.01]
    #kp = [p * 0.8 for p in kp]
    kd = [0.04978, 0.1712, 0.0, 0.0]
    ki = [0.05629, 0.08202, .11, .25]                           #[0.035,0.07,0.06,0.07]#[0.0275,0.055,0.04,0.05]
    #ki = [p * 0.8 for p in ki]
    ndp = [.9]*4                                                 #[0.5,0.6,0.5,0.5]
    time_step = 0.05
    o80_time_step = o80.Duration_us.milliseconds(int(time_step*1000))
    extra_steps = 25
    dq_desired = [math.pi*.5]*4#[0.6, 0.5, 0.5, 0.5]
    position_controller_factory = o80_pam.position_control.PositionControllerFactory(
        dq_desired,
        robot_config,
        kp,kd,ki,ndp,
        time_step,extra_steps
    )
    return time_step, o80_time_step, position_controller_factory



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
        


def control(target_position):
    # starting position
    current_position = frontend.latest().get_positions()

    # constructing the controller
    controller = position_controller_factory.get(current_position,target_position)

    # for managing the frequency
    frequency = o80.FrequencyManager(1./time_step)

    
    # running the controller
    while controller.has_next():

        # current state
        obs = frontend.latest()
        positions = obs.get_positions()
        velocities = obs.get_velocities()

        # getting suitable pressures from the controller
        pressures = controller.next(positions,velocities)

        # applying the pressure
        for dof, (p_ago,p_antago) in enumerate(pressures):
            frontend.add_command(dof,p_ago,p_antago,o80_time_step,o80.Mode.OVERWRITE)
        frontend.pulse()

        # making sure we run at the controller
        # suitable frequency
        frequency.wait()
    
    # evaluating results
    position = frontend.latest().get_positions()
    current_it = frontend.latest().get_iteration()
    current_obs = frontend.get_observations_since(current_it-50)
    velocities = [[c_obs.get_velocities()[dim] for c_obs in current_obs] for dim in range(4)]
    error = [abs(p-t) for p,t in zip(position,target_position)]

    return position, velocities, error

# going to desired position using the controller
def go_to(target_position, error_check = True):
    t = time.time()
    for i in range(10):
        position, velocities, error = control(target_position)
        
        if not ( max([abs(i/math.pi*180) for i in error]) > 5 or max(abs(np.mean(velocities,axis=1))) > 0.5 ):
            return
        elif not error_check:
            print("error:", [abs(i/math.pi*180) for i in error], "velocities", np.mean(velocities,axis=1))
            print("ignore...")
            return
        else:
            print("error crit:", max([abs(i/math.pi*180) for i in error]), max(abs(np.mean(velocities,axis=1))))
            print("error:", [abs(i/math.pi*180) for i in error], "velocities", np.mean(velocities,axis=1))
            time.sleep(1)
            print("continuing...")
    
    print("target position not reached...")
    print("error:", [abs(i/math.pi*180) for i in error], "velocities", np.mean(velocities,axis=1))
    print("pausing...")
    input()

def move_to_zero_position():
    target_position = [0,0,0,0]
    go_to(target_position)
    pressures_current = frontend.latest().get_observed_pressures()
    print(pressures_current)
    pressures = [[20500, 20000, 20000, 25000, 20000, 20500, 20000, 20000]]
    open_loop_motion(500, 1, pressures)

def move_to_random_position():
    target_position = [np.random.random() * np.pi - np.pi/2] * 4
    go_to(target_position)

def move_to_list_of_positions():
    position_list = [[-1, 1, -1, 1], [1, 1, 1, -1], [1, -1, -1, 1], [-1, -1, 1, -1],
                     [1, 1, 1, 1], [-1, 1, -1, -1], [-1, -1, 1, 1], [1, -1, -1, -1],
                    [-1, 1, 0, 1], [1, 1, 0, -1], [1, -1, 0, 1], [-1, -1, 0, -1]]
    for position in position_list:
        target_position = [x * np.pi * 0.4 for x in position]
        print(target_position)
        go_to(target_position, error_check = False)
        time.sleep(0.05)

def move_to_list_of_pressures():
    position_list = [[-1, 1.0 * 0.5, -1, 1], [1, 0.8 * 0.5, 1, -1], [1, 0 * 0.5, -1, 1], [-1, -0.4 * 0.5, 1, -1],
                     [1, 0.8 * 0.5, 1, 1], [-1, 0.8 * 0.5, -1, -1], [-1, -0.4 * 0.5, 1, 1], [1, -0.8 * 0.5, -1, -1],
                    [-1, 0.8 * 0.5, 0, 1], [1, 0.8 * 0.5, 0, -1], [1, -0.2 * 0.5, 0, 0.5], [-1, -0.6 * 0.5, 0, -0.8]]
    for position in position_list:
        print(position)
        pressures = np.multiply(position + [-p for p in position], pressures_half_range) * 0.9 + np.array(pressures_mid)
        pressures = pressures.astype(int)
        pressures = [x.tolist() for x in pressures]
        print(pressures)
        open_loop_motion(1300, 1, [pressures])


def fast_hitting_motions(n, log=False):

    pressureAGO_0 = [15000, 15000, 15000, 15000]
    pressureANT_0 = [30000, 30000, 30000, 30000]
    pressureAGO_1 = [30000, 30000, 30000, 30000]
    pressureANT_1 = [15000, 15000, 15000, 15000]
    duration = 625  # Default: 625 ms (aggressiv but not dangerous)

    # go to initial position
    open_loop_motion(2000, 1, [pressureAGO_0 + pressureANT_0])

    if log:
        first_iteration = frontend.latest().get_iteration()

    # do fast movement n times
    open_loop_motion(duration, n, [pressureAGO_0 + pressureANT_0, pressureAGO_1 + pressureANT_1])

    if log:
        outname = "data/obs_fast_hitting_ " + str(duration) + "_duration_" + str(n) + "_times_"
        now = datetime.now()
        now_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        file_name = f"{outname}_{now_str}"
        with open(str(file_name), "wb+") as f:
            serializer = o80_pam.Serializer()
            observations = frontend.get_observations_since(first_iteration)
            for observation in observations:
                f.write(serializer.serialize(observation))
            f.flush()


def fast_hitting_motions_bowden():

    pressureAGO_0 = [18000, 16000, 16000, 16000]
    pressureANT_0 = [18000, 21000, 21000, 21000]
    pressureAGO_1 = [18000, 21000, 21000, 21000]
    pressureANT_1 = [18000, 16000, 16000, 16000]
    duration = 300  # Default: 625 ms (aggressiv but not dangerous)

    # do fast movement n times
    t_end = time.time() + 60
    while time.time() < t_end:
        open_loop_motion(duration, 1, [pressureAGO_0 + pressureANT_0, pressureAGO_1 + pressureANT_1])
    

def open_loop_sin_motion(freq, phase, ampl, t, pressures_mid):
    freq = np.array(freq)
    phase = np.array(phase)
    resolution = 100
    length = t * 2 * np.pi * freq
    sin_wave = np.array([np.sin(np.arange(0, length[i], 1 / resolution * freq[i]) + phase[i]) for i in range(len(phase))])
    sin_wave = np.swapaxes(sin_wave, 0, 1)
    pressures = pressures_mid + sin_wave * ampl * pressures_half_range
    pressures = pressures.astype(int)
    pressures = pressures.tolist()
    duration = int(1/resolution * 100)
    #print(pressures)
    # import matplotlib.pyplot as plt
    # plt.plot(pressures)
    # plt.show()

    open_loop_motion(1000, 1, [pressures[0]])
    print(len(pressures))
    print(duration)
    open_loop_motion(duration, 1, pressures)

    return

def repeatability_motion():
    
    pressures_mid = [20000, 19800, 17000, 19000, 20500, 19500, 17000, 19000]

    for i in range(2):
        move_to_zero_position()
    freq = [0.4] * 8
    phase = [0, 0, np.pi, 0, np.pi, np.pi, 0, np.pi]
    amp = [0.7, 0.3, 0.5, 0.6] * 2
    duration = 5
    
    first_iteration = frontend.latest().get_iteration()

    open_loop_sin_motion(freq, phase, amp, duration, pressures_mid)

    outname = "data/obs_log"
    now = datetime.now()
    now_str = now.strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"{outname}_{now_str}"
    with open(str(file_name), "wb+") as f:
        serializer = o80_pam.Serializer()
        observations = frontend.get_observations_since(first_iteration)
        for observation in observations:
            f.write(serializer.serialize(observation))
        f.flush()


def repeatability_motion2():
    
    pressures_mid =[20000, 19800, 17000, 19000, 20500, 20500, 17000, 19000]

    for i in range(2):
        move_to_zero_position()
    freq = [0.4] * 8
    phase = [0, 0, np.pi, 0, np.pi, np.pi, 0, np.pi]
    amp = [0.7, 0.3, 0.5, 0.6] * 2
    duration = 5
    
    first_iteration = frontend.latest().get_iteration()

    open_loop_sin_motion(freq, phase, amp, duration, pressures_mid)

    outname = "data/obs_log"
    now = datetime.now()
    now_str = now.strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"{outname}_{now_str}"
    with open(str(file_name), "wb+") as f:
        serializer = o80_pam.Serializer()
        observations = frontend.get_observations_since(first_iteration)
        for observation in observations:
            f.write(serializer.serialize(observation))
        f.flush()



def random_open_loop_motion(duration):
    freq = [np.random.random() * 0.5 for i in range(8)]
    phase = [np.random.random() * np.pi for i in range(8)]
    amp = [np.random.random() for i in range(8)]
    print("freq", freq)
    print("phase", phase)
    print("amp", amp)
    print("......")
    open_loop_sin_motion(freq, phase, amp, duration, pressures_mid)


def capture_image():
    #
    # Takes one thermal image and saves it as PNG
    #
    HOST = "localhost"
    PORT = 4223
    camera_uid = "Vi1"
    high_contrast = False
    outname = "data/thermal_img"

    ipcon = IPConnection() # Create IP connection
    ti = BrickletThermalImaging(camera_uid, ipcon) # Create device object
    ipcon.connect(HOST, PORT) # Connect to brickd


    # Don't use device before ipcon is connected

    if high_contrast:
        ti.set_image_transfer_config(ti.IMAGE_TRANSFER_MANUAL_HIGH_CONTRAST_IMAGE)
    else:
        ti.set_image_transfer_config(ti.IMAGE_TRANSFER_MANUAL_TEMPERATURE_IMAGE)

    # If we change between transfer modes we have to wait until one more
    # image is taken after the mode is set and the first image is saved
    # we can call get_high_contrast_image any time.
    time.sleep(0.5)

    if high_contrast:
        image_data = np.array(ti.get_high_contrast_image())
    else:
        image_data = np.array(ti.get_temperature_image())

    now = datetime.now()
    now_str = now.strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"{outname}_{now_str}"
    np.save(file_name + ".npy", image_data)

    ipcon.disconnect()




if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("data", type=str)
    args = parser.parse_args()
    print(args.data)

    frontend, robot_config = init_robot()
    time_step, o80_time_step, position_controller_factory = init_position_controller(robot_config)

    # for i in range(3):
    #     print("...")
    #     repeatability_motion()

    #move_to_zero_position()
    for i in range(10):
        print("------", i+1, "/ 2000 --------")
        fast_hitting_motions(5)
        # move_to_zero_position()
        # move_to_list_of_positions()
        # move_to_list_of_pressures()
        # repeatability_motion2()
        # move_to_zero_position()
        # time.sleep(0.1)
        # capture_image()
        # time.sleep(0.1)
        
        
    print("done")
    #repeatability_motion()
    # #repeatability experiment
    # for i in range(2):
    #     fast_hitting_motions(2)
    #     repeatability_motion()
    #     capture_image()
    