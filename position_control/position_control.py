import time
import math
import o80
import pam_interface
import o80_pam
import pam_mujoco
import random
import numpy as np


real_robot = True


if not real_robot:

    mujoco_id = "position_control"
    robot_segment_id = "robot"
    burst_mode=False
    accelerated_time=False

    def _get_handle():

        control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
        graphics = True
        pamy = pam_mujoco.RobotType.PAMY1
        robot = pam_mujoco.MujocoRobot(pamy,robot_segment_id, control=control)
        handle = pam_mujoco.MujocoHandle(
            mujoco_id, robot1=robot, graphics=graphics, accelerated_time=accelerated_time,
            burst_mode=burst_mode
        )
        return handle,robot.json_control_path


    # getting the handle and the json file used to configure
    # the robot (to get min and max pressures)
    handle,robot_config_path = _get_handle()

    # frontend to robot
    frontend = handle.frontends[robot_segment_id]
    robot_config = pam_interface.JsonConfiguration(robot_config_path)
    
else:

    frontend = o80_pam.FrontEnd("real_robot")
    robot_config = pam_interface.Pamy2DefaultConfiguration(False)


# will be used for plotting
starting_iteration = frontend.latest().get_iteration()
    
# configuring the controller

kp = [0.4309, 1.212, 0.55, .2]#[0.06,0.6,0.4,0.03] #[0.05,0.5,0.3,0.01]
kd = [0.04978, 0.1712, 0.0, 0.0]
ki = [0.05629, 0.08202, .11, .25]#[0.035,0.07,0.06,0.07]#[0.0275,0.055,0.04,0.05]
ndp = [.9]*4#[0.5,0.6,0.5,0.5]
time_step = 0.05
o80_time_step = o80.Duration_us.milliseconds(int(time_step*1000))
extra_steps = 15
dq_desired = [math.pi*.75]*4#[0.6, 0.5, 0.5, 0.5]
position_controller_factory = o80_pam.position_control.PositionControllerFactory(
    dq_desired,
    robot_config,
    kp,kd,ki,ndp,
    time_step,extra_steps
)

# going to desired position using the controller
def go_to(target_position,p):

    def control():
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
        if p:
            # evaluate if tracking sufficient
            # print(np.asarray(velocities).T)
            # print(max(abs(np.mean(velocities,axis=1))))
            print("mean vels: ",np.mean(velocities,axis=1))
            
            # print()
            # print([i/math.pi*180 for i in target_position])
            # print([i/math.pi*180 for i in position])
            print("errs: ",[i/math.pi*180 for i in error])

        return position, velocities, error
    
    
    t = time.time()
    position, velocities, error = control()
    its=0
    while ( max([abs(i/math.pi*180) for i in error]) > 10 or max(abs(np.mean(velocities,axis=1))) >1.0 ) and p:
        if its<10:
            for dof in range(4):
                frontend.add_command(dof,25000,25000,o80.Duration_us.seconds(1),o80.Mode.OVERWRITE)
            frontend.pulse_and_wait()
            print("   ",its+1,"another time ... woohoo: ")
            print("   errs: ",[abs(i/math.pi*180) for i in error])
            print("   vels: ",abs(np.mean(velocities,axis=1)))
            position, velocities, error = control()
            its +=1
        else:
            print("STOPPING: could not move to target pos with sufficient accuracy!")
            break
    print("elapsed time: ",time.time() - t)
        
# desired positions
pi2 = math.pi/2.0
pi4 = math.pi/4.0
pi6 = math.pi/6.0

target_position2 = [0,+pi4,0,0]

# starting in a vertical position

n_its = 10
for i in range(n_its):
    # calling twice in a row for each target position
    target_position1 = [random.uniform(-pi2, pi2), random.uniform(-pi2, pi2),
                        random.uniform(-pi2, pi2), random.uniform(-pi2, pi2)]
    # target_position2 = [random.uniform(-pi2, pi2), random.uniform(-pi2, pi2),
    #                     random.uniform(-pi2, pi2), random.uniform(-pi2, pi2)]
    print("---------")
    print("it ",i+1," / ",n_its)
    time.sleep(2)
    go_to(target_position1,False)
    # time.sleep(1)
    go_to(target_position2,True)


# plotting motions of joints
# observations = frontend.get_observations_since(starting_iteration)
# dofs = [[obs.get_positions()[dim]/math.pi*180 for obs in observations] for dim in range(4)]
# x = [i for i in range(len(observations))]

# import matplotlib.pyplot as plt

# for dim,history in enumerate(dofs):
#     plt.scatter(x,dofs[dim])
# plt.grid()
# plt.xlabel("time")
# plt.ylabel("joint angle in deg")
# plt.show()
