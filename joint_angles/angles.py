import math,time
import matplotlib.pyplot as plt
import o80,pam_mujoco


mujoco_id = "angles"
robot_segment_id = "robot"
control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
graphics=True
accelerated_time=False
robot = pam_mujoco.MujocoRobot(robot_segment_id,
                               control=control)
handle = pam_mujoco.MujocoHandle(mujoco_id,
                                 robot1=robot,
                                 graphics=graphics,
                                 accelerated_time=accelerated_time)


frontend = handle.frontends["robot"]

high_pressure = 22000

all_positions = []

for dof in range(4):

    frontend.add_command([high_pressure]*4,[high_pressure]*4,
                         o80.Duration_us.seconds(1),
                         o80.Mode.OVERWRITE)
    
    frontend.pulse_and_wait()
    
    
    p1 = 12000
    p2 = 22000

    duration = 2
    
    frontend.add_command(dof,
                         p1,p2,
                         o80.Duration_us.seconds(duration),
                         o80.Mode.QUEUE)
    frontend.add_command(dof,
                         p2,p1,
                         o80.Duration_us.seconds(duration),
                         o80.Mode.QUEUE)

    time_start = time.time()

    positions = []
    
    while time.time()-time_start < 2*duration:

        observation = frontend.pulse()
        position = observation.get_positions()[dof]
        positions.append(position)

    plt.plot(positions)
    plt.show()




    
