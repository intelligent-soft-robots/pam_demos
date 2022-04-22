import time
import math
import o80
import pam_interface
import o80_pam
import pam_mujoco


real_robot = False


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

# configuring the controller

kp = [0.2,0.2,0.2,0.2]
kd = [0.02,0.02,0.02,0.02]
ki = [0.05,0.05,0.05,0.05]
ndp = [0.5,0.5,0.6,0.5]
time_step = 0.05
o80_time_step = o80.Duration_us.milliseconds(int(time_step*1000))
extra_steps = 20
dq_desired = [0.3]*4
position_controller_factory = o80_pam.position_control.PositionControllerFactory(
    dq_desired,
    robot_config,
    kp,kd,ki,ndp,
    time_step,extra_steps
)

# going to desired position using the controller
def go_to(target_position):

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
    error = [abs(p-t) for p,t in zip(position,target_position)]
    print()
    print(target_position)
    print(position)
    print(error)
    print()
    
        

# desired positions
pi4 = math.pi/4.0
pi6 = math.pi/6.0
target_position1 = [+pi4,+pi6,-pi6,+pi4]
target_position2 = [-pi4,-pi6,+pi6,+pi6]

# starting in a vertical position
for dof in range(4):
    frontend.add_command(dof,25000,25000,o80.Duration_us.seconds(3),o80.Mode.OVERWRITE)
frontend.pulse_and_wait()
    

# calling twice in a row for each target position
go_to(target_position1)
go_to(target_position1)

go_to(target_position2)
go_to(target_position2)

go_to(target_position1)
go_to(target_position1)
