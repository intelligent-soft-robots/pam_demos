import o80
import pam_interface
import o80_pam
import pam_mujoco

real_robot = True

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


# parameters
starting_iteration = frontend.latest().get_iteration()
pressureAGO_0 = [15000, 15000, 15000, 15000]
pressureANT_0 = [30000, 30000, 30000, 30000]
pressureAGO_1 = [30000, 30000, 30000, 30000]
pressureANT_1 = [15000, 15000, 15000, 15000]
duration = 3000  # Default: 625 ms (aggressiv but not dangerous)

# bring robot to initial position
frontend.add_command(pressureAGO_0, pressureANT_0,
                     o80.Duration_us.milliseconds(duration),
                     o80.Mode.OVERWRITE)
frontend.pulse_and_wait()
input("press key...")

# run repetitive hitting motions
for i in range(1):
    # creating a command locally. The command is *not* sent to the robot yet.
    frontend.add_command(pressureAGO_0, pressureANT_0,
                         o80.Duration_us.milliseconds(duration),
                         o80.Mode.QUEUE)
    frontend.add_command(pressureAGO_1, pressureANT_1,
                         o80.Duration_us.milliseconds(duration),
                         o80.Mode.QUEUE)

    # sending the command to the robot, and waiting for its completion.
    frontend.pulse_and_wait()
    

# bring robot to initial position for the next run
frontend.add_command(pressureAGO_0, pressureANT_0,
                     o80.Duration_us.milliseconds(duration),
                     o80.Mode.QUEUE)
frontend.pulse_and_wait()
