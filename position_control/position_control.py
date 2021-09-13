import time
import math
import pam_interface
import o80_pam
import pam_mujoco


mujoco_id = "position_control"
robot_segment_id = "robot"
burst_mode=False

def _get_handle():

    control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
    graphics = True
    accelerated_time = False
    robot = pam_mujoco.MujocoRobot(robot_segment_id, control=control)
    handle = pam_mujoco.MujocoHandle(
        mujoco_id, robot1=robot, graphics=graphics, accelerated_time=accelerated_time
    )
    return handle,robot.json_control_path



# getting the handle and the json file used to configure
# the robot (to get min and max pressures)
handle,robot_config_path = _get_handle()

# read robot configuration
robot_config = pam_interface.JsonConfiguration(robot_config_path)

# configuring the controller
ctrl_config = o80_pam.JointPositionControllerConfig(handle.interfaces[robot_segment_id],
                                                    robot_config)

# instantiating the controller
ctrl = o80_pam.JointPositionController(ctrl_config,burst_mode)


# going to a position
error = [0.1]*4
pi4 = math.pi/4.0
pi6 = math.pi/6.0
target_position1 = [+pi4,pi6,-pi4,pi4]
target_position2 = [-pi4,pi4,pi4,pi6]

for finished,positions,velocities,errors in ctrl.go_to(target_position1,q_err=error,timeout_s=10):
    if finished is not None:
        break
    time.sleep(0.01)

for finished,positions,velocities,errors in ctrl.go_to(target_position2,q_err=error):
    if finished is not None:
        break
    time.sleep(0.01)

    


