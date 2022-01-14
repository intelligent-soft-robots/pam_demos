import time
import math
import pam_interface
import o80_pam
import pam_mujoco


mujoco_id = "position_control"
robot_segment_id = "robot"
burst_mode = False
accelerated_time = False


def _get_handle():

    control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
    graphics = True
    robot = pam_mujoco.MujocoRobot(robot_segment_id, control=control)
    handle = pam_mujoco.MujocoHandle(
        mujoco_id,
        robot1=robot,
        graphics=graphics,
        accelerated_time=accelerated_time,
        burst_mode=burst_mode,
    )
    return handle, robot.json_control_path


# getting the handle and the json file used to configure
# the robot (to get min and max pressures)
handle, robot_config_path = _get_handle()
pressure_interface = handle.interfaces[robot_segment_id]

# frontend to robot
frontend = handle.frontends[robot_segment_id]

# configuring the controller
robot_config = pam_interface.JsonConfiguration(robot_config_path)
kp = [200.0] * 4
kd = [0.0] * 4
ki = [0.0] * 4
controller = o80_pam.JointPositionController(robot_config, kp, kd, ki)

# going to positions
timeout = 4.0  # seconds
accepted_error = 0.1
pi4 = math.pi / 4.0
pi6 = math.pi / 6.0
target_position1 = [+pi4, pi6, -pi4, pi4]
target_position2 = [-pi4, pi4, -pi6, pi6]


def _go_to(target_position):
    def _eval_error():
        _, _, position, _ = pressure_interface.read()
        return [abs(p - t) for p, t in zip(position, target_position)]

    def _loop():
        time_start = frontend.latest().get_time_stamp() * 1e-9
        current_time = time_start
        while current_time - time_start < timeout:
            current_time = frontend.latest().get_time_stamp() * 1e-9
            desired_pressures = controller.go_to(
                pressure_interface, target_position, current_time
            )
            pressure_interface.set(desired_pressures, burst=burst_mode)
            errors = _eval_error()
            if max(errors) < accepted_error:
                return True
            handle.sleep(0.05, robot_segment_id)
        return False

    def _round(a):
        template = "{:.3f} " * len(a)
        return template.format(*a)

    pressure_interface.set([(18000, 18000)] * 4, duration_ms=500, wait=True)
    time.sleep(2)

    finished = _loop()

    if not finished:
        errors = _eval_error()
        print(
            "failed to reach {} in {} seconds. errors: {}".format(
                _round(target_position), timeout, _round(errors)
            )
        )

    return


_go_to(target_position1)
_go_to(target_position2)
