import matplotlib.pyplot as plt
import math
import time
import o80
import pam_mujoco

mujoco_id = "fk"
segment_id = "robot_fk"

graphics = True
accelerated_time = False
burst_mode = False

robot_control = pam_mujoco.MujocoRobot(
    segment_id, control=pam_mujoco.MujocoRobot.JOINT_CONTROL
)
handle = pam_mujoco.MujocoHandle(
    mujoco_id,
    graphics=graphics,
    accelerated_time=accelerated_time,
    burst_mode=burst_mode,
    robot1=robot_control,
)

robot = handle.frontends[segment_id]


def go_to_posture(postures, robot, duration_ms=1000):

    for posture in postures:
        robot.add_command(
            posture,
            (0, 0, 0, 0),
            o80.Duration_us.milliseconds(duration_ms),
            o80.Mode.QUEUE,
        )
        robot.pulse_and_wait()


# for checking forward kinematics position

print("\n\nchecking cartesian fk position\n\n")


def _cartesian_plot(robot, posture_start, posture_end):

    print("start: {}, end: {}".format(posture_start, posture_end))

    go_to_posture([posture_start], robot)

    start_iteration = robot.latest().get_iteration()

    go_to_posture((posture_start, posture_end), robot)

    observations = robot.get_observations_since(start_iteration)
    cartesian_positions = [o.get_cartesian_position() for o in observations]

    x = [cp[0] for cp in cartesian_positions]
    y = [cp[1] for cp in cartesian_positions]
    z = [cp[2] for cp in cartesian_positions]

    plt.plot(x, label="x")
    plt.plot(y, label="y")
    plt.plot(z, label="z")

    legend = plt.legend()

    plt.show()


pi2 = math.pi / 2.0


_cartesian_plot(robot, (0, 0, +pi2, 0), (0, 0, -pi2, 0))

_cartesian_plot(robot, (0, 0, 0, +pi2), (0, 0, 0, -pi2))

_cartesian_plot(robot, (pi2, +pi2, 0, 0), (pi2, -pi2, 0, 0))


# for checking forward kinematics orientation

print("\n\nchecking cartesian fk orientation\n\n")


def _angles_from_matrix(matrix):

    x = math.atan2(matrix[2][1], matrix[2][2])
    y = math.atan2(-matrix[2][0], math.sqrt(matrix[2][1] ** 2 + matrix[2][2] ** 2))
    z = math.atan2(matrix[1][0], matrix[0][0])

    return (x, y, z)


def _print_orientation_matrix(robot, posture):

    go_to_posture([posture], robot)

    # 9 dim array
    orientation = robot.latest().get_cartesian_orientation()

    # 3*3 orientation
    matrix = [
        [o for o in orientation[0:3]],
        [o for o in orientation[3:6]],
        [o for o in orientation[6:9]],
    ]

    for dim in range(3):
        print(
            "| {:.2f}\t{:.2f}\t{:.2f} |".format(
                matrix[dim][0], matrix[dim][1], matrix[dim][2]
            )
        )
    angles = _angles_from_matrix(matrix)
    print("x:", angles[0])
    print("y:", angles[1])
    print("z:", angles[2])
    print()
    time.sleep(2)


_print_orientation_matrix(robot, (0, 0, 0, 0))

_print_orientation_matrix(robot, (0, 0, 0, +pi2))

_print_orientation_matrix(robot, (0, +pi2, 0, +pi2))

_print_orientation_matrix(robot, (0, +pi2, 0, 0))

_print_orientation_matrix(robot, (pi2, 0, pi2, 0))

_print_orientation_matrix(robot, (pi2, 0, pi2, pi2))
