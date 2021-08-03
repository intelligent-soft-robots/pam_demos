import math, time
import o80
from handle_contacts import MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID
from handle_contacts import get_robot_contact_handle

handle = get_robot_contact_handle()

ball = handle.frontends[BALL_SEGMENT_ID]
robot = handle.frontends[ROBOT_SEGMENT_ID]

handle.reset_contact(BALL_SEGMENT_ID)


def _velocity(p1, p2, duration):
    return [(a - b) / duration for a, b in zip(p2, p1)]


def _play_contact(
    joints_start, joints_end, duration_racket, ball_start, ball_end, duration_ball
):

    global handle, ball, robot

    # deativating contacts
    handle.deactivate_contact(BALL_SEGMENT_ID)

    # going to start position
    ball.add_command(
        ball_start, [0, 0, 0], o80.Duration_us.milliseconds(200), o80.Mode.QUEUE
    )
    ball.pulse_and_wait()
    robot.add_command(
        joints_start, (0, 0, 0, 0), o80.Duration_us.milliseconds(200), o80.Mode.QUEUE
    )
    robot.pulse_and_wait()

    # reativating contacts
    handle.activate_contact(BALL_SEGMENT_ID)

    # starting with clean contact
    handle.reset_contact(BALL_SEGMENT_ID)

    # going to end positions
    ball.add_command(
        ball_end,
        _velocity(ball_start, ball_end, duration_ball * 1e-3),
        o80.Duration_us.milliseconds(duration_ball),
        o80.Mode.QUEUE,
    )
    robot.add_command(
        joints_end,
        (0, 0, 0, 0),
        o80.Duration_us.milliseconds(duration_racket),
        o80.Mode.QUEUE,
    )

    ball.pulse()
    robot.pulse()

    # sleep time of motion + 1 seconds
    handle.sleep(max(duration_racket * 1e-3, duration_ball * 1e-3) + 1, BALL_SEGMENT_ID)


ball_starts = [(+1.10, +1.50, +0.05), (+1.50, +1.50, +0.05), (+0.80, +1.50, +0.05)]
ball_ends = [(+1.15, -1.00, +0.00), (+1.00, -1.00, +0.00), (+1.45, -1.00, +0.00)]

for ball_start, ball_end in zip(ball_starts, ball_ends):
    joints_start = (-math.pi / 3.0, +math.pi / 2.5, -math.pi / 8.0, 0.1)
    joints_end = (+math.pi / 3.0, +math.pi / 2.5, -math.pi / 8.0, 0.1)
    _play_contact(joints_start, joints_end, 500, ball_start, ball_end, 500)
