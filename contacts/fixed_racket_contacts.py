"""
Ball colliding with robot racket from various angles.
Assumes `pam_mujoco pam_demos_contacts` has been called
from another terminal. 
"""

import math, time
import o80
from handle_contacts import MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID
from handle_contacts import get_robot_contact_handle


def run():

    # creating the robot and the ball
    handle = get_robot_contact_handle()

    # frontend for control of ball and robot
    ball = handle.frontends[BALL_SEGMENT_ID]
    robot = handle.frontends[ROBOT_SEGMENT_ID]

    # in case contact have been disabled during a previous run
    handle.reset_contact(BALL_SEGMENT_ID)


    def _velocity(p1, p2, duration_ms):
        # (constant) velocity when going from p1 to p2 in duration ms 
        return [(a - b) / (float(duration_ms) / 1000.0) for a, b in zip(p2, p1)]


    def _play_contact(start, end, duration=1000):

        # start and end should have been passed as relative
        # to the robot racket position, converting to
        # abs position
        cartesian_robot = robot.latest().get_cartesian_position()
        start = [s+cr for s,cr in zip(start,cartesian_robot)]
        end = [e+cr for e,cr in zip(end,cartesian_robot)]

        # deativating contacts
        handle.deactivate_contact(BALL_SEGMENT_ID)

        # going to start position
        ball.add_command(
            start, [0, 0, 0], o80.Duration_us.milliseconds(200), o80.Mode.QUEUE
        )
        ball.pulse_and_wait()

        # reativating contacts
        handle.activate_contact(BALL_SEGMENT_ID)

        # starting with clean contact
        handle.reset_contact(BALL_SEGMENT_ID)

        # set the ball to start position
        ball.add_command(
            start, [0, 0, 0], o80.Duration_us.milliseconds(400), o80.Mode.QUEUE
        )
        ball.pulse_and_wait()

        # having the ball going to the end position
        ball.add_command(
            end,
            _velocity(start, end, duration),
            o80.Duration_us.milliseconds(duration),
            o80.Mode.QUEUE,
        )
        ball.pulse_and_wait()

        time.sleep(0.5)

    # target position of robot
    joints = (-math.pi / 4.0, +math.pi / 4.0, -math.pi / 4.0, +math.pi / 4.0)
    # set the robot to position
    robot.add_command(
        joints, (0, 0, 0, 0), o80.Duration_us.milliseconds(400), o80.Mode.QUEUE
    )
    robot.pulse_and_wait()

    # different speed
    start = (0, 2.0, 0)
    end = (0, -5.0, 0)
    _play_contact(start, end, 3000)
    _play_contact(start, end, 1000)

    # bouncing left/right
    start = (0, 2.0, 0)
    end = (0, -5.0, 0)
    _play_contact(start, end, 1000)
    end = (-0.2, -5.0, 0)
    _play_contact(start, end, 1000)
    end = (+0.2, -5.0, 0)
    _play_contact(start, end, 1000)

    # bouncing left/right (2)
    start = (-0.5, 2.0, 0)
    end = (0, -0.1, 0)
    _play_contact(start, end, 500)
    start = (+0.5, 2.0, 0)
    _play_contact(start, end, 500)

    # bouncing top/down 
    start = (0, 2.0, -0.5)
    end = (0, -0.1, 0)
    _play_contact(start, end, 500)
    start = (+0.5, 2.0, 0.5)
    _play_contact(start, end, 500)


if __name__ == "__main__":
    run()
