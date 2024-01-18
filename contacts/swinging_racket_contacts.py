import math
import time
import o80
from handle_contacts import (
    MUJOCO_ID,
    ROBOT1_SEGMENT_ID,
    ROBOT2_SEGMENT_ID,
    BALL_SEGMENT_ID,
)
from handle_contacts import get_handle


def run():
    robot2_position = (0.85, 3.20, 0.8)
    handle = get_handle(True, True, True, robot2_position)

    ball = handle.frontends[BALL_SEGMENT_ID]
    robot1 = handle.frontends[ROBOT1_SEGMENT_ID]
    robot2 = handle.frontends[ROBOT2_SEGMENT_ID]
    items = ("table", "robot1", "robot2")

    for item in items:
        handle.reset_contact(BALL_SEGMENT_ID, item)

    def _velocity(p1, p2, duration):
        return [(a - b) / duration for a, b in zip(p2, p1)]

    def _play_contact(
        robot2_joints,
        joints_start,
        joints_end,
        duration_racket,
        ball_start,
        ball_end,
        duration_ball,
    ):
        # deativating contacts
        for item in items:
            handle.deactivate_contact(BALL_SEGMENT_ID, item)

        # robot1 going to start position
        robot1.add_command(
            joints_start,
            (0, 0, 0, 0),
            o80.Duration_us.milliseconds(200),
            o80.Mode.QUEUE,
        )
        robot1.pulse_and_wait()

        # robot2 going to start position
        robot2.add_command(
            robot2_joints,
            (0, 0, 0, 0),
            o80.Duration_us.milliseconds(200),
            o80.Mode.QUEUE,
        )
        robot2.pulse_and_wait()

        # ball_start and ball_end are relative to the racket position
        cartesian_robot = robot1.latest().get_cartesian_position()
        ball_start = [s + cr for s, cr in zip(ball_start, cartesian_robot)]
        ball_end = [e + cr for e, cr in zip(ball_end, cartesian_robot)]

        # ball going to start position
        ball.add_command(
            ball_start, [0, 0, 0], o80.Duration_us.milliseconds(200), o80.Mode.QUEUE
        )
        ball.pulse_and_wait()

        # reativating contacts
        for item in items:
            handle.activate_contact(BALL_SEGMENT_ID, item)

        # starting with clean contact
        for item in items:
            handle.reset_contact(BALL_SEGMENT_ID, item)

        # going to end positions
        ball.add_command(
            ball_end,
            _velocity(ball_start, ball_end, duration_ball * 1e-3),
            o80.Duration_us.milliseconds(duration_ball),
            o80.Mode.QUEUE,
        )
        robot1.add_command(
            joints_end,
            (0, 0, 0, 0),
            o80.Duration_us.milliseconds(duration_racket),
            o80.Mode.QUEUE,
        )

        ball.pulse()
        robot1.pulse()

        # sleep time of motion + 1 seconds
        handle.sleep(
            max(duration_racket * 1e-3, duration_ball * 1e-3) + 1, BALL_SEGMENT_ID
        )

        # report contacts
        for item in items:
            contact = handle.get_contact(BALL_SEGMENT_ID, item)
            if contact.contact_occured:
                print(
                    f"{item}: contact | time stamp: {contact.time_stamp} | position: {contact.position}"
                )
            else:
                print(f"{item}: no contact | min distance: {contact.minimal_distance}")

    ball_starts = [(+0.25, +2.00, +0.00), (-0.24, +2.00, +0.00), (+0.66, +2.00, +0.00)]
    ball_ends = [(+0.25, -1.00, +0.00), (+0.66, -1.00, -0.00), (-0.24, -1.00, -0.00)]

    # ball_starts = [(+0.25, +2.00, +0.00)]
    # ball_ends = [(+0.25, -1.00, +0.00)]

    # durations_robot = [500, 550, 550]
    durations_robot = [450]

    durations_ball = [450, 510, 410]

    robot2_joints = (0, +math.pi / 2.5, -math.pi / 8.0, 0.1)

    joints_start = (-math.pi / 3.0, +math.pi / 2.5, -math.pi / 8.0, 0.1)
    joints_end = (+math.pi / 3.0, +math.pi / 2.5, -math.pi / 8.0, 0.1)

    for ball_start, ball_end, drobot, dball in zip(
        ball_starts, ball_ends, durations_robot, durations_ball
    ):
        _play_contact(
            robot2_joints, joints_start, joints_end, drobot, ball_start, ball_end, dball
        )


if __name__ == "__main__":
    run()
