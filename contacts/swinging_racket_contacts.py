import math, time
import o80
from handle_contacts import MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID
from handle_contacts import get_robot_contact_handle


def run():

    handle = get_robot_contact_handle()

    ball = handle.frontends[BALL_SEGMENT_ID]
    robot = handle.frontends[ROBOT_SEGMENT_ID]

    handle.reset_contact(BALL_SEGMENT_ID)


    def _velocity(p1, p2, duration):
        return [(a - b) / duration for a, b in zip(p2, p1)]


    def _play_contact(
        joints_start, joints_end, duration_racket, ball_start, ball_end, duration_ball
    ):

        # deativating contacts
        handle.deactivate_contact(BALL_SEGMENT_ID)

        # robot going to start position
        robot.add_command(
            joints_start, (0, 0, 0, 0), o80.Duration_us.milliseconds(200), o80.Mode.QUEUE
        )
        robot.pulse_and_wait()

        # ball_start and ball_end are relative to the racket position
        cartesian_robot = robot.latest().get_cartesian_position()
        ball_start = [s+cr for s,cr in zip(ball_start,cartesian_robot)]
        ball_end = [e+cr for e,cr in zip(ball_end,cartesian_robot)]
        
        # ball going to start position
        ball.add_command(
            ball_start, [0, 0, 0], o80.Duration_us.milliseconds(200), o80.Mode.QUEUE
        )
        ball.pulse_and_wait()

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


    ball_starts = [ (+0.25,+2.00,+0.00) , (-0.24,+2.00,+0.00) , (+0.66,+2.00,+0.00)   ]
    ball_ends =   [ (+0.25,-1.00,+0.00) , (+0.66,-1.00,-0.00) , (-0.24,-1.00,-0.00)]

    durations_robot = [ 500 , 500 , 500]
    durations_ball = [ 500 , 800 , 600]
    
    joints_start = (-math.pi / 3.0, +math.pi / 2.5, -math.pi / 8.0, 0.1)
    joints_end = (+math.pi / 3.0, +math.pi / 2.5, -math.pi / 8.0, 0.1)

    
    for ball_start, ball_end, drobot, dball in zip(ball_starts, ball_ends, durations_robot, durations_ball):
        _play_contact(joints_start, joints_end, drobot, ball_start, ball_end, dball)


if __name__ == "__main__":

    run()
