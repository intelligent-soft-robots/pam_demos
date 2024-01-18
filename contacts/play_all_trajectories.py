import math
import time
import o80
import context
from handle_contacts import MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID
from handle_contacts import get_handle

# set the trajectories you want to play.
# empty or none: play all trajectories
FILTER = [20, 35, 44]

# set to True for slow-motion
SLOWMO = False


def run():
    def play_trajectory(handle, ball, trajectory):
        # to be able to fly to the racket on
        # the way to the init position
        handle.deactivate_contact(BALL_SEGMENT_ID)

        # for iterating over the trajectory
        iterator = context.BallTrajectories.iterate(trajectory)

        # ball going to launch position
        duration, state = next(iterator)
        ball.add_command(
            state.get_position(),
            [0] * 3,
            o80.Duration_us.milliseconds(400),
            o80.Mode.OVERWRITE,
        )
        ball.pulse_and_wait()
        # reset and reactivate the contact
        handle.reset_contact(BALL_SEGMENT_ID)
        handle.activate_contact(BALL_SEGMENT_ID)
        for duration, state in iterator:
            print("\t", duration, state)
            ball.add_command(
                state.get_position(),
                state.get_velocity(),
                o80.Duration_us.microseconds(duration),
                o80.Mode.QUEUE,
            )

        # playing commands
        if SLOWMO:
            # in slow motion
            ball.pulse()
            total_time = len(trajectory) * duration * 0.001
            sleep = 0.01
            nb_iter = int((total_time / sleep) + 0.5)
            for iter in range(nb_iter):
                time.sleep(sleep)
                handle.pause(True)
                time.sleep(0.1)
                handle.pause(False)
        else:
            # in "normal" time
            ball.pulse_and_wait()

    handle = get_handle(True, False, False)
    ball = handle.frontends[BALL_SEGMENT_ID]
    robot = handle.frontends[ROBOT_SEGMENT_ID]

    # target position of robot
    joints = (0, +math.pi / 3.0, +math.pi / 4.0, +math.pi / 4.0)
    joint_velocities = (0, 0, 0, 0)
    robot.add_command(
        joints, joint_velocities, o80.Duration_us.seconds(1), o80.Mode.QUEUE
    )
    robot.pulse_and_wait()

    trajectories_generator = context.BallTrajectories("originals")
    trajectories = trajectories_generator.get_all_trajectories()

    for index, trajectory in trajectories.items():
        if (not FILTER) or (index in FILTER):
            print("\t", index)
            play_trajectory(handle, ball, trajectory)


if __name__ == "__main__":
    run()
