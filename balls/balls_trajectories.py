"""
Plays pre-recorded ball trajectories
"""


import math
import time
import context
import o80
import o80_pam
import pam_mujoco

MUJOCO_ID = "balls_trajectories"

# only accepted number of balls !
# NB_BALLS = 3
# NB_BALLS = 10
NB_BALLS = 20
## Boost error ! shared memory size issue ?
# NB_BALLS = 50
# NB_BALLS = 100


def run():

    # configuring pam_mujoco.
    # It is assumed here that "pam_mujoco ball_trajectories" has
    # been called in another terminal.
    # -----------------------------------------------------------
    # A joint controlled robot
    robot = pam_mujoco.MujocoRobot(
        False, "robot", control=pam_mujoco.MujocoRobot.JOINT_CONTROL
    )
    # A few balls, setting contact with robot's racket
    balls = pam_mujoco.MujocoItems("balls")
    ball_segment_ids = ["ball_" + str(index) for index in range(NB_BALLS)]
    for index in range(NB_BALLS):
        ball = pam_mujoco.MujocoItem(
            ball_segment_ids[index],
            control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
            contact_type=pam_mujoco.ContactTypes.racket1,
        )
        balls.add_ball(ball)
    graphics = True
    accelerated_time = False
    # The handle to all o80 frontends
    handle = pam_mujoco.MujocoHandle(
        MUJOCO_ID,
        table=pam_mujoco.MujocoTable("table"),
        robot1=robot,
        combined=balls,
        graphics=graphics,
        accelerated_time=accelerated_time,
    )

    # for control of the robot
    robot = handle.frontends["robot"]
    # for control of the balls
    balls = handle.frontends["balls"]

    # Reads pre-recorded ball trajectories from either /opt/mpi-is/pam/context/ball_trajectories.hdf5
    # or ~/.mpi-is/pam/context/ball_trajectories.hdf5
    trajectories_generator = context.BallTrajectories(group="originals")

    for iteration in range(5):

        print("\niteration:", iteration)

        # reseting the contacts. Otherwise the balls that had a contact
        # with the robot during one of the previous iteration will not
        # be controllable.
        for ball_index in range(NB_BALLS):
            handle.reset_contact(ball_segment_ids[ball_index])

        # moving the balls to a starting position
        for index_ball in range(NB_BALLS):
            ball_start = o80.Item3dState([0.1 * index_ball, 1, 2], [0, 0, 0])
            balls.add_command(
                index_ball,
                ball_start,
                o80.Duration_us.milliseconds(400),
                o80.Mode.OVERWRITE,
            )
        balls.pulse_and_wait()

        # loading pre-recorded trajectories
        for index_ball in range(NB_BALLS):
            _, trajectory = trajectories_generator.random_trajectory()
            stamped_trajectory = trajectories_generator.random_trajectory()
            iterator = trajectories_generator.iterate(stamped_trajectory)
            # for going to first point
            _, state = next(iterator)
            balls.add_command(
                index_ball, state, o80.Duration_us.milliseconds(400), o80.Mode.QUEUE
            )
            # loading the rest of the trajectory
            for duration, state in iterator:
                balls.add_command(
                    index_ball,
                    state,
                    o80.Duration_us.microseconds(duration),
                    o80.Mode.QUEUE,
                )

        # setting the target position of robot
        joints = (0, +math.pi / 3.0, +math.pi / 4.0, +math.pi / 4.0)
        joint_velocities = (0, 0, 0, 0)
        robot.add_command(
            joints, joint_velocities, o80.Duration_us.milliseconds(500), o80.Mode.QUEUE
        )

        # sending commands to backends
        balls.pulse()
        robot.pulse()

        # monitoring for contacts and ball position.
        # stopping when one of the balls is below 0.8
        contact_detected = [False for _ in range(NB_BALLS)]
        while True:
            # printing contacts
            for ball_index in range(NB_BALLS):
                contact = handle.get_contact(ball_segment_ids[ball_index])
                if contact.contact_occured and not contact_detected[ball_index]:
                    contact_detected[ball_index] = True
                    print(
                        "contact ball {} with racket at {}".format(
                            ball_index, contact.position
                        )
                    )
            # checking positions to see if we should exit
            stop = False
            observation = balls.latest()
            states = observation.get_observed_states()
            for ball_index in range(NB_BALLS):
                position = states.get(ball_index).get_position()
                if position[2] < 0.8:
                    stop = True
            if stop:
                break
            time.sleep(0.5)

        # reset simulation to starting state
        handle.reset()

    print()


if __name__ == "__main__":
    run()
