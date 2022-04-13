"""
Plays some pre-recorded ball trajectories
"""

import math
import time
import numpy as np
import context
import o80
import o80_pam
import pam_mujoco

MUJOCO_ID = "balls"

# only accepted number of balls !
# NB_BALLS = 3
# NB_BALLS = 10
NB_BALLS = 20
## Boost error ! shared memory size issue ?
# NB_BALLS = 50
# NB_BALLS = 100

def run():

    # configuring pam_mujoco. Assuming "pam_mujoco balls"
    # has been called in another terminal.
    # creating ball items
    balls = pam_mujoco.MujocoItems("extra_balls")
    # the segment ids of the balls
    ball_segment_ids=["ball_"+str(index) for index in range(NB_BALLS)]
    # adding balls one by one. Monitoring contact with the table.
    for index,segment_id in enumerate(ball_segment_ids):
        ball = pam_mujoco.MujocoItem(
            segment_id,
            control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
            contact_type=pam_mujoco.ContactTypes.table,
        )
        balls.add_ball(ball)
    # adding the table tennis table
    table = pam_mujoco.MujocoTable(
        "table"
    )
    # configuring the robot, with joint control
    robot = pam_mujoco.MujocoRobot(
        pam_mujoco.RobotType.PAMY1,
        "robot",
        control=pam_mujoco.MujocoRobot.JOINT_CONTROL,
    )
    graphics = True
    accelerated_time = False
    handle = pam_mujoco.MujocoHandle(
        MUJOCO_ID,
        table=table,
        robot1=robot,
        combined=balls,
        graphics=graphics,
        accelerated_time=accelerated_time,
    )

    # frontend to the ball items (same segment_id:"extra_balls")
    frontend = handle.frontends["extra_balls"]

    # disabling contact between the balls and the table,
    # so that the balls do not collide while going to
    # initial position
    for segment_id in ball_segment_ids:
        handle.deactivate_contact(segment_id)

    # each instance of Item3dState
    # encapsulates the position and velocity
    # of a ball as state/desired state
    item3d = o80.Item3dState()
    item3d.set_velocity([0] * 3)
    step = 0.0
    duration = o80.Duration_us.milliseconds(500)

    # having the balls performing a little dance
    for index_ball in range(NB_BALLS):
        item3d.set_position([step, step, 2.0])
        frontend.add_command(index_ball, item3d, duration, o80.Mode.QUEUE)
        step += 0.1
    for index_ball in range(NB_BALLS):
        item3d.set_position([step, step, 2.0])
        frontend.add_command(index_ball, item3d, duration, o80.Mode.QUEUE)
        step -= 0.1
    frontend.pulse_and_wait()

    # enabling balls/table contact
    for segment_id in ball_segment_ids:
        handle.reset_contact(segment_id)
        handle.activate_contact(segment_id)

    # trajectory of balls going through the tables
    observation = frontend.latest().get_observed_states()
    positions = [observation.values[index_ball].get_position() for index_ball in range(NB_BALLS)]
    target = [0.5,1,0.5]
    velocity = 5. # m per second
    trajectories = [
        context.ball_trajectories.velocity_line_trajectory(position,target,velocity)
        for position in positions
    ]
    for index_ball, trajectory in enumerate(trajectories):
        durations,positions_,velocities = trajectory
        for d,p,v in zip(durations,positions_,velocities):
            item3d.set_position(p)
            item3d.set_velocity(v)
            frontend.add_command(index_ball,item3d,o80.Duration_us.microseconds(d),o80.Mode.QUEUE)
    frontend.pulse_and_wait()


if __name__ == "__main__":
    run()
