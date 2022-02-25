import math
import time
import context
import o80
import o80_pam
import pam_mujoco

mujoco_id = "balls"

# only accepted number of balls !
# nb_balls = 3
# nb_balls = 10
nb_balls = 20
## Boost error ! shared memory size issue ?
# nb_balls = 50
# nb_balls = 100

# creating ball items
balls = pam_mujoco.MujocoItems("extra_balls")
# the segment ids of the balls
ball_segment_ids=["ball_"+str(index) for index in range(nb_balls)]
# adding balls one by one
for index,segment_id in enumerate(ball_segment_ids):
    ball = pam_mujoco.MujocoItem(
        segment_id,
        control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
        contact_type=pam_mujoco.ContactTypes.table,
    )
    balls.add_ball(ball)

table = pam_mujoco.MujocoTable(
    "table",
    position=(0.1, 0, -0.44),
    orientation="-1 0 0 0 -1 0",
)

# configuring the robot
robot = pam_mujoco.MujocoRobot(
    False,
    "robot",
    position=(0.435, 0.1175, -0.0025),
    orientation="-1 0 0 0 -1 0",
    control=pam_mujoco.MujocoRobot.JOINT_CONTROL,
)

graphics = True
accelerated_time = False

handle = pam_mujoco.MujocoHandle(
    mujoco_id,
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
for index_ball in range(nb_balls):
    item3d.set_position([step, step, 0])
    frontend.add_command(index_ball, item3d, duration, o80.Mode.QUEUE)
    step += 0.1
for index_ball in range(nb_balls):
    item3d.set_position([step, step, 0])
    frontend.add_command(index_ball, item3d, duration, o80.Mode.QUEUE)
    step -= 0.1
frontend.pulse_and_wait()

# trajectories_generator encapsulate recorded ball
# trajectories
trajectories_generator = context.BallTrajectories()
sampling_rate_ms = trajectories_generator.get_sampling_rate_ms()
duration = o80.Duration_us.milliseconds(int(sampling_rate_ms))

# enabling balls/table contact
for segment_id in ball_segment_ids:
    handle.reset_contact(segment_id)
    handle.activate_contact(segment_id)

# balls playing pre-recorded trajectories 
for index_ball in range(nb_balls):
    _, trajectory = trajectories_generator.random_trajectory()
    item3d.set_position(trajectory[0].position)
    item3d.set_velocity([0] * 3)
    frontend.add_command(index_ball, item3d, o80.Duration_us.seconds(1), o80.Mode.QUEUE)
    for item in trajectory[1:]:
        item3d.set_position(item.position)
        item3d.set_velocity(item.velocity)
        frontend.add_command(index_ball, item3d, duration, o80.Mode.QUEUE)

frontend.pulse()
