import math
import time
import context
import o80
import o80_pam
import pam_mujoco

mujoco_id = "balls_demo"

# only accepted number of balls !
#nb_balls = 3
#nb_balls = 10
nb_balls = 20
## Boost error ! shared memory size issue ?
#nb_balls = 50
#nb_balls = 100


balls = pam_mujoco.MujocoItems("balls")

for index in range(nb_balls):
    ball = pam_mujoco.MujocoItem("ball_"+str(index),
                                 control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
                                 contact_type=pam_mujoco.ContactTypes.table)
    balls.add_ball(ball)

graphics=True
accelerated_time=False

handle = pam_mujoco.MujocoHandle(mujoco_id,
                                 table=True,
                                 combined=balls,
                                 graphics=graphics,
                                 accelerated_time=accelerated_time)

frontend = handle.frontends["balls"]

item3d = o80.Item3dState()
item3d.set_velocity([0]*3)
step = 0.0
duration = o80.Duration_us.milliseconds(500)

for index_ball in range(nb_balls):
    item3d.set_position([step,step,0])
    frontend.add_command(index_ball,item3d,duration,o80.Mode.QUEUE)
    step+=0.1
for index_ball in range(nb_balls):
    item3d.set_position([step,step,0])
    frontend.add_command(index_ball,item3d,duration,o80.Mode.QUEUE)
    step-=0.1
frontend.pulse_and_wait()

trajectories_generator = context.BallTrajectories()
sampling_rate_ms = trajectories_generator.get_sampling_rate_ms()
duration = o80.Duration_us.milliseconds(int(sampling_rate_ms))

for index_ball in range(nb_balls):
    _,trajectory = trajectories_generator.random_trajectory()
    item3d.set_position(trajectory[0].position)
    item3d.set_velocity([0]*3)
    frontend.add_command(index_ball,item3d,o80.Duration_us.seconds(1),o80.Mode.QUEUE)
    for item in trajectory[1:]:
        item3d.set_position(item.position)
        item3d.set_velocity(item.velocity)
        frontend.add_command(index_ball,item3d,duration,o80.Mode.QUEUE)

frontend.pulse()
        
