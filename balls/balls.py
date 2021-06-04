import math
import time
import context
import o80
import o80_pam
import pam_mujoco

mujoco_id = "balls"

# only accepted number of balls !
#nb_balls = 3
#nb_balls = 10
nb_balls = 20
#nb_balls = 50
#nb_balls = 100


balls = pam_mujoco.MujocoItems("balls")

for index in range(nb_balls):
    ball = pam_mujoco.MujocoItem("ball_"+str(index),
                                 control=pam_mujoco.MujocoItem.COMMAND_ACTIVE_CONTROL)
    balls.add_ball(ball)

graphics=True
accelerated_time=False
handle = pam_mujoco.MujocoHandle(mujoco_id,
                                 table=True,
                                 combined=balls,
                                 graphics=graphics,
                                 accelerated_time=accelerated_time)
