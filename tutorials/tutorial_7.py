import math
import time
import o80
import o80_pam
import pam_mujoco
import context


# to run this tutorial, start pam_mujoco with mujoco_id "tutorial_7"

# creating the mujoco's configuration, and getting the handle
robot = pam_mujoco.MujocoRobot("robot", control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
ball = pam_mujoco.MujocoItem("ball", control=pam_mujoco.MujocoItem.CONSTANT_CONTROL)
handle = pam_mujoco.MujocoHandle(
    "tutorial_7", burst_mode=True, accelerated_time=True, robot1=robot, balls=(ball,)
)


ball = handle.frontends["ball"]
robot = handle.frontends["robot"]


# target position of ball
position1 = (0.5, 3, 1)
position2 = (1.5, 3, 1)
velocity = (0, 0, 0)

# target position of robot
joints = (math.pi / 4.0, -math.pi / 4.0, math.pi / 4.0, -math.pi / 4.0)
joint_velocities = (0, 0, 0, 0)

nb_iterations = 3000
current_iteration = robot.latest().get_iteration()
iteration = o80.Iteration(current_iteration + nb_iterations)

ball.add_command(position1, velocity, iteration, o80.Mode.QUEUE)
robot.add_command(joints, joint_velocities, iteration, o80.Mode.QUEUE)

ball.pulse()
robot.pulse()

per_step = 500
nb_steps = int(nb_iterations / per_step)

for _ in range(nb_steps):
    print("step !")
    handle.burst(per_step)
    time.sleep(2)

time.sleep(4)
print("mujoco exit !")
handle.mujoco_exit()
