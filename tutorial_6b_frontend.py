import math
import time
import o80
import o80_pam
import pam_mujoco
import context

ball = o80_pam.BallFrontEnd("ball")
robot = o80_pam.JointFrontEnd("robot")


# target position of ball
position1 = (0.5,3,1)
position2 = (1.5,3,1)
velocity = (0,0,0)

# target position of robot
joints = (math.pi/4.0,-math.pi/4.0,
          math.pi/4.0,-math.pi/4.0)
joint_velocities = (0,0,0,0)

nb_iterations = 3000
current_iteration = robot.latest().get_iteration()
iteration = o80.Iteration(current_iteration+nb_iterations)

ball.add_command(position1,velocity,iteration,o80.Mode.QUEUE)
robot.add_command(joints,joint_velocities,iteration,o80.Mode.QUEUE)

ball.pulse()
robot.pulse()

per_step = 500
nb_steps = int(nb_iterations/per_step)

for _ in range(nb_steps):
    print("step !")
    robot.burst(per_step)
    time.sleep(2)
