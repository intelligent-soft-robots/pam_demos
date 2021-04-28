import math
import time
import o80
import o80_pam
import pam_mujoco

mujoco_id = "tutorial_mujoco_id"
robot = o80_pam.JointFrontEnd("robot")
ball = o80_pam.BallFrontEnd("ball")

joints1 = (math.pi/4.0,-math.pi/4.0,
           math.pi/4.0,-math.pi/4.0)
joints2 = [0]*4
joint_velocities = [0]*4
position1 = [1]*3
position2 = [0]*3
velocity = [0]*3

duration = o80.Duration_us.seconds(4)
robot.add_command(joints1,joint_velocities,duration,o80.Mode.OVERWRITE)
ball.add_command(position1,velocity,duration,o80.Mode.OVERWRITE)
print("go !")
robot.pulse()
ball.pulse()

time.sleep(2)

duration = o80.Duration_us.milliseconds(50)
robot.add_command(joints2,joint_velocities,duration,o80.Mode.OVERWRITE)
ball.add_command(position2,velocity,duration,o80.Mode.OVERWRITE)
print("go !")
robot.pulse()
ball.pulse()


pam_mujoco.request_reset(mujoco_id)


def commented():

    def reset(mujoco_id,ball,robot):
        # requesting the mujoco simulation
        # to reset
        #pam_mujoco.request_reset(mujoco_id)
        # o80 backends controllers send 
        # a command requested the next desired states
        # to be the first observed states
        #ball.add_reinit_command()
        #robot.add_reinit_command()
        ball_position = (0,0,0)
        velocity = (0,0,0)
        joints = (0,0,0,0)
        joint_velocities = (0,0,0,0)
        #ball.add_command(ball_position,velocity,o80.Mode.OVERWRITE)
        robot.add_command(joints,joint_velocities,o80.Mode.OVERWRITE)
        #ball.pulse()
        robot.pulse()
        time.sleep(1)

    mujoco_id = "tutorial_mujoco_id"
    #ball = o80_pam.BallFrontEnd("ball")
    ball=None
    robot = o80_pam.JointFrontEnd("robot")

    # target position for ball
    ball_position = (1.5,1.5,1.5)
    velocity = (0,0,0)

    # target position for robot
    joints = (math.pi/4.0,-math.pi/4.0,
              math.pi/4.0,-math.pi/4.0)
    joint_velocities = (0,0,0,0)

    # will try go there over 4 seconds
    duration = o80.Duration_us.seconds(4)
    #ball.add_command(ball_position,velocity,duration,o80.Mode.OVERWRITE)
    robot.add_command(joints,joint_velocities,duration,o80.Mode.OVERWRITE)
    print("go !")
    #ball.pulse()
    robot.pulse()

    # but resetting after 2 seconds
    time.sleep(2)
    print("reset !")
    reset(mujoco_id,ball,robot)

    # again
    duration = o80.Duration_us.seconds(10)
    #ball.add_command(ball_position,velocity,duration,o80.Mode.OVERWRITE)
    robot.add_command(joints,joint_velocities,duration,o80.Mode.OVERWRITE)
    print("go !")
    #ball.pulse()
    robot.pulse()

    # but resetting after 4 seconds
    time.sleep(4)
    print("reset !")
    reset(mujoco_id,ball,robot)



