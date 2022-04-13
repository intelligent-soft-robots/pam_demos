import matplotlib.pyplot as plt
import math
import time
import o80
import pam_mujoco

mujoco_id = "fk"
robot_segment_id = "robot_fk"
ball_segment_id = "ball_fk"

graphics = True
accelerated_time = False
burst_mode = False

ball = pam_mujoco.MujocoItem(
    ball_segment_id,
    control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
    #contact_type=pam_mujoco.ContactTypes.racket1,
)

robot_control = pam_mujoco.MujocoRobot(
    pam_mujoco.RobotType.PAMY2,
    robot_segment_id,
    position = [0.]*3,
    control=pam_mujoco.MujocoRobot.JOINT_CONTROL
)

handle = pam_mujoco.MujocoHandle(
    mujoco_id,
    graphics=graphics,
    accelerated_time=accelerated_time,
    burst_mode=burst_mode,
    balls=(ball,),
    robot1=robot_control,
)

#handle.reset_contact(ball_segment_id)

robot = handle.frontends[robot_segment_id]
ball = handle.frontends[ball_segment_id]

robot_positions = ( [0.]*4,
                    [math.pi/4.,0,math.pi/4.,0.],
                    [math.pi/2.,0,math.pi/4.,0.],
                    [-math.pi/4.,0,math.pi/4.,0.],
                    [-math.pi/2.,0,math.pi/4.,0.] )

for robot_position in robot_positions:
    
    robot.add_command(robot_position,[0.]*4,o80.Duration_us.seconds(1),o80.Mode.OVERWRITE)
    robot.pulse_and_wait()

    cartesian_robot = robot.latest().get_cartesian_position()

    ball.add_command(cartesian_robot,[0.]*3,o80.Duration_us.seconds(1),o80.Mode.QUEUE)
    ball.pulse_and_wait()

    time.sleep(2)

