import math
import time
import o80
import o80_pam
import pam_mujoco
import context

ball = o80_pam.BallFrontEnd("ball")
robot = o80_pam.JointFrontEnd("robot")

pam_mujoco.reset_contact("robot_contact")

def _velocity(p1,p2,duration):
    return [(a-b)/duration for a,b in zip(p2,p1)]

def _play_contact(joints,start,end,duration=1):

    # deativating contacts
    pam_mujoco.deactivate_contact("robot_contact")

    # going to start position
    ball.add_command(start,[0,0,0],o80.Duration_us.milliseconds(200),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    
    # reativating contacts
    pam_mujoco.activate_contact("robot_contact")
    
    # starting with clean contact
    pam_mujoco.reset_contact("robot_contact")

    # set the robot to position
    robot.add_command(joints,(0,0,0,0),
                      o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
    robot.pulse_and_wait()

    # set the ball to start position
    ball.add_command(start,[0,0,0],
                     o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
    ball.pulse_and_wait()

    # having the ball going to the end position
    ball.add_command(end,_velocity(start,end,duration),
                     o80.Duration_us.seconds(duration),o80.Mode.QUEUE)
    ball.pulse_and_wait()



    
# target position of robot
joints = (-math.pi/4.0,+math.pi/4.0,
          -math.pi/4.0,+math.pi/4.0)
start = (0.78,1.5,0.5)
end = (0.78,-1.0,0.2)
_play_contact(joints,start,end,1)
end = (0.78,-1.0,0.15)
_play_contact(joints,start,end,1)
end = (0.82,-1.0,0.2)
_play_contact(joints,start,end,1)
start = (0.78,1.5,0.0)
end = (0.78,-1.0,0.3)
_play_contact(joints,start,end,1)
end = (0.78,-1.0,0.35)
_play_contact(joints,start,end,1)
end = (0.82,-1.0,0.3)
_play_contact(joints,start,end,1)

