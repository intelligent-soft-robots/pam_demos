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

def _play_contact(joints_start,joints_end,duration_racket,
                  ball_start,ball_end,duration_ball):

    # deativating contacts
    pam_mujoco.deactivate_contact("robot_contact")

    # going to start position
    ball.add_command(ball_start,[0,0,0],o80.Duration_us.milliseconds(200),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    robot.add_command(joints_start,(0,0,0,0),
                      o80.Duration_us.milliseconds(200),o80.Mode.QUEUE)
    robot.pulse_and_wait()
    
    # reativating contacts
    pam_mujoco.activate_contact("robot_contact")
    
    # starting with clean contact
    pam_mujoco.reset_contact("robot_contact")

    # going to end positions
    ball.add_command(ball_end,_velocity(ball_start,ball_end,duration_ball*1e-3),
                     o80.Duration_us.milliseconds(duration_ball),o80.Mode.QUEUE)
    robot.add_command(joints_end,(0,0,0,0),
                      o80.Duration_us.milliseconds(duration_racket),o80.Mode.QUEUE)

    
    ball.pulse()
    robot.pulse()

    time.sleep(max(duration_racket*1e-3,duration_ball*1e-3))



    
joints_start = (-math.pi/4.0,+math.pi/4.0,
                -math.pi/8.0,+math.pi/4.0)
joints_end = (+math.pi/4.0,+math.pi/4.0,
                -math.pi/8.0,+math.pi/4.0)
ball_start = (0.9,1.5,0.25)
ball_end = (0.95,-1.0,0.2)
_play_contact(joints_start,joints_end,500,
              ball_start,ball_end,500)
