import math
import time
import o80
import o80_pam
import pam_mujoco
import context

ball = o80_pam.BallFrontEnd("ball")

def _distance(p1,p2):
    return math.sqrt([(a-b)**2 for a,b in zip(p1,p2)])

def _velocity(p1,p2,duration):
    return [(a-b)/duration for a,b in zip(p2,p1)]

def _dropping(ball,start,end,duration):
    # deativating contacts
    pam_mujoco.deactivate_contact("table")
    # going to start position
    ball.add_command(start,[0,0,0],o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    # reativating contacts
    pam_mujoco.activate_contact("table")
    # starting with clean contact
    pam_mujoco.reset_contact("table")
    # going to end point
    velocity = _velocity(start,end,duration)
    ball.add_command(end,velocity,o80.Duration_us.seconds(duration),o80.Mode.QUEUE)
    ball.pulse_and_wait()

# dropping back to front
duration = 1 
start = (1.0,1.5,0.5)
end = (1.1,0.5,-1.0)
_dropping(ball,start,end,duration)
