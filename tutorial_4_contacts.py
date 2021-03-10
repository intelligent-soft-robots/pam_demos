import math
import time
import o80
import o80_pam
import pam_mujoco
import context

ball = o80_pam.BallFrontEnd("ball2")

def _distance(p1,p2):
    return math.sqrt([(a-b)**2 for a,b in zip(p1,p2)])

def _velocity(p1,p2,duration):
    return [(a-b)/duration for a,b in zip(p1,p2)]

def _go_back_to_start(ball,start):
    # going to the start point while avoiding touching the
    # table, to avoid messing up with the contact
    fp = ball.latest().get_position()
    # going to the right side
    fp[0]=-0.5
    ball.add_command(fp,[0,0,0],o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    # going up
    fp = ball.latest().get_position()
    fp[2]=start[2]
    ball.add_command(fp,[0,0,0],o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    # going to start position
    ball.add_command(start,[0,0,0],o80.Duration_us.milliseconds(400),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    
def _dropping(ball,start,end,duration):
    # starting with clean contact
    pam_mujoco.reset_contact("table")
    # going back to start while avoiding the table,
    # then moving to end, and printing if a contact occured
    _go_back_to_start(ball,start)
    # starting with new contact
    velocity = _velocity(start,end,duration)
    ball.add_command(end,velocity,o80.Duration_us.seconds(duration),o80.Mode.QUEUE)
    ball.pulse_and_wait()


    
# dropping vertically
duration = 1 
start = (0.5,1,0.5)
#end = (0.5,1,-1)
#_dropping(ball,start,end,duration)



# dropping left to right
end = (1.0,1.5,-1)
_dropping(ball,start,end,duration)
