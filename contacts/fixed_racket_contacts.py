import math,time
import o80
from handle_contacts import MUJOCO_ID,ROBOT_SEGMENT_ID,BALL_SEGMENT_ID
from handle_contacts import get_robot_contact_handle


handle = get_robot_contact_handle()

ball = handle.frontends[BALL_SEGMENT_ID]
robot = handle.frontends[ROBOT_SEGMENT_ID]

handle.reset_contact(BALL_SEGMENT_ID)


def _velocity(p1,p2,duration_ms):
    return [(a-b)/(float(duration_ms)/1000.0) for a,b in zip(p2,p1)]


def _play_contact(joints,start,end,duration=1000):

    global handle,ball,robot
    
    # deativating contacts
    handle.deactivate_contact(BALL_SEGMENT_ID)

    # going to start position
    ball.add_command(start,[0,0,0],o80.Duration_us.milliseconds(200),o80.Mode.QUEUE)
    ball.pulse_and_wait()
    
    # reativating contacts
    handle.activate_contact(BALL_SEGMENT_ID)
    
    # starting with clean contact
    handle.reset_contact(BALL_SEGMENT_ID)

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
                     o80.Duration_us.milliseconds(duration),o80.Mode.QUEUE)
    ball.pulse_and_wait()

    
# target position of robot
joints = (-math.pi/4.0,+math.pi/4.0,
          -math.pi/4.0,+math.pi/4.0)
start = (0.78,1.5,0.5)
end = (0.78,-1.0,0.2)
_play_contact(joints,start,end,1000)
end = (0.78,-1.0,0.15)
_play_contact(joints,start,end,1000)
end = (0.82,-1.0,0.2)
_play_contact(joints,start,end,1000)
start = (0.78,1.5,0.0)
end = (0.78,-1.0,0.3)
_play_contact(joints,start,end,1000)
end = (0.78,-1.0,0.35)
_play_contact(joints,start,end,1000)
end = (0.82,-1.0,0.3)
_play_contact(joints,start,end,1000)
start = (1.5,1.5,0.0)
end = (0.3,-1.0,0.3)
_play_contact(joints,start,end,1000)
start = (-0.5,1.5,0.0)
end = (1.5,-1.0,0.3)
_play_contact(joints,start,end,1000)
