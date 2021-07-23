import math,time
import o80,context
from handle_contacts import MUJOCO_ID,ROBOT_SEGMENT_ID,BALL_SEGMENT_ID
from handle_contacts import get_robot_contact_handle

# set the trajectories you want to play.
# empty or none: play all trajectories
FILTER=[4,35]

def play_trajectory(handle,ball,trajectory,duration):
    # to be able to fly to the racket on
    # the way to the init position
    handle.deactivate_contact(BALL_SEGMENT_ID)
    # ball going to launch position
    ball.add_command(trajectory[0].position,
                     [0]*3,
                     o80.Duration_us.milliseconds(400),
                     o80.Mode.OVERWRITE)
    ball.pulse_and_wait()
    # reset and reactivate the contact
    handle.reset_contact(BALL_SEGMENT_ID)
    handle.activate_contact(BALL_SEGMENT_ID)
    # loading full ball trajectory
    for point in trajectory[1:]:
        ball.add_command(point.position,
                         point.velocity,
                         duration,o80.Mode.QUEUE)
    # playing commands
    ball.pulse_and_wait()
    
handle = get_robot_contact_handle()
ball = handle.frontends[BALL_SEGMENT_ID]
robot = handle.frontends[ROBOT_SEGMENT_ID]

# target position of robot
joints = (0,+math.pi/3.0,
          -math.pi/4.0,+math.pi/4.0)
joint_velocities = (0,0,0,0)
robot.add_command(joints,joint_velocities,o80.Duration_us.seconds(1),o80.Mode.QUEUE)
robot.pulse_and_wait()

trajectories_generator = context.BallTrajectories()
trajectories = trajectories_generator.get_all_trajectories()
sampling_rate_ms = trajectories_generator.get_sampling_rate_ms()
duration = o80.Duration_us.milliseconds(int(sampling_rate_ms))

for index,trajectory in enumerate(trajectories):
    if ( (not FILTER ) or (index in FILTER) ):
        print("playing: ",index," | ",trajectories_generator.get_file_name(index))
        play_trajectory(handle,ball,trajectory,duration)
    
