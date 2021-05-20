import math
import time
import context
import o80
import o80_pam
import pam_mujoco

# how to run

# in a terminal start: pam_mujoco demo-simulation
# then: pam_mujoco_visualization demo-simulation demo-visualization 


# creating the mujoco's configuration, and getting the handle
robot = pam_mujoco.MujocoRobot("robot",
                               control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
ball = pam_mujoco.MujocoItem("ball",
                             control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
                             contact_type=pam_mujoco.ContactTypes.table)
hit_point = pam_mujoco.MujocoItem("hit_point",
                                  control=pam_mujoco.MujocoItem.CONSTANT_CONTROL)
goal = pam_mujoco.MujocoItem("goal",
                             control=pam_mujoco.MujocoItem.CONSTANT_CONTROL)
handle = pam_mujoco.MujocoHandle("demo-simulation",
                                 graphics=False,
                                 table=True,
                                 robot1=robot,
                                 balls=(ball,),
                                 goals=(goal,),
                                 hit_points=(hit_point,))

# getting the frontend connected to the robot's pressure controller 
robot = handle.frontends["robot"]
ball = handle.frontends["ball"]
hit_point = handle.frontends["hit_point"]

# moving the balls and the robot to a target position

print("starting state ball: {}".format(ball.latest()))


# target position of balls
position1 = (0.5,3,1)
position2 = (1.5,3,1)
velocity = (0,0,0)
# target position of robot
joints = (math.pi/4.0,-math.pi/4.0,
          math.pi/4.0,-math.pi/4.0)
joint_velocities = (0,0,0,0)

duration = o80.Duration_us.seconds(2)

ball.add_command(position1,velocity,duration,o80.Mode.QUEUE)
robot.add_command(joints,joint_velocities,duration,o80.Mode.QUEUE)

ball.pulse()
robot.pulse()


print("reached state ball: {}".format(ball.latest()))
print("reached state robot: {}".format(robot.latest()))

time.sleep(3)


# reading pre-recorded trajectories


index,trajectory = list(context.BallTrajectories().random_trajectory())

print("ball will play trajectory {}".format(context.BallTrajectories().get_file_name(index)))

# moving ball 1 and ball 2 to first trajectories point
position = trajectory[0].position 
velocity = trajectory[0].velocity 
duration = o80.Duration_us.seconds(2)
ball.add_command(position,velocity,duration,o80.Mode.QUEUE)
ball.pulse()

time.sleep(4)


# loading and playing the trajectories
duration_s = 0.01
duration = o80.Duration_us.milliseconds(int(duration_s*1000))
total_duration = duration_s * len(trajectory)
for traj_point in trajectory:
    ball.add_command(traj_point.position,traj_point.velocity,
                     duration,o80.Mode.QUEUE)
    ball.pulse()


# monitoring for contact
# note: contact will not happen at all the runs of this executable,
# as the virtual table heights does not match exactly the height of the
# real table used for the recording of the ball trajectory.

print("\nmonitoring for contact ...")
time_start = time.time()
contact = None

while time.time()-time_start < 5:

    # monitoring contact
    contact = handle.get_contact("ball")
    if contact.contact_occured:
        break

    # having the hit point following the ball
    position = ball.latest().get_position()
    hit_point.add_command(position,[0,0,0],o80.Mode.OVERWRITE)
    hit_point.pulse()

if not contact.contact_occured:
    print("no contact between ball and table! "
          "(minimal distance: {})".format(contact.minimal_distance))
else:
    print("contact occured\n"
          "\tposition: {}\n"
          "\ttime stamp: {}".format(contact.position,contact.time_stamp))


print()

