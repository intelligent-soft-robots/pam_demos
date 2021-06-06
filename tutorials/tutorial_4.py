import math
import time
import context
import o80
import o80_pam
import pam_mujoco

# to run this tutorial, start pam_mujoco with mujoco_id "tutorial_4"

# creating the mujoco's configuration, and getting the handle.
# contrary to tutorial 1 to 3, the robot will be joint controlled (i.e.
# position and velocity).
# also other items (balls, hit point) are added and controlled.
robot = pam_mujoco.MujocoRobot("robot",
                               control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
ball1 = pam_mujoco.MujocoItem("ball1",
                              control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
                              color=(1,0,0,1))
ball2 = pam_mujoco.MujocoItem("ball2",
                              control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
                              color=(0,0,1,1),
                              contact_type=pam_mujoco.ContactTypes.table)
hit_point = pam_mujoco.MujocoItem("hit_point",
                                  control=pam_mujoco.MujocoItem.CONSTANT_CONTROL)
graphics=True
accelerated_time=False
handle = pam_mujoco.MujocoHandle("tutorial_4",
                                 table=True,
                                 robot1=robot,
                                 balls=(ball1,ball2),
                                 hit_points=(hit_point,),
                                 graphics=graphics,
                                 accelerated_time=accelerated_time)

# getting the frontend connected to the robot's pressure controller 
robot = handle.frontends["robot"]
ball1 = handle.frontends["ball1"]
ball2 = handle.frontends["ball2"]
hit_point = handle.frontends["hit_point"]

# moving the balls and the robot to a target position

print("starting state ball1: {}".format(ball1.latest()))
print("starting state ball2: {}".format(ball2.latest()))


# target position of balls
position1 = (0.5,3,1)
position2 = (1.5,3,1)
velocity = (0,0,0)
# target position of robot
joints = (math.pi/4.0,-math.pi/4.0,
          math.pi/4.0,-math.pi/4.0)
joint_velocities = (0,0,0,0)

duration = o80.Duration_us.seconds(2)

ball1.add_command(position1,velocity,duration,o80.Mode.QUEUE)
ball2.add_command(position2,velocity,duration,o80.Mode.QUEUE)
robot.add_command(joints,joint_velocities,duration,o80.Mode.QUEUE)

ball1.pulse()
ball2.pulse()
robot.pulse()


print("reached state ball1: {}".format(ball1.latest()))
print("reached state ball2: {}".format(ball2.latest()))
print("reached state robot: {}".format(robot.latest()))

time.sleep(3)


# reading pre-recorded trajectories


index1,trajectory1 = list(context.BallTrajectories().random_trajectory())
index2,trajectory2 = list(context.BallTrajectories().random_trajectory())

print("ball1 will play trajectory {}".format(context.BallTrajectories().get_file_name(index1)))
print("ball2 will play trajectory {}".format(context.BallTrajectories().get_file_name(index2)))

# moving ball 1 and ball 2 to first trajectories point
position1 = trajectory1[0].position 
velocity1 = trajectory1[0].velocity 
position2 = trajectory2[0].position 
velocity2 = trajectory2[0].velocity 
duration = o80.Duration_us.seconds(2)
ball1.add_command(position1,velocity1,duration,o80.Mode.QUEUE)
ball2.add_command(position2,velocity2,duration,o80.Mode.QUEUE)
ball1.pulse()
ball2.pulse()

time.sleep(4)


# loading and playing the trajectories
duration_s = 0.01
duration = o80.Duration_us.milliseconds(int(duration_s*1000))
for trajectory,ball in zip((trajectory1,trajectory2),(ball1,ball2)):
    total_duration = duration_s * len(trajectory)
    for traj_point in trajectory:
        ball.add_command(traj_point.position,traj_point.velocity,
                         duration,o80.Mode.QUEUE)
        ball1.pulse()
        ball2.pulse()


# monitoring for contact
# note: contact will not happen at all the runs of this executable,
# as the virtual table heights does not match exactly the height of the
# real table used for the recording of the ball trajectory.

print("\nmonitoring for contact ...")
time_start = time.time()
contact = None

while time.time()-time_start < 5:

    # monitoring contact
    contact = handle.get_contact("ball2")
    if contact.contact_occured:
        break

    # having the hit point following the ball
    position = ball2.latest().get_position()
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

