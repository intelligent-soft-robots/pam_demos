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
robot = pam_mujoco.MujocoRobot(
    pam_mujoco.RobotType.PAMY2, "robot", control=pam_mujoco.MujocoRobot.JOINT_CONTROL
)
ball1 = pam_mujoco.MujocoItem(
    "ball1", control=pam_mujoco.MujocoItem.CONSTANT_CONTROL, color=(1, 0, 0, 1)
)
ball2 = pam_mujoco.MujocoItem(
    "ball2",
    control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
    color=(0, 0, 1, 1),
    contact_table=True,
    contact_robot1=True,
)
hit_point = pam_mujoco.MujocoItem(
    "hit_point", control=pam_mujoco.MujocoItem.CONSTANT_CONTROL
)
table = pam_mujoco.MujocoTable("table")

graphics = True
accelerated_time = False
handle = pam_mujoco.MujocoHandle(
    "tutorial_4",
    table=table,
    robot1=robot,
    balls=(ball1, ball2),
    hit_points=(hit_point,),
    graphics=graphics,
    accelerated_time=accelerated_time,
)

# getting the frontend connected to the robot's pressure controller
robot = handle.frontends["robot"]
ball1 = handle.frontends["ball1"]
ball2 = handle.frontends["ball2"]
hit_point = handle.frontends["hit_point"]

# moving the balls and the robot to a target position
print("starting state ball1: {}".format(ball1.latest()))
print("starting state ball2: {}".format(ball2.latest()))

# one the way to the initial position, the contacts
# between ball2 and the table may be annoying
handle.deactivate_contact("ball2", "table")
handle.deactivate_contact("ball2", "robot1")

# target position of balls
position1 = (0.5, 3, 1)
position2 = (1.5, 3, 1)
velocity = (0, 0, 0)

# target position of robot
joints = (math.pi, -math.pi / 2.0, 0, +math.pi / 4.0)
joint_velocities = (0, 0, 0, 0)

# the balls and robot will move over 2 seconds
duration = o80.Duration_us.seconds(1)

# loading the commands
ball1.add_command(position1, velocity, duration, o80.Mode.QUEUE)
ball2.add_command(position2, velocity, duration, o80.Mode.QUEUE)
robot.add_command(joints, joint_velocities, duration, o80.Mode.QUEUE)

# sending the commands
ball1.pulse()
ball2.pulse()
robot.pulse()

# waiting for completion
time.sleep(2)

# reached state
print("reached state ball1: {}".format(ball1.latest()))
print("reached state ball2: {}".format(ball2.latest()))
print("reached state robot: {}".format(robot.latest()))

# reactivating the contacts
handle.activate_contact("ball2", "table")
handle.activate_contact("ball2", "robot1")

# reading pre-recorded trajectories
trajectory1 = context.BallTrajectories("originals").get_trajectory(0)
trajectory2 = context.BallTrajectories("originals").get_trajectory(1)
iterator1 = context.BallTrajectories.iterate(trajectory1)
iterator2 = context.BallTrajectories.iterate(trajectory2)

# moving ball 1 and ball 2 to first trajectories point
_, state1 = next(iterator1)
_, state2 = next(iterator2)
position1 = state1.get_position()
velocity1 = state1.get_velocity()
position2 = state2.get_position()
velocity2 = state2.get_velocity()
duration = o80.Duration_us.seconds(1)
ball1.add_command(position1, velocity1, duration, o80.Mode.QUEUE)
ball2.add_command(position2, velocity2, duration, o80.Mode.QUEUE)
ball1.pulse()
ball2.pulse()

time.sleep(2)


# loading and playing the trajectories
duration_s = 0.01
duration = o80.Duration_us.milliseconds(int(duration_s * 1000))
for iterator, ball in zip((iterator1, iterator2), (ball1, ball2)):
    for duration, state in iterator:
        ball.add_command(
            state.get_position(),
            state.get_velocity(),
            o80.Duration_us.microseconds(duration),
            o80.Mode.QUEUE,
        )
ball1.pulse()
ball2.pulse()


# monitoring for contact
# note: contact will not happen at all the runs of this executable,
# as the virtual table heights does not match exactly the height of the
# real table used for the recording of the ball trajectory.

print("\nmonitoring for contact ...")
detected_contacts = {"table": False, "robot1": False}
time_start = time.time()
while time.time() - time_start < 3:
    for item in detected_contacts.keys():
        if not detected_contacts[item]:
            contact = handle.get_contact("ball2", item)
            if contact.contact_occured:
                # not reporting the same contact twice
                detected_contacts[item] = True
                print(f"detected contact with {item}:")
                print("\tposition:", contact.position)
                print("\ttime stamp", contact.time_stamp)

# reporting also if contact did not occur
for item in detected_contacts.keys():
    if not detected_contacts[item]:
        contact = handle.get_contact("ball2", item)
        print(
            f"no contact with {item}, minimal distance ball/{item}: {contact.minimal_distance}"
        )

print()
