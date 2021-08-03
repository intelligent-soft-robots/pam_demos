import math
import time
import context
import o80
import o80_pam
import pam_mujoco

mujoco_id = "balls_trajectories"

# only accepted number of balls !
# nb_balls = 3
# nb_balls = 10
nb_balls = 20
## Boost error ! shared memory size issue ?
# nb_balls = 50
# nb_balls = 100

robot = pam_mujoco.MujocoRobot("robot", control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
balls = pam_mujoco.MujocoItems("balls")
ball_segment_ids = ["ball_" + str(index) for index in range(nb_balls)]
for index in range(nb_balls):
    ball = pam_mujoco.MujocoItem(
        ball_segment_ids[index],
        control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
        contact_type=pam_mujoco.ContactTypes.racket1,
    )
    balls.add_ball(ball)

graphics = True
accelerated_time = False

handle = pam_mujoco.MujocoHandle(
    mujoco_id,
    table=True,
    robot1=robot,
    combined=balls,
    graphics=graphics,
    accelerated_time=accelerated_time,
)

robot = handle.frontends["robot"]
balls = handle.frontends["balls"]

item3d = o80.Item3dState()
trajectories_generator = context.BallTrajectories()
sampling_rate_ms = trajectories_generator.get_sampling_rate_ms()
duration = o80.Duration_us.milliseconds(int(sampling_rate_ms))

for iteration in range(5):

    print("\niteration:", iteration)

    for ball_index in range(nb_balls):
        handle.reset_contact(ball_segment_ids[ball_index])

    for index_ball in range(nb_balls):
        _, trajectory = trajectories_generator.random_trajectory()
        # going to first trajectory point
        item3d.set_position(trajectory[0].position)
        item3d.set_velocity([0] * 3)
        balls.add_command(
            index_ball, item3d, o80.Duration_us.seconds(1), o80.Mode.QUEUE
        )
        # loading full trajectory
        for item in trajectory[1:]:
            item3d.set_position(item.position)
            item3d.set_velocity(item.velocity)
            balls.add_command(index_ball, item3d, duration, o80.Mode.QUEUE)

    # target position of robot
    joints = (0, +math.pi / 3.0, -math.pi / 4.0, +math.pi / 4.0)
    joint_velocities = (0, 0, 0, 0)
    robot.add_command(
        joints, joint_velocities, o80.Duration_us.seconds(2), o80.Mode.QUEUE
    )

    balls.pulse()
    robot.pulse()

    # monitoring for contacts and ball position.
    # stopping when one of the balls is below -0.5
    contact_detected = [False for _ in range(nb_balls)]
    while True:
        # printing contacts
        for ball_index in range(nb_balls):
            contact = handle.get_contact(ball_segment_ids[ball_index])
            if contact.contact_occured and not contact_detected[ball_index]:
                contact_detected[ball_index] = True
                print(
                    "contact ball {} with racket at {}".format(
                        ball_index, contact.position
                    )
                )
        # checking positions to see if we should exit
        stop = False
        observation = balls.latest()
        states = observation.get_observed_states()
        for ball_index in range(nb_balls):
            position = states.get(ball_index).get_position()
            if position[2] < -0.5:
                stop = True
        if stop:
            break
        time.sleep(0.5)

    handle.reset()

print()
