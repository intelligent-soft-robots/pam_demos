import math
import time
import o80
import context
import pam_mujoco
import vicon_transformer
import numpy as np
from scipy.spatial.transform import Rotation as R

# to run this demo
# 1) start pam_mujoco with mujoco_id "vicon_robot_init"
# 2) also network connection has to be established to the MPI-IS network
# 3) start the zmq broadcaster on vicon PC in the lab (otherwise a test frame will be read only)
# 4) execute 'pam_mujoco vicon_robot_init && python3 <pathToCode>/pam_demos/vicon_mujoco_init/robot_init.py'

def main():
    # read a test vicon frame from file
    print('read a test vicon frame from file')
    vj = vicon_transformer.ViconJson()
    print(vj.jsonObj['subjectNames'])
    vj.zmq_disconnect()

    # read distances of objects as sanity check
    vj.get_distances()

    ### translations
    # robot
    r_pos_shoulder = vj.get_robot_shoulder_pos()

    # table
    t_pos = vj.get_table_pos()

    ### rotations
    # robot
    r_rot_xy = vj.get_robot_rot()

    # table
    t_rot_xy = vj.get_table_rot()

    # creating the mujoco's configuration, and getting the handle.
    # contrary to tutorial 1 to 3, the robot will be joint controlled (i.e.
    # position and velocity).
    robot_default = pam_mujoco.MujocoRobot("robot_default", control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
    robot_vicon_init = pam_mujoco.MujocoRobot("robot_vicon_init", 
                                            control=pam_mujoco.MujocoRobot.JOINT_CONTROL,
                                            position=r_pos_shoulder,
                                            orientation=r_rot_xy)

    table = pam_mujoco.MujocoTable("table_vicon_init",
                                    position = t_pos,
                                    orientation = t_rot_xy)
    ball = pam_mujoco.MujocoItem(
        "ball", control=pam_mujoco.MujocoItem.CONSTANT_CONTROL, color=(1, 0, 0, 1)
    )

    hit_point = pam_mujoco.MujocoItem(
        "hit_point", control=pam_mujoco.MujocoItem.CONSTANT_CONTROL
    )
    graphics = True
    accelerated_time = False
    handle = pam_mujoco.MujocoHandle(
        "vicon_robot_init",
        table=table,
        robot1=robot_default, 
        robot2=robot_vicon_init,
        balls=(ball,),
        hit_points=(hit_point,),
        graphics=graphics,
        accelerated_time=accelerated_time,
    )

    # getting the frontend connected to the robot's joint controller
    robot_default = handle.frontends["robot_default"]
    robot_vicon_init = handle.frontends["robot_vicon_init"]
    ball = handle.frontends["ball"]
    hit_point = handle.frontends["hit_point"]

    # moving the balls and the robot to a target position

    print("starting state ball: {}".format(ball.latest()))


    # target position of balls
    position1 = (0.5, 3, 1)
    position2 = (1.5, 3, 1)
    velocity = (0, 0, 0)
    # target position of robot
    joints = (math.pi / 4.0, -math.pi / 4.0, math.pi / 4.0, -math.pi / 4.0)
    joint_velocities = (0, 0, 0, 0)

    duration = o80.Duration_us.seconds(2)

    ball.add_command(position1, velocity, duration, o80.Mode.QUEUE)
    robot_default.add_command(joints, joint_velocities, duration, o80.Mode.QUEUE)
    robot_vicon_init.add_command(joints, joint_velocities, duration, o80.Mode.QUEUE)

    ball.pulse()
    robot_default.pulse()
    robot_vicon_init.pulse()

    time.sleep(3)


    # reading pre-recorded trajectories


    index1, trajectory = list(context.BallTrajectories().random_trajectory())

    print(
        "ball will play trajectory {}".format(
            context.BallTrajectories().get_file_name(index1)
        )
    )
    print('traj')
    print(trajectory[0].position)

    # moving ball 1 and ball 2 to first trajectories point
    position = trajectory[0].position
    velocity = trajectory[0].velocity
    duration = o80.Duration_us.seconds(2)
    ball.add_command(position, velocity, duration, o80.Mode.QUEUE)
    ball.pulse()

    time.sleep(1)


    # loading and playing the trajectories
    duration_s = 0.01
    duration = o80.Duration_us.milliseconds(int(duration_s * 1000))
    for traj in iter(trajectory):
        #total_duration = duration_s * len(traj)
        for traj_point in traj:
            ball.add_command(
                traj_point.position, traj_point.velocity, duration, o80.Mode.QUEUE
            )
            ball.pulse()


    # monitoring for contact
    # note: contact will not happen at all the runs of this executable,
    # as the virtual table heights does not match exactly the height of the
    # real table used for the recording of the ball trajectory.


    time_start = time.time()


    while time.time() - time_start < 5:

        # having the hit point following the ball
        position = ball.latest().get_position()
        hit_point.add_command(position, [0, 0, 0], o80.Mode.OVERWRITE)
        hit_point.pulse()




    print()

if __name__ == "__main__":
    main()