import math
import time
import context
import o80
import o80_pam
import pam_mujoco
import vicon_transformer
import numpy as np
from scipy.spatial.transform import Rotation as R

# to run this demo, start pam_mujoco with mujoco_id "vicon_robot_init"

# read a test vicon frame from file
print('read a test vicon frame from file')
vj = vicon_transformer.ViconJson()
print(vj.j['subjectNames'])

### translations

# robot
r_pos=vj.get_robot_base_trans()
# add translation from vicon frame to shoulder frame
tr_to_shoulder = np.asarray([.25,.075,0])
r_pos_shoulder = r_pos+tr_to_shoulder
print('robot position from vicon')
print(r_pos_shoulder)

# table
t_pos_1=vj.get_table1_trans()
t_pos_2=vj.get_table2_trans()
t_pos_3=vj.get_table3_trans()
t_pos_4=vj.get_table4_trans()
t_x = (t_pos_1[0]+t_pos_2[0]+t_pos_3[0]+t_pos_4[0])/4
t_y = (t_pos_1[1]+t_pos_2[1]+t_pos_3[1]+t_pos_4[1])/4
t_z = (t_pos_1[2]+t_pos_2[2]+t_pos_3[2]+t_pos_4[2])/4
print('table position from vicon ('+str(t_x)+' '+str(t_y)+' '+str(t_z)+')')
t_pos = np.asarray([t_x,t_y,t_z])

### rotations

## robot
# read from vicon json
r_rot_mat = vj.get_robot_base_rot()
print('robot rot mat'+str(r_rot_mat))
# rotate robot base vicon frame to robot shoulder frame
# 1) z axis 180 deg by flipping x & y
r_tmp = r_rot_mat[1,:].copy()
print('r_tmp 1'+str(r_tmp))
r_rot_mat[1,:] = r_rot_mat[0,:].copy()
print('robot rot mat X'+str(r_rot_mat))
print('r_tmp 2'+str(r_tmp))
r_rot_mat[0,:] = r_tmp.copy()
print('robot rotshoulder mat'+str(r_rot_mat))
# bring into xy_axes form
r_rot_xy_ = np.concatenate((r_rot_mat[0,:].T,r_rot_mat[1,:].T),axis=0)
r_rot_xy = np.squeeze(np.asarray(r_rot_xy_))
print('r_rot_xy'+str(r_rot_xy))


# table
t_rot_mat = vj.get_table_rot1()
t_rot_xy_ = np.concatenate((t_rot_mat[0,:].T,t_rot_mat[1,:].T),axis=0)
t_rot_xy = np.squeeze(np.asarray(t_rot_xy_))
print('table rot mat 1 '+str(t_rot_xy))
# t_rot_mat = vj.get_table_rot2()
# t_rot_xy_ = np.concatenate((t_rot_mat[0,:].T,t_rot_mat[1,:].T),axis=0)
# t_rot_xy = np.squeeze(np.asarray(t_rot_xy_))
# print('table rot mat 2 '+str(t_rot_xy))
# t_rot_mat = vj.get_table_rot3()
# t_rot_xy_ = np.concatenate((t_rot_mat[0,:].T,t_rot_mat[1,:].T),axis=0)
# t_rot_xy = np.squeeze(np.asarray(t_rot_xy_))
# print('table rot mat 3 '+str(t_rot_xy))
# t_rot_mat = vj.get_table_rot4()
# t_rot_xy_ = np.concatenate((t_rot_mat[0,:].T,t_rot_mat[1,:].T),axis=0)
# t_rot_xy = np.squeeze(np.asarray(t_rot_xy_))
# print('table rot mat 4 '+str(t_rot_xy))



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

# getting the frontend connected to the robot's pressure controller
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


print("reached state ball: {}".format(ball.latest()))
print("reached state robot: {}".format(robot_default.latest()))
print("reached state robot: {}".format(robot_vicon_init.latest()))

time.sleep(3)


# reading pre-recorded trajectories


index1, trajectory = list(context.BallTrajectories().random_trajectory())

print(
    "ball will play trajectory {}".format(
        context.BallTrajectories().get_file_name(index1)
    )
)


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
for traj in zip(trajectory):
    total_duration = duration_s * len(traj)
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