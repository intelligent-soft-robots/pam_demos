import time
import pam_mujoco

robot = pam_mujoco.MujocoRobot("robot",control=pam_mujoco.MujocoRobot.JOINT_CONTROL)

handle = pam_mujoco.MujocoHandle("foo",
                                 robot1=robot)


joint = handle.interfaces["robot"]

