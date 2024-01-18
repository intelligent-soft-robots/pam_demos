from typing import Sequence
import pam_mujoco

MUJOCO_ID = "pam_demos_contacts"
ROBOT_SEGMENT_ID = "robot1"
ROBOT1_SEGMENT_ID = "robot1"
ROBOT2_SEGMENT_ID = "robot2"
BALL_SEGMENT_ID = "ball"


def get_handle(
    contact_robot1: bool,
    contact_robot2: bool,
    contact_table: bool,
    robot2_position: Sequence[float] = pam_mujoco.MujocoRobot.ROBOT2_POSITION,
):
    global MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID

    robot_control = pam_mujoco.MujocoRobot.JOINT_CONTROL
    robot1 = pam_mujoco.MujocoRobot(
        pam_mujoco.RobotType.PAMY1, ROBOT1_SEGMENT_ID, control=robot_control
    )
    robot2 = pam_mujoco.MujocoRobot(
        pam_mujoco.RobotType.PAMY1,
        ROBOT2_SEGMENT_ID,
        position=robot2_position,
        orientation=pam_mujoco.MujocoRobot.ROBOT2_ORIENTATION,
        control=robot_control,
    )

    table = pam_mujoco.MujocoTable("table")

    ball_control = pam_mujoco.MujocoItem.CONSTANT_CONTROL
    ball = pam_mujoco.MujocoItem(
        BALL_SEGMENT_ID,
        control=ball_control,
        contact_table=contact_table,
        contact_robot1=contact_robot1,
        contact_robot2=contact_robot2,
    )

    graphics = True
    accelerated_time = False

    handle = pam_mujoco.MujocoHandle(
        MUJOCO_ID,
        table=table,
        robot1=robot1,
        robot2=robot2,
        balls=(ball,),
        graphics=graphics,
        accelerated_time=accelerated_time,
    )

    return handle
