import pam_mujoco

MUJOCO_ID = "pam_demos_contacts"
ROBOT_SEGMENT_ID = "robot"
BALL_SEGMENT_ID = "ball"


def get_handle(robot_contact):

    global MUJOCO_ID, ROBOT_SEGMENT_ID, BALL_SEGMENT_ID

    robot_control = pam_mujoco.MujocoRobot.JOINT_CONTROL
    pamy1 = True
    robot = pam_mujoco.MujocoRobot(pamy1,ROBOT_SEGMENT_ID, control=robot_control)
    table = pam_mujoco.MujocoTable("table")
    
    ball_control = pam_mujoco.MujocoItem.CONSTANT_CONTROL
    if robot_contact:
        ball_contact = pam_mujoco.ContactTypes.racket1
    else:
        ball_contact = pam_mujoco.ContactTypes.table
    ball = pam_mujoco.MujocoItem(
        BALL_SEGMENT_ID, control=ball_control, contact_type=ball_contact
    )

    graphics = True
    accelerated_time = False

    handle = pam_mujoco.MujocoHandle(
        MUJOCO_ID,
        table=table,
        robot1=robot,
        balls=(ball,),
        graphics=graphics,
        accelerated_time=accelerated_time,
    )

    return handle


def get_robot_contact_handle():
    return get_handle(True)


def get_table_contact_handle():
    return get_handle(False)
