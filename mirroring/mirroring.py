import time
import o80_pam
import pam_mujoco
import o80
from pam_mujoco import mirroring

MUJOCO_ID_PSEUDO_REAL = "pressure"
MUJOCO_ID_SIMULATED = "joint"

SEGMENT_ID_PSEUDO_REAL_ROBOT = "pseudo-real-robot"
SEGMENT_ID_ROBOT_MIRROR = "simulated-robot"


def configure_pseudo_real(
    mujoco_id=MUJOCO_ID_PSEUDO_REAL, graphics=True, accelerated_time=False
):

    if accelerated_time:
        burst_mode = True
    else:
        burst_mode = False

    robot = pam_mujoco.MujocoRobot(
        False, SEGMENT_ID_PSEUDO_REAL_ROBOT, control=pam_mujoco.MujocoRobot.PRESSURE_CONTROL
    )
    handle = pam_mujoco.MujocoHandle(
        mujoco_id,
        graphics=graphics,
        accelerated_time=accelerated_time,
        burst_mode=burst_mode,
        robot1=robot,
    )

    return handle


def configure_simulation(mujoco_id=MUJOCO_ID_SIMULATED, graphics=True):

    accelerated_time = True
    burst_mode = True

    robot = pam_mujoco.MujocoRobot(
        SEGMENT_ID_ROBOT_MIRROR, control=pam_mujoco.MujocoRobot.JOINT_CONTROL
    )
    handle = pam_mujoco.MujocoHandle(
        mujoco_id,
        graphics=graphics,
        accelerated_time=accelerated_time,
        burst_mode=burst_mode,
        robot1=robot,
    )

    return handle


accelerated = False

real_robot_handle = configure_pseudo_real(accelerated_time=accelerated)
mirroring_handle = configure_simulation()

pressures = real_robot_handle.interfaces[SEGMENT_ID_PSEUDO_REAL_ROBOT]
joints = mirroring_handle.interfaces[SEGMENT_ID_ROBOT_MIRROR]

posture1 = [[19900, 16000], [16800, 19100], [18700, 17300], [18000, 18000]]
posture2 = [[16000, 19900], [19100, 16800], [17300, 18700], [18000, 18000]]


t = time.time()

mirroring.align_robots(pressures, joints)

mirroring.go_to_pressure_posture(pressures, joints, posture1, 5, accelerated)

mirroring.go_to_pressure_posture(pressures, joints, posture2, 5, accelerated)

print(time.time() - t)
