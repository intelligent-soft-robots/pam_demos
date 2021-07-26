import matplotlib.pyplot as plt
import math
import time
import o80
import pam_mujoco

mujoco_id="fk"
segment_id="robot_fk"

graphics=True
accelerated_time=False
burst_mode=False

robot_control = pam_mujoco.MujocoRobot(segment_id,
                               control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
handle = pam_mujoco.MujocoHandle(mujoco_id,
                                 graphics=graphics,
                                 accelerated_time=accelerated_time,
                                 burst_mode=burst_mode,
                                 robot1=robot_control)

robot=handle.frontends[segment_id]

def go_to_posture(postures,robot,duration_ms=1000):
    for posture in postures:
        robot.add_command(posture,(0,0,0,0),
                          o80.Duration_us.milliseconds(duration_ms),o80.Mode.QUEUE)
        robot.pulse_and_wait()


posture_start = (0,0,+math.pi/2.0,0)
posture_end = (0,0,-math.pi/2.0,0)

start_iteration=robot.latest().get_iteration()

go_to_posture( (posture_start,posture_end),
               robot)

observations = robot.get_observations_since(start_iteration)
cartesian_positions = [ o.get_cartesian_position()
                        for o in observations]

x = [cp[0] for cp in cartesian_positions]
y = [cp[1] for cp in cartesian_positions]
z = [cp[2] for cp in cartesian_positions]

plt.plot(x)
plt.plot(y)
plt.plot(z)
plt.show()
