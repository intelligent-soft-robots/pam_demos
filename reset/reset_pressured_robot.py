import math
import time
import o80
import pam_mujoco

robot = pam_mujoco.MujocoRobot(
    pam_mujoco.RobotType.PAMY2,
    "robot",
    control=pam_mujoco.MujocoRobot.PRESSURE_CONTROL,
)
handle = pam_mujoco.MujocoHandle("pam_demos_reset", robot1=robot)
robot = handle.frontends["robot"]

start_obs = robot.pulse()
start_obs_pressures = start_obs.get_observed_pressures()
start_des_pressures = start_obs.get_desired_pressures()

print("\n---start:")
print("observed pressures:", start_obs_pressures)
print("desired pressures:", start_des_pressures)
print("---\n")

pressures = [15000, 20000, 12000, 20000, 0]

for pressure in pressures:
    print("target pressure:", pressure)

    duration = 5  # seconds
    robot.add_command(
        [pressure] * 4,
        [pressure] * 4,
        o80.Duration_us.seconds(duration),
        o80.Mode.QUEUE,
    )
    robot.pulse()
    time.sleep(3)
    print("reset !")
    handle.reset()
    time_start = time.time()
    while time.time() - time_start < 3:
        observation = robot.pulse()
        current_pressures = observation.get_desired_pressures()
        print("\tpressures:", current_pressures)
        time.sleep(0.5)
