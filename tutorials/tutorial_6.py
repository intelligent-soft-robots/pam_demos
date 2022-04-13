import time
import o80
import pam_mujoco

# to run this tutorial, start pam_mujoco with mujoco_id "tutorial_6"

robot = pam_mujoco.MujocoRobot(pam_mujoco.RobotType.PAMY2, "robot", control=pam_mujoco.MujocoRobot.PRESSURE_CONTROL)
handle = pam_mujoco.MujocoHandle("tutorial_6", robot1=robot, burst_mode=True)


frontend = handle.frontends["robot"]


# starting at low pressure
start_iteration = frontend.latest().get_iteration()
frontend.add_command(
    [12000] * 4, [12000] * 4, o80.Iteration(start_iteration + 1000), o80.Mode.OVERWRITE
)
# sharing the command
frontend.pulse()

# requesting mujoco to perform 200 iterations
handle.burst(2000)


start_iteration = frontend.latest().get_iteration()
low_pressure = 12000
high_pressure = 20000
nb_iterations = 1000

# requesting to go to high, low and again high pressures
frontend.add_command(
    [high_pressure] * 4,
    [high_pressure] * 4,
    o80.Iteration(start_iteration + nb_iterations),
    o80.Mode.OVERWRITE,
)
frontend.add_command(
    [low_pressure] * 4,
    [low_pressure] * 4,
    o80.Iteration(start_iteration + 2 * nb_iterations),
    o80.Mode.QUEUE,
)
frontend.add_command(
    [high_pressure] * 4,
    [high_pressure] * 4,
    o80.Iteration(start_iteration + 3 * nb_iterations),
    o80.Mode.QUEUE,
)
frontend.pulse()


# playing the sequence by "jumps" of 2000 iterations
total_to_play = 3 * nb_iterations
total_played = 0
jump = 500
while total_played < total_to_play:
    print("\nJump ...")
    handle.burst(jump)
    total_played += jump
    print("...wait")
    time.sleep(1.0)
