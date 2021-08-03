import math, time
import o80
from handle_tutorial_1_to_3 import get_handle

# how to run:
# first call in another terminal "pam_mujoco tutorials_1_to_3"
# then start this file (python3 ./tutoral1.py)

# configuring pam_mujoco to implement a pressure controlled robot
handle = get_handle()

# getting the frontend connected to the robot's pressure controller
# Note: "robot" is the frontend segment id of the robot, see handle_tutorial_1_to_3.py
frontend = handle.frontends["robot"]

pressure = 20000
duration = 1  # seconds
# creating a command locally. The command is *not* sent to the robot yet.
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)

# sending the command to the robot, and waiting for its completion.
frontend.pulse_and_wait()


###
###
###

# adding a first command
pressure = 12000
duration = 1  # seconds
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)

# adding a second command
pressure = 20000
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)


# sending both commands to the robot, and waiting for their completions.
frontend.pulse_and_wait()


###
###
###


pressure = 15000
duration = 2
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)
pressure = 20000
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)
# sending command to the robot, and returning immediately
frontend.pulse()


###
###
###


pressure = 0
duration = 1
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)
frontend.pulse()


###
###
###


pressure = 0
duration = 1
frontend.add_command(
    [pressure] * 4,
    [pressure] * 4,
    o80.Duration_us.seconds(duration),
    o80.Mode.OVERWRITE,
)
frontend.pulse()


###
###
###


pressure = 12000
duration = 2
frontend.add_command(
    [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
)
frontend.pulse()
time.sleep(5)
pressure = 0
duration = 1
frontend.add_command(
    [pressure] * 4,
    [pressure] * 4,
    o80.Duration_us.seconds(duration),
    o80.Mode.OVERWRITE,
)
frontend.pulse()


###
###
###


muscle = 0
frontend.add_command(muscle, 12000, o80.Duration_us.seconds(3), o80.Mode.OVERWRITE)
frontend.pulse()


###
###
###

frontend.add_command(0, 15000, o80.Duration_us.seconds(3), o80.Mode.OVERWRITE)
frontend.add_command(1, 20000, o80.Duration_us.seconds(6), o80.Mode.OVERWRITE)
frontend.pulse()


###
###
###


dof = 0
frontend.add_command(dof, 20000, 15000, o80.Duration_us.seconds(3), o80.Mode.OVERWRITE)
frontend.pulse()


###
###
###

agonists = [15000, 16000, 17000, 18000]
antagonists = [18000, 17000, 16000, 15000]
frontend.add_command(
    agonists, antagonists, o80.Duration_us.seconds(3), o80.Mode.OVERWRITE
)
frontend.pulse()


###
###
###


agonists = [15000, 16000, 17000, 18000]
antagonists = [18000, 17000, 16000, 15000]
frontend.add_command(
    dof, 20000, 15000, o80.Speed.per_second(300 * 3), o80.Mode.OVERWRITE
)
dof = 0
frontend.add_command(dof, 20000, 15000, o80.Speed.per_second(200 * 3), o80.Mode.QUEUE)
frontend.add_command(3, 15000, o80.Speed.per_second(400 * 3), o80.Mode.OVERWRITE)
frontend.add_command(4, 20000, o80.Speed.per_second(300 * 3), o80.Mode.OVERWRITE)
frontend.pulse()

###
###
###


# creating a sinusoid trajectory
pressure = 15000
v = 0.0
increment = 0.025
amplitude = 3000
dof = 2
for _ in range(5000):
    v += increment
    ago = pressure + int(amplitude * math.sin(v))
    antago = pressure - int(amplitude * math.sin(v))
    frontend.add_command(dof, ago, antago, o80.Mode.QUEUE)
    # playing the trajectory
    frontend.pulse_and_wait()
