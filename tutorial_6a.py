import time
import math
import o80
import o80_pam


# default id when starting pam_robot executable
segment_id = "o80_pam_robot"
frontend = o80_pam.FrontEnd(segment_id)


# starting at low pressure
start_iteration = frontend.latest().get_iteration()
frontend.add_command([12000]*4,[12000]*4,
                     o80.Iteration(start_iteration+1000),
                     o80.Mode.OVERWRITE)
frontend.burst(2000)


start_iteration = frontend.latest().get_iteration()
low_pressure = 12000
high_pressure = 20000
nb_iterations = 1000

# requesting to go to high, low and again high pressures
frontend.add_command([high_pressure]*4,[high_pressure]*4,
                     o80.Iteration(start_iteration+nb_iterations),
                     o80.Mode.OVERWRITE)
frontend.add_command([low_pressure]*4,[low_pressure]*4,
                     o80.Iteration(start_iteration+2*nb_iterations),
                     o80.Mode.QUEUE)
frontend.add_command([high_pressure]*4,[high_pressure]*4,
                     o80.Iteration(start_iteration+3*nb_iterations),
                     o80.Mode.QUEUE)
frontend.pulse()


# playing the sequence by "jumps" of 2000 iterations
total_to_play = 3*nb_iterations
total_played = 0
jump = 500
while total_played<total_to_play:
    print("\nJump ...")
    frontend.burst(jump)
    total_played+=jump
    print("...wait")
    time.sleep(1.0)

