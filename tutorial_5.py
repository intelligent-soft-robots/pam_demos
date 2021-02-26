import time
import o80
import o80_pam

frontend = o80_pam.FrontEnd("o80_pam_robot")

pressure = 0
nb_iterations = 3000
target_iteration = frontend.latest().get_iteration()+nb_iterations
frontend.add_command([pressure]*4,[pressure]*4,
                     o80.Iteration(target_iteration),
                     o80.Mode.QUEUE)
print("playing {} iterations ...".format(nb_iterations))
frontend.burst(nb_iterations)
print("... done")

time.sleep(2)

pressure = 20000
nb_iterations = 3000
target_iteration = frontend.latest().get_iteration()+nb_iterations
frontend.add_command([pressure]*4,[pressure]*4,
                     o80.Iteration(target_iteration),
                     o80.Mode.QUEUE)

iteration_per_strike=500
nb_strikes = int(nb_iterations/iteration_per_strike)
for _ in range(nb_strikes):
    print("playing {} iterations ...".format(iteration_per_strike))
    frontend.burst(iteration_per_strike)
    time.sleep(1)
print("... done")
