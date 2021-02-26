import time
import o80
import o80_pam

frontend = o80_pam.FrontEnd("o80_pam_robot")


start_iteration = frontend.latest().get_iteration()

pressure1 = 20000
target_iteration1 = start_iteration+1000
frontend.add_command([pressure1]*4,[pressure1]*4,
                     o80.Iteration(target_iteration1),
                     o80.Mode.QUEUE)

pressure2 = 0
target_iteration2 = start_iteration+1500
frontend.add_command([pressure2]*4,[pressure2]*4,
                     o80.Iteration(target_iteration2),
                     o80.Mode.QUEUE)

pressure3 = 10000
target_iteration3 = start_iteration+2000
frontend.add_command([pressure3]*4,[pressure3]*4,
                     o80.Iteration(target_iteration3),
                     o80.Mode.QUEUE)

print("sending commands")
frontend.pulse()

print("waiting for iteration: {}".format(target_iteration1))
observation = frontend.read(target_iteration1)
print("reached iteration: {}".format(observation.get_iteration()))

print("waiting for iteration: {}".format(target_iteration2))
observation = frontend.read(target_iteration2)
print("reached iteration: {}".format(observation.get_iteration()))

print("waiting for iteration: {}".format(target_iteration3))
observation = frontend.read(target_iteration3)
print("reached iteration: {}".format(observation.get_iteration()))


print("\nsynchronizing frontend and backend:")

pressure = 10000
target_iteration = frontend.latest().get_iteration()+5
backend_frequency = frontend.latest().get_frequency()
time_start = time.time()
print("running frontend at 1/5th of backend frequency ({} Hz)".format(backend_frequency/5.0))
for _ in range(500):
    previous_iteration = target_iteration
    target_iteration += 5
    pressure+=20
    frontend.add_command([pressure]*4,[pressure]*4,
                         o80.Iteration(target_iteration),
                         o80.Mode.QUEUE)
    frontend.pulse()
    frontend.read(previous_iteration)
time_end = time.time()
print("frequency: {} Hz".format(500.0/(time_end-time_start)))
    

print("\nrelative iteration:")
# this corresponds to 1000 iterations counting
# from the moment the command is started
relative=True
reset=True
iteration = o80.Iteration(1000,relative,reset)
pressure=0
# i.e. reaching pressure 0 in 1000 iteration
frontend.add_command([pressure]*4,[pressure]*4,
                     iteration,
                     o80.Mode.QUEUE)
obs1 = frontend.pulse_and_wait()
# this corresponds to iteration number 2000
# counting from relative iteration number at
# which the previous command was started. 
relative=True
reset=False
iteration = o80.Iteration(2000,relative,reset)
pressure=16000
# i.e. reaching pressure 16000 in 1000 iterations
# (previous command took 1000 iterations, and this
#  want to reach a relative iteration number 2000)
frontend.add_command([pressure]*4,[pressure]*4,
                     iteration,
                     o80.Mode.QUEUE)
obs2 = frontend.pulse_and_wait()
print("number of iterations of second commands: "
      "{}".format(obs2.get_iteration()-obs1.get_iteration()))
