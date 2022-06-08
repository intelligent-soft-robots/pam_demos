'''
Provides some statistics about the frequency of a o80 backend
controlling the pressures of a robot.
Before running this script, one of "o80_pamy1", "o80_pamy2" or
"o80_dummy" must have been started in another terminal.
This script will have the robot performing some commands, and then
display information about the frequency of the backend. 
'''

import numpy
import math
import o80
import o80_pam
import pam_mujoco

# should be the same as the segment_id used by the robot backend.
# "real_robot" is the default.
ROBOT_SEGMENT_ID = "real_robot"

def run(robot_segment_id: str):

    # frontend to the robot. o80_pamy1, o80_pamy2 or o80_dummy
    # expected to have been started from another terminal
    robot : o80_pam.FrontEnd = o80_pam.FrontEnd(robot_segment_id)
    
    # getting the number of the current iteration
    iteration_start = robot.latest().get_iteration()

    # sending some commands
    print("sending some commands:")

    for iteration in range(5):

        print("\titeration {}/5".format(iteration+1))
        
        ## sending two commands and waiting for completion
        pressure = 12000
        duration = 1  # seconds
        robot.add_command(
            [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
        )
        pressure = 20000
        robot.add_command(
            [pressure] * 4, [pressure] * 4, o80.Duration_us.seconds(duration), o80.Mode.QUEUE
        )
        robot.pulse_and_wait()

        ## creating a sinusoid trajectory and playing it
        pressure = 15000
        v = 0.0
        increment = 0.025
        amplitude = 3000
        dof = 2
        for _ in range(5000):
            v += increment
            ago = pressure + int(amplitude * math.sin(v))
            antago = pressure - int(amplitude * math.sin(v))
            robot.add_command(dof, ago, antago, o80.Mode.QUEUE)
        # playing the trajectory
        robot.pulse_and_wait()


    # check how the frequency of the backend did

    ## backend frequency and allowed limits
    expected_frequency = 500.
    five_percent = expected_frequency * 0.05
    ten_percent = expected_frequency * 0.1
    up_tolerance1 = expected_frequency + five_percent
    up_tolerance2 = expected_frequency + ten_percent
    down_tolerance1 = expected_frequency - five_percent
    down_tolerance2 = expected_frequency - ten_percent
    
    ## getting all observations
    observations = robot.get_observations_since(iteration_start)
    nb_observations = len(observations)

    ## reading the frequency of each step
    frequencies = [o.get_frequency() for o in observations]
    
    ## some stats
    max_frequency = max(frequencies)
    min_frequency = min(frequencies)
    up_spikes1 = [f for f in frequencies if f > up_tolerance1]
    nb_up_spikes1 = len(up_spikes1)
    up_spikes2 = [f for f in frequencies if f > up_tolerance2]
    nb_up_spikes2 = len(up_spikes2)
    down_spikes1 = [f for f in frequencies if f < down_tolerance1]
    nb_down_spikes1 = len(down_spikes1)
    down_spikes2 = [f for f in frequencies if f < down_tolerance2]
    nb_down_spikes2 = len(down_spikes2)
    decent_frequencies = [
        f for f in frequencies if f <= up_tolerance1 and f >= down_tolerance1
    ]
    if decent_frequencies:
        average = sum(decent_frequencies) / len(decent_frequencies)
        std = numpy.std(numpy.array(decent_frequencies))
    else:
        average,std = None,None
        
    ## printing results
    print()
    print("Frequency monitoring, over {} iterations".format(nb_observations))
    print("\texpected frequency: {}".format(expected_frequency))
    print(
        "\tspikes over {}: {} ({}%)".format(
            up_tolerance1, nb_up_spikes1, 100. * float(nb_up_spikes1) / nb_observations
        )
    )
    print(
        "\tspikes over {}: {} ({}%)".format(
            up_tolerance2, nb_up_spikes2, 100. * float(nb_up_spikes2) / nb_observations
        )
    )
    print(
        "\tspikes below {}: {} ({}%)".format(
            down_tolerance1, nb_down_spikes1, 100. * float(nb_down_spikes1) / nb_observations
        )
    )
    print(
        "\tspikes below {}: {} ({}%)".format(
            down_tolerance2, nb_down_spikes2, 100. * float(nb_down_spikes2) / nb_observations
        )
    )
    print("\tmax frequency observed: {}".format(max_frequency))
    print("\tmin frequency observed: {}".format(min_frequency))
    if average and std:
        print("\taverage frequency (spikes excluded): {}".format(average))
        print("\tstandard deviation (spikes excluded): {}".format(std))
    else:
        print("\taverage frequency (spikes excluded): no data")
        print("\tstandard deviation (spikes excluded): no data")
    print()


if __name__ == "__main__":
    run(ROBOT_SEGMENT_ID)
