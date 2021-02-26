import time
import o80
import o80_pam

ball1 = o80_pam.BallFrontEnd("ball1")
ball2 = o80_pam.BallFrontEnd("ball2")

print("starting state ball1: {}".format(ball1.latest()))
print("starting state ball2: {}".format(ball2.latest()))

position1 = (0.5,1,1)
position2 = (0.5,2,1)
velocity = (0,0,0)
duration = o80.Duration_us.seconds(2)

ball1.add_command(position1,velocity,duration,o80.Mode.QUEUE)
ball2.add_command(position2,velocity,duration,o80.Mode.QUEUE)

ball1.pulse()
ball2.pulse()

time.sleep(3)

print("reached state ball1: {}".format(ball1.latest()))
print("reached state ball2: {}".format(ball2.latest()))
