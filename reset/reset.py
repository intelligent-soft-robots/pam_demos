import math
import time
import o80
import pam_mujoco

# creating the mujoco's configuration, and getting the handle
robot = pam_mujoco.MujocoRobot("robot",
                                control=pam_mujoco.MujocoRobot.JOINT_CONTROL)
ball1 = pam_mujoco.MujocoItem("ball1",
                              control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
                              color=(1,0,0,1))
ball2 = pam_mujoco.MujocoItem("ball2",
                              control=pam_mujoco.MujocoItem.CONSTANT_CONTROL,
                              color=(0,0,1,1),
                              contact_type=pam_mujoco.ContactTypes.table)
handle = pam_mujoco.MujocoHandle("pam_demos_reset",
                                 table=True,
                                 robot1=robot,
                                 balls=(ball1,ball2))

# getting the frontend connected to the robot's pressure controller 
robot = handle.frontends["robot"]
ball1 = handle.frontends["ball1"]
ball2 = handle.frontends["ball2"]

for iteration in range(5):

    print("iteration:",iteration)
    
    # setting commands that move the balls and the robots to target positions,
    # but resetting before the end

    ## target positions
    
    # target position of balls
    position1 = (0.5,3,1)
    position2 = (1.5,3,1)
    velocity = (0,0,0)

    # target position of robot1
    joints = (math.pi/4.0,-math.pi/4.0,
              math.pi/4.0,-math.pi/4.0)
    joint_velocities = (0,0,0,0)

    ## setting commands
    
    duration = o80.Duration_us.seconds(4)

    ball1.add_command(position1,velocity,duration,o80.Mode.QUEUE)
    ball2.add_command(position2,velocity,duration,o80.Mode.QUEUE)
    robot.add_command(joints,joint_velocities,duration,o80.Mode.QUEUE)


    ball1.pulse()
    ball2.pulse()
    robot.pulse()

    ## interrupting after 2 seconds
    time.sleep(2)
    print("\treset !")
    handle.reset()
    


