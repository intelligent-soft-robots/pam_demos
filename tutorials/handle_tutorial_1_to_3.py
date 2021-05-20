import pam_mujoco


def get_handle():

    # what a "handle" do:
    
    # 1. it creates a configuration for a waiting instance of pam_mujoco,
    #    i.e. it assumes that "pam_mujoco o80_pam_robot" has already been called
    # 2. it writes this configuration in a shared memory. As a result, the waiting
    #    instance of pam_mujoco can start
    # 3. it provides to the user code access to the o80 frontend(s) that will
    #    allow to send command to the pam mujoco instance

    # it is assumed that the waiting instance of pam_mujoco was started using
    # this same mujoco_id
    mujoco_id = "tutorials_1_to_3"

    # the o80 segment id of the robot. You can choose an arbitrary string
    robot_segment_id = "robot"

    # the robot will be pressured control (i.e. use pneumatic muscle for control)
    control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL

    # to get a graphical view of the robot
    graphics=True

    # will run in "natural" time
    # You can set it to True and see how the tutorials script behave.
    accelerated_time=False

    # creating the robot, specifying pressure control
    robot = pam_mujoco.MujocoRobot(robot_segment_id,
                                   control=control)

    
    # during its construction, the handle will write the configuration in the
    # shared memory, and pam_mujoco will start.
    # Note: if the pam mujoco instance has already been configured and is
    #       already running, this will have no effect (the running instance
    #       will not be reconfigured)
    handle = pam_mujoco.MujocoHandle(mujoco_id,
                                     robot1=robot,
                                     graphics=graphics,
                                     accelerated_time=accelerated_time)

    # so that the client can access the frontend (to send pressure commmands in this
    # case)
    return handle
