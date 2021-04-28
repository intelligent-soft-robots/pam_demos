import pam_mujoco

model_name = "tutorial_model"
mujoco_id = "tutorial_mujoco_id"

# generation of a mujoco model xml file
# corresponding to a table, 2 balls, 1 robots
# and a goal (a goal is a simple visual marker)
items = pam_mujoco.model_factory(model_name,
                                 table=True,
                                 nb_balls=1,
                                 robot1=True)

# some config for mujoco
config = pam_mujoco.MujocoConfig()
config.graphics = True # set to False if you do not need graphics
config.realtime = True # set to False if you prefer accelerated time

# starting the mujoco simulation
pam_mujoco.init_mujoco(config)

# getting handles on objects
ball = items["ball"]
robot = items["robot"]

# to allow frontend to control the ball
pam_mujoco.add_o80_ball_control("ball",ball)

# to allow frontend to control the robot
pam_mujoco.add_o80_joint_control("robot",robot)

# starting the simulation
pam_mujoco.execute(mujoco_id,items["path"])


