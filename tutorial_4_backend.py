import pam_mujoco

model_name = "tutorial_model"
mujoco_id = "tutorial_mujoco_id"

# generation of a mujoco model xml file
# corresponding to a table, 2 balls, 1 robots
# and a goal (a goal is a simple visual marker)
items = pam_mujoco.model_factory(model_name,
                                 table=True,
                                 nb_balls=2,
                                 robot1=True,
                                 goal=True,
                                 ball_colors=((1,0,0,1),(0,0,1,1)))

# some config for mujoco
config = pam_mujoco.MujocoConfig()
config.graphics = True # set to False if you do not need graphics
config.realtime = True # set to False if you prefer accelerated time

# starting the mujoco simulation
pam_mujoco.init_mujoco(config)


# getting handles on objects
ball1 = items["balls"][0]
ball2 = items["balls"][1]
table = items["table"]
robot = items["robot"]
goal = items["goal"]

# to allow frontend to control the first ball
pam_mujoco.add_o80_ball_control("ball1",ball1)

# to allow the frontend to monitor the contact between the second ball and the table
pam_mujoco.add_table_contact("table",ball2,table)

# to allow the frontend to control the second ball,
# but mujoco's engine will take over if the ball touches the table
# ("table" is the contact id, as set right above)
pam_mujoco.add_o80_ball_control_until_contact("ball2","table",ball2)

# to allow frontend to control the goal
pam_mujoco.add_o80_goal_control("goal",goal)

# to allow frontend to control the robot's joint
# (i.e. angles and angular velocities)
pam_mujoco.add_o80_joint_control("robot",robot)


# starting the simulation
pam_mujoco.execute(mujoco_id,items["path"])


