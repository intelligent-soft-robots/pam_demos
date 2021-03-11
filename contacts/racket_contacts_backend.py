import pam_mujoco

model_name = "tutorial_model"
mujoco_id = "tutorial_mujoco_id"

items = pam_mujoco.model_factory(model_name,
                                 table=True,
                                 nb_balls=1,
                                 robot1=True)

config = pam_mujoco.MujocoConfig()
config.graphics = True 
config.realtime = True 
pam_mujoco.init_mujoco(config)

ball = items["ball"]
robot = items["robot"]

pam_mujoco.add_robot_contact("robot_contact",ball,robot)
pam_mujoco.add_o80_ball_control_until_contact("ball","robot_contact",ball)
pam_mujoco.add_o80_joint_control("robot",robot)

pam_mujoco.execute(mujoco_id,items["path"])


