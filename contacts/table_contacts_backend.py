import pam_mujoco

model_name = "tutorial_model"
mujoco_id = "tutorial_mujoco_id"

items = pam_mujoco.model_factory(model_name,
                                 table=True)

config = pam_mujoco.MujocoConfig()
config.graphics = True # set to False if you do not need graphics
config.realtime = True # set to False if you prefer accelerated time

pam_mujoco.init_mujoco(config)


ball = items["ball"]
table = items["table"]

# to allow the frontend to monitor the contact between the ball and the table
pam_mujoco.add_table_contact("table",ball,table)

# to allow the frontend to control the ball,
# but mujoco's engine will take over if the ball touches the table
# ("table" is the contact id, as set right above)
pam_mujoco.add_o80_ball_control_until_contact("ball","table",ball)


pam_mujoco.execute(mujoco_id,items["path"])


