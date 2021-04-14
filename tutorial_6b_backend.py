import pam_mujoco
import o80_pam


model_name = "pam_tutorial_6"
mujoco_id = "tutorial_6_mujoco_id"

items = pam_mujoco.model_factory(model_name,
                                 table=True,robot1=True,
                                 muscles=False)

ball = items["ball"]
robot = items["robot"]

mconfig = pam_mujoco.MujocoConfig()
mconfig.graphics = True
mconfig.realtime = True

pam_mujoco.init_mujoco(mconfig)

pam_mujoco.add_o80_ball_control("ball",ball)
pam_mujoco.add_o80_joint_control("robot",robot)

# setting bursting mode !
pam_mujoco.add_bursting(mujoco_id,"robot")

model_path = pam_mujoco.paths.get_model_path(model_name)
pam_mujoco.execute(mujoco_id,model_path)
