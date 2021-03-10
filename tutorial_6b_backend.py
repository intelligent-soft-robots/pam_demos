import pam_mujoco
import o80_pam


model_name = "pam_tutorial_6"
segment_id = "pam_tutorial_6"

items = pam_mujoco.model_factory(model_name,
                                 table=True,robot1=True,
                                 muscles=False)

ball = items["ball"]
robot = items["robot"]


mconfig = pam_mujoco.MujocoConfig()
mconfig.graphics = config.graphics
mconfig.realtime = True

pam_mujoco.init_mujoco(mconfig)

pam_mujoco.add_mirror_until_contact_free_joint(config.segment_id_ball,
                                               ball.joint,
                                               ball.index_qpos,ball.index_qvel,
                                               config.segment_id_contact_robot)
pam_mujoco.add_mirror_robot(config.segment_id_robot_mirror,robot.joint)


# setting bursting mode !
pam_mujoco.add_bursting(config.mujoco_id,config.segment_id_robot_mirror)

model_path = pam_mujoco.paths.get_model_path(model_name)
pam_mujoco.execute(config.mujoco_id,model_path)
