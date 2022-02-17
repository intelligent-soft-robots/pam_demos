"""
Demo for o80_pam.PositionController
The (pressure controlled) robot is requested to return to a specified position
after random motion.
Required before running:
  pam_mujoco position_control
"""

from typing import Sequence,Tuple
import random
import time
from datetime import datetime
import math
import matplotlib.pyplot as plt
import o80
import pam_interface
import o80_pam
import pam_mujoco
from numpy import array
import numpy as np
from lightargs import BrightArgs,FileExists


ROBOT_SEGMENT_ID = "robot"
MUJOCO_ID = "position_control"

## gains for simulated robot ##
#KP = [0.007,0.4,3.0,0.8]
#KI=[0.002,0.0001,0.0,0.005]
#KD = [0.007,0.01,0.00,0.004]
#NDP = [0.1,-0.3,0.05,0.2]

## gains for real robot (pamy1) ##
#KP = [ 0.8 , -3.0  , 1.2,  -1.0 ]
#KI = [ 0.015 , -0.25 , 0.02 ,-0.05 ]
#KD = [ 0.04 , -0.09 , 0.09 , -0.09 ]
#NDP = [ -0.3 , -0.5 , -0.34 , -0.48 ]

## gains for real robot (pamy2) ##
# KP = [ 10000 , 0.1 , 0.1,  0.1 ]
# KI = [ 0.0 , 0.0 , 0.0 , 0.0 ]
# KD = [ 0.0 , 0.0 , 0.0 , 0.0 ]
# NDP = [ -0.5 , 0.5 , 0.5 , 0.5] 


# gains, advice:
# starting plotting the pressure values,
# i.e. running o80_plotting
# is helpful to tune NDP



# TIME_STEP = 0.01 # seconds
# QD_DESIRED = [0.7,0.7,0.7,0.7] # radian per seconds
PI4 = math.pi/4.
Q_DESIRED = [0,-PI4,0,0] # radian per seconds

def plot_angle(axis,data,q_traj,dof):
    # plot data (data as returned
    # by reach target above)
    # print(array(q_traj).shape[1])
    # print(type(q_traj))    
    angle = [d.get_positions()[dof]*180/math.pi
            for d in data]
    tr = q_traj[dof]#[x[dof] for x in q_traj]
    axis.set_title("dof {}| max {:.2f} | min {:.2f} ".format(dof,max(angle),min(angle))) 
    iterations = list(range(len(data)))
    iterations_q = list(range(array(q_traj).shape[1]))
    axis.plot(iterations_q,tr/math.pi*180 ,c='red')
    axis.plot(iterations,angle,c='blue')
    axis.grid(True)

def plot_pr(axis,data,dof,sign):
    # plot data (data as returned
    # by reach target above)
    if sign == 0:
        print("plotting for dof",dof,"agonist")
        s = "agonist"
    else:
        print("plotting for dof",dof,"antagonist")
        s = "antagonist"
    axis.set_title(str(dof)+" | "+s)
    observed = [d.get_observed_pressures()[dof][sign]
                for d in data]
    desired = [d.get_desired_pressures()[dof][sign]
                for d in data]
    
    iterations = list(range(len(data)))
    axis.plot(iterations,observed,c='blue')
    axis.plot(iterations,desired,c='red')
    axis.grid(True)
    
def _get_handle() -> pam_mujoco.MujocoHandle :
    """
    Configure mujoco to be realtime, to have graphics,
    and to setup a pressure controlled robot
    """
    burst_mode=False
    accelerated_time=False
    control = pam_mujoco.MujocoRobot.PRESSURE_CONTROL
    graphics = True
    robot = pam_mujoco.MujocoRobot(ROBOT_SEGMENT_ID, control=control)
    handle = pam_mujoco.MujocoHandle(
        MUJOCO_ID, robot1=robot, graphics=graphics, accelerated_time=accelerated_time,
        burst_mode=burst_mode
    )
    return handle


def _get_interface_and_frontend(segment_id:str) -> Tuple[o80_pam.o80Pressures,o80_pam.FrontEnd]:
    """
    Returns interface and frontend to the robot.
    If segment_id is empty: mujoco simulated robot
    For real robot: segment_id must be the segment_id for
    the robot backend.
    """
    if segment_id:
        # segment id: i.e. real robot
        frontend = o80_pam.FrontEnd(segment_id)
        interface = o80_pam.o80Pressures(segment_id,frontend=frontend)
        return (interface,frontend)

    handle = _get_handle()
    frontend = handle.frontends["robot"]
    interface = handle.interfaces["robot"]
    return (interface,frontend)

def _go_to_itz(
    pam_config: pam_interface.Configuration,
        interface:o80_pam.o80Pressures,
        q_desired: Sequence[float], ):

    def deltaP(min_ago,max_ago,min_antago,max_antago,ndp,u):
        if u>1:
            u_=1
        elif u<-1:
            u_=-1
        else:
            u_=u
        p_ago = (max_ago-min_ago)*(ndp-u)+min_ago
        p_antago = (max_antago-min_antago)*(ndp+u_)+min_antago
        if p_ago>max_ago:
            p_ago=max_ago
        if p_ago<min_ago:
            p_ago=min_ago
        if p_antago>max_antago:
            p_antago=max_antago
        return np.asarray([int(p_ago),int(p_antago)]).T

    # moves the pseudo-real robot to a desired position (in radians)
    # via a position controller (i.e. compute the pressure trajectory
    # required to reach, hopefully, for the position) in
    # synchronization with the simulated robot(s).
    
    TESTMODE = True
    
    if TESTMODE:
        fdir = "/home/vberenz/Desktop/pid_position_tuning/"
        fig = plt.figure()
        now = datetime.now()
        
        interface,frontend = _get_interface_and_frontend("real_robot")
        start_iteration = frontend.latest().get_iteration()  

    # max and min pressures
    p_min_ago=[15000,15000,13000,10000]
    p_min_ant=[17000,17000,13000,10000]
    p_max_ago=[25000,21500,21000,20000]
    p_max_ant=[25000,25000,21000,20000]
        
    # configuration for the controller
    KP  = [ .15 , 0.5 , 0.1,  0.1 ]
    KI  = [ 0.001 , 0.002 , 0.001 , 0.001 ]
    KD  = [ 0.0 , 0.0 , 0.0 , 0.0 ]
    NDP = [ 0.3 , 0.3 , 0.3 , 0.3 ]
    PI4 = math.pi/4
    TIME_STEP = 0.01 # seconds
    Q_DESIRED = [0,-PI4,0,0] # radian per seconds
    QD_DESIRED = [0.05,0.075,0.1,0.1] # radian per seconds
    post_time = 20 # in seconds
    nb_dofs = 4
    post_time_steps =  math.ceil(post_time/TIME_STEP)
    
    # controlling
    qerrsum=[0.,0.,0.,0.]
    qerr=[0.,0.,0.,0.]
    dqerr=[0.,0.,0.,0.]
    u=[0.,0.,0.,0.]
    p_ago = [0,0,0,0]
    p_ant = [0,0,0,0]
    pressures=np.asarray([p_ago,p_ant]).T

    # current error
    if TESTMODE:
        _,_,q,_ = interface.read()
    else:
        _,_,q,_ = self._pressure_commands.read()

    for i_dof in range(nb_dofs):
        qerr[i_dof] = Q_DESIRED[i_dof]-q[i_dof]
    
    # calulate max steps
    max_steps = -9999999
    steps = [0.,0.,0.,0.]
    for i_dof in range(nb_dofs):
        steps[i_dof] = math.ceil(math.fabs(qerr[i_dof]/TIME_STEP/QD_DESIRED[i_dof]))
        if(max_steps < steps[i_dof]):
            max_steps = steps[i_dof]
            
    # init qtraj
    dur = int(max_steps+post_time_steps)
    qtraj = [ [ None for y in range( dur ) ] for x in range( nb_dofs ) ]
    dqtraj = [ [ None for y in range( dur ) ] for x in range( nb_dofs ) ]
    
     # compute the desired trajectory
    for i_t in range(dur):
        for i_dof in range(nb_dofs):
            # print("dof: ",i_dof," t: ",i_t,"\n ")
            if i_t<=steps[i_dof] and steps[i_dof]>0:
                qtraj[i_dof][i_t] = ((Q_DESIRED[i_dof]-q[i_dof])/(steps[i_dof]))*i_t + q[i_dof]
                dqtraj[i_dof][i_t]= QD_DESIRED[i_dof]# TODO switch depending on direction
            else:
                qtraj[i_dof][i_t] = Q_DESIRED[i_dof]
                dqtraj[i_dof][i_t]= 0
    qtraj = np.asarray(qtraj)
    dqtraj = np.asarray(dqtraj)
    Q_DESIRED = np.asarray(Q_DESIRED)
    
    if not TESTMODE:
        # determine NB_SIM_BURSTS
        # configuration for HYSR
        NB_SIM_BURSTS = int ( (TIME_STEP/self._hysr_config.mujoco_time_step) +0.5 )
        # configuration for accelerated time
        if self._accelerated_time:
            NB_ROBOT_BURSTS = int( 
                (TIME_STEP/hysr_config.o80_pam_time_step) +0.5 
            )

        # configuration for real time
        if not self._accelerated_time:
            frequency_manager = o80.FrequencyManager(1./TIME_STEP)
        
    
    
    now = datetime.now()
    print(dur*TIME_STEP)
    for i_t in range(dur):
        if TESTMODE:
            _,_,q,dq = interface.read()
        else:
            _,_,q,dq = self._pressure_commands.read()
            # mirroing the simulated robot(s)
            for mirroring_ in self._mirrorings:
                mirroring_.set(q, dq)
            self._parallel_burst.burst(NB_SIM_BURSTS)
        # calc errors
        qerr    =   np.asarray(np.asarray(q) - qtraj[:,i_t])
        dqerr   =   np.asarray(np.asarray(dq) - dqtraj[:,i_t])
        qerrsum +=  qerr
        #print(" E: ",qerr," DE: ",dqerr," ESUM: ",qerrsum)

        #apply new controls
        u =  KP*qerr + KD*dqerr + KI*qerrsum
        # print("** u = ",u)
        for i_dof in range(nb_dofs):
            pressures[i_dof]= deltaP(p_min_ago[i_dof],p_max_ago[i_dof], 
                                            p_min_ant[i_dof],p_max_ant[i_dof], 
                                            NDP[i_dof],u[i_dof])
        #print("** pr = ",pressures)
        if TESTMODE:
            #print(pressures)
            t_start=time.time()
            interface.set(pressures,duration_ms=int(TIME_STEP*1000),wait=True)
            print("?",TIME_STEP,time.time()-t_start)
        else:
            # setting the pressures to real robot
            if self._accelerated_time:
                # if accelerated times, running the pseudo real robot iterations
                # (note : o80_pam expected to have started in bursting mode)
                self._pressure_commands.set([p_ago,p_ant], burst=NB_ROBOT_BURSTS)
            else:
                # Should start acting now in the background if not accelerated time
                self._pressure_commands.set([p_ago,p_ant], burst=False)
                frequency_manager.wait()
    print("time :",datetime.now()-now)

    if TESTMODE:
        # plotting
        data = frontend.get_observations_since(start_iteration)   
        axes = fig.subplots(3,4)
        for dof in range(4):
            axis = axes[0,dof]
            plot_angle(axis,data,qtraj,dof)
            axis = axes[1,dof]
            plot_pr(axis,data,dof,0)
            axis = axes[2,dof]
            plot_pr(axis,data,dof,1)
        fig.suptitle(now.strftime("%H:%M:%S"))
        plt.savefig("{}{}.png".format(fdir,now),dpi=450,bbox_inches="tight")
        plt.show(block = True)

    

def _go_to(
        pam_config: pam_interface.Configuration,
        interface:o80_pam.o80Pressures,
        q_desired: Sequence[float], ):
        
    
    """
    Apply a position controller to reach the
    desired position
    """
    

  
    #controlling
    _,_,q_current,_ = interface.read()
    controller = o80_pam.PositionController(q_current,q_desired,QD_DESIRED,pam_config,
                                            KP,KD,KI,NDP,TIME_STEP)
    
    while controller.has_next():
        _,_,q,qd = interface.read()
        pressures = controller.next(q,qd)
        interface.set(pressures,duration_ms=int(TIME_STEP*1e3),wait=True)
 


def _random_posture(interface: o80_pam.o80Pressures,
                    min_pressure: int=14000, max_pressure: int=25000):
    """
    set the robot to a random posture
    """

    posture = [ (random.randrange(min_pressure,max_pressure),
                 random.randrange(min_pressure,max_pressure)) for dof in range(4) ]
    print("going to ",posture)
    interface.set(posture,duration_ms=2000,wait=True)
    
        
def _run(segment_id: str, pam_config_file_path: str):
    """
    runs 3 iterations of 1) robot going to random posture,
    2) robot going to specified Q_DESIRED
    """
    interface,frontend = _get_interface_and_frontend(segment_id)
    pam_config = pam_interface.JsonConfiguration(pam_config_file_path)
    
    
    for _ in range(1):

        print("going to random pressures")
        _random_posture(interface)
        time.sleep(2)
        print("going to target position")
        _go_to_itz(pam_config,interface,Q_DESIRED)
        time.sleep(1)
        print("\t desired:", np.asarray(Q_DESIRED)/math.pi*180 )
        print("\t reached:", np.asarray(interface.read()[2])/math.pi*180)
        print("\t error: ", (np.asarray(Q_DESIRED)-np.asarray(interface.read()[2]))/math.pi*180)
        
    
       

def _config() -> BrightArgs :
    """
    Configuration dialog
    """
    config = BrightArgs("pam position control demo")
    config.add_option("segment_id",
                      "real_robot",
                      "segment_id of the real robot, empty string for simulated robot",
                      str)
    config.add_option("pam_config_file",
                      pam_interface.Pamy2DefaultConfiguration.get_path(),
                      "file specifying the max and min pressures for all muscles",
                      str,
                      [FileExists])
    change_all = False
    config.dialog(change_all)
    print()
    return config


if __name__ == "__main__":

    config = _config()
    _run(config.segment_id,config.pam_config_file)
    
    
