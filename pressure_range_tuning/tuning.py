#!/usr/bin/env python3

import logging
import sys
import time
import math
import o80
import o80_pam
import matplotlib.pyplot as plt
import numpy as np
from lightargs import BrightArgs,Set,Range,Positive
from datetime import datetime


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
    
def plot_angle(axis,data,dof):
    # plot data (data as returned
    # by reach target above)
    
    angle = [d.get_positions()[dof]*180/math.pi
                for d in data]
    axis.set_title("dof {} | max {:.2f} | min {:.2f} ".format(dof,max(angle),min(angle))) 
    iterations = list(range(len(data)))
    axis.plot(iterations,angle,c='blue')
    axis.grid(True)

def reset(config,frontend):
    '''    
    p_min_ago=[13000,14000,13000,10000]
    p_min_ant=[13000,15000,13000,10000]
    p_max_ago=[25000,21500,21000,20000]
    p_max_ant=[25000,25000,21000,20000]
    '''
    p_min_ago=[15000,15000,13000,10000]
    p_min_ant=[15000,17000,13000,10000]
    p_max_ago=[25000,21500,21000,20000]
    p_max_ant=[25000,25000,21000,20000]
    '''
    18160 19840
    17750 20000
    17080 16920
    20000 20000
    '''
    p_ago=[18160,17000,17080,19000]
    p_ant=[19250,20000,15000,20000]
    r_ago=np.array([0,0,0,0])
    r_ant=np.array([0,0,0,0])
    for _ in range(1):
        now = datetime.now()
        fig = plt.figure()
        start_iteration = frontend.latest().get_iteration()
        duration = o80.Duration_us.milliseconds(1000)
        # for dof in range(4):
        #     i_m_ago = 2*dof # muscle index ago and antago
        #     i_m_ant = 2*dof+1
        #     r_ago[dof]  = np.random.uniform(p_min_ago[dof],p_max_ago[dof])
        #     r_ant[dof]  = np.random.uniform(p_min_ant[dof],p_max_ant[dof])
        #     frontend.add_command(i_m_ago,int(r_ago[dof]),duration,o80.Mode.OVERWRITE)
        #     frontend.add_command(i_m_ant,int(r_ant[dof]),duration,o80.Mode.OVERWRITE)
        frontend.add_command(0,p_max_ago[0],duration,o80.Mode.OVERWRITE)
        frontend.add_command(1,p_min_ant[0],duration,o80.Mode.OVERWRITE)
        frontend.add_command(2,p_min_ago[1],duration,o80.Mode.OVERWRITE)
        frontend.add_command(3,p_max_ant[1],duration,o80.Mode.OVERWRITE)
        frontend.add_command(4,p_min_ago[2],duration,o80.Mode.OVERWRITE)
        frontend.add_command(5,p_max_ant[2],duration,o80.Mode.OVERWRITE)
        frontend.add_command(6,p_min_ago[3],duration,o80.Mode.OVERWRITE)
        frontend.add_command(7,p_max_ant[3],duration,o80.Mode.OVERWRITE)
        frontend.pulse_and_wait()
        time.sleep(0.5)
        duration = o80.Duration_us.milliseconds(450)
        for dof in range(4):
            i_m_ago = 2*dof # muscle index ago and antago
            i_m_ant = 2*dof+1
            frontend.add_command(i_m_ago,p_min_ago[dof],duration,o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_max_ant[dof],duration,o80.Mode.OVERWRITE)
        frontend.pulse_and_wait()

        time.sleep(1.5)
        duration = o80.Duration_us.milliseconds(2000)
        for dof in range(4):
            i_m_ago = 2*dof # muscle index ago and antago
            i_m_ant = 2*dof+1
            frontend.add_command(i_m_ago,p_max_ago[dof],duration,o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_min_ant[dof],duration,o80.Mode.OVERWRITE)
        frontend.pulse_and_wait()
        #time.sleep(1.5)
        duration = o80.Duration_us.milliseconds(1000)
        for dof in range(4):
            i_m_ago = 2*dof # muscle index ago and antago
            i_m_ant = 2*dof+1
            frontend.add_command(i_m_ago,p_max_ago[dof],duration,o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_max_ant[dof],duration,o80.Mode.OVERWRITE)
        frontend.pulse_and_wait()
        duration = o80.Duration_us.milliseconds(2000)
        for dof in range(4):
            i_m_ago = 2*dof # muscle index ago and antago
            i_m_ant = 2*dof+1
            if dof == 0:
                frontend.add_command(i_m_ago,p_ago[dof],duration,o80.Mode.OVERWRITE)
                frontend.add_command(i_m_ant,p_ant[dof],duration,o80.Mode.OVERWRITE)
                #print(int((p_max_ago[dof]-p_min_ago[dof])*.43+p_min_ago[dof]),
                      #int((p_max_ant[dof]-p_min_ant[dof])*.57+p_min_ant[dof]))            
            elif dof == 1:
                frontend.add_command(i_m_ago,p_ago[dof],duration,o80.Mode.OVERWRITE)
                frontend.add_command(i_m_ant,p_ant[dof],duration,o80.Mode.OVERWRITE)
                #print(int((p_max_ago[dof]-p_min_ago[dof])*.5+p_min_ago[dof]),
                      #int((p_max_ant[dof]-p_min_ant[dof])*.5+p_min_ant[dof]))              
            elif dof == 2:
                frontend.add_command(i_m_ago,p_ago[dof],duration,o80.Mode.OVERWRITE)
                frontend.add_command(i_m_ant,p_ant[dof],duration,o80.Mode.OVERWRITE)
                #print(int((p_max_ago[dof]-p_min_ago[dof])*.51+p_min_ago[dof]),
                      #int((p_max_ant[dof]-p_min_ant[dof])*.49+p_min_ant[dof]))              
            elif dof == 3:
                frontend.add_command(i_m_ago,p_ago[dof],duration,o80.Mode.OVERWRITE)
                frontend.add_command(i_m_ant,p_ant[dof],duration,o80.Mode.OVERWRITE)
                #print(p_max_ago[dof],
                      #p_max_ant[dof]) 
            
        frontend.pulse_and_wait()
        
        time.sleep(1.5)
        data = frontend.get_observations_since(start_iteration)
        obs = frontend.latest()
        later = datetime.now()
        #print(later-now)
        print(r_ago,r_ant,["{:.2f}".format(d/math.pi*180) for d in obs.get_positions()])
        
        
        
        # plotting
        logging.info("plotting")  
        axes = fig.subplots(3,4)
        for dof in range(4):
            plot_angle(axes[0][dof],data,dof)
            plot_pr(axes[1][dof],data,dof,0)
            plot_pr(axes[2][dof],data,dof,1)
    #plt.show(block = True)

def run(config,frontend):
    for _ in range(3):
        log_handler = logging.StreamHandler(sys.stdout)
        logging.basicConfig(
            format="[o80 check segment_id: {}] %(message)s".format(config.segment_id),
            level=logging.DEBUG,
            handlers=[log_handler]
        )
        
        dof = 0
        sign = 0
        
        p_min_ago=[15000,15000,13000,10000]
        p_min_ant=[17000,17000,13000,10000]
        p_max_ago=[25000,21500,21000,20000]
        p_max_ant=[25000,25000,21000,20000]

        all_data = {dof:[None,None] for dof in range(config.nb_dofs)}
        
        fdir = "/home/vberenz/Desktop/pressure_tuning/"
        
        # init to high pressure vals
        for d in range(4):
            for s in range(2):
                frontend.add_command(2*d+s,20000,o80.Mode.OVERWRITE)
                frontend.pulse()
                time.sleep(.1)
      
        #start
        fig = plt.figure()
        now = datetime.now() 
        logging.info("checking dof: {}".format(dof))
            
        start_iteration = frontend.latest().get_iteration()
        # set start position
        i_m_ago = 2*dof # muscle index ago and antago
        i_m_ant = 2*dof+1
        duration = o80.Duration_us.milliseconds(2000)
        if sign==0:
            frontend.add_command(i_m_ago,p_min_ago[dof],duration,o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_max_ant[dof],duration,o80.Mode.OVERWRITE)
        else:
            frontend.add_command(i_m_ago,p_max_ago[dof],duration,o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_min_ant[dof],duration,o80.Mode.OVERWRITE)
        frontend.pulse_and_wait()
        time.sleep(1)
        # execute fast motion
        #input("starting fast motion. Press enter to continue...")
        if sign==0:
            frontend.add_command(i_m_ago,p_max_ago[dof],o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_min_ant[dof],o80.Mode.OVERWRITE)
        else:
            frontend.add_command(i_m_ago,p_min_ago[dof],o80.Mode.OVERWRITE)
            frontend.add_command(i_m_ant,p_max_ant[dof],o80.Mode.OVERWRITE)
        frontend.pulse()
        time.sleep(2)

        data = frontend.get_observations_since(start_iteration)
        
        
        # plotting
        logging.info("plotting")  
        axes = fig.subplots(3,1)
        plot_angle(axes[0],data,dof)
        plot_pr(axes[1],data,dof,sign)
        plot_pr(axes[2],data,dof,1-sign)
        # fig.suptitle("{}\n p_min_ago{} \n p_min_ant{} \n p_max_ago{} \n p_max_ant{} \n".format(now.strftime("%H:%M:%S"),p_min_ago,p_min_ant,p_max_ago,p_max_ant))
        plt.savefig("{}{}.png".format(fdir,now),dpi=450,bbox_inches="tight")
    # plt.show(block = True)
    
def _get_frontend(config):
    try :
        frontend = o80_pam.FrontEnd(config.segment_id)
        return frontend
    except Exception as e :
        return None


def _configure():
    config = BrightArgs("o80 PAM console")
    config.add_option("segment_id",
                      o80_pam.segment_ids.robot,
                      "o80 segment_id of the instance of o80 pam to display",
                      str)
    config.add_option("nb_dofs",
                      4,
                      "number of degrees of freedom of the robots",
                      int,
                      integrity_checks=[Positive()])
    change_all=False
    finished = config.dialog(change_all,sys.argv[1:])
    if not finished:
        return None
    return config
    

def execute():

    #config = _configure()
    #if config is None:
    #    return

    class Config:
        pass
    config = Config()
    config.segment_id = "real_robot"
    config.nb_dofs = 4

    frontend = _get_frontend(config)
    if frontend is None:
        print(str("\nfailed to start an o80 frontend on segment_id: {}."+
                  "Is a corresponding robot running ?\n").format(config.segment_id))
    else:
        print()
        reset(config,frontend)
        #run(config,frontend)
        print()


if __name__ == "__main__":

    execute()
