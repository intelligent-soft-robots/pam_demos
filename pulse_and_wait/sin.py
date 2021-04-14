import sys,time,random
import o80,o80_pam
import numpy as np


def sin_all_dofs(frontend,
                 target_pressures,
                 sin_wave,dof_order,
                 idx_multi,
                 step_off_set):

    counter = 0
    for idx in range(0,len(sin_wave)-max(idx_multi),step_off_set):
        
        counter = counter +1

        ago = target_pressures+sin_wave[idx+idx_multi[0]]
        antago = target_pressures-sin_wave[idx+idx_multi[0]]
    
        frontend.add_command(dof_order[0],
                            int(ago),int(antago),o80.Mode.QUEUE)

        ago = target_pressures+sin_wave[idx+idx_multi[1]]
        antago = target_pressures-sin_wave[idx+idx_multi[1]]
        frontend.add_command(dof_order[1],
                              int(ago),int(antago), o80.Mode.QUEUE)
        
        ago = target_pressures+sin_wave[idx+idx_multi[2]]
        antago = target_pressures-sin_wave[idx+idx_multi[2]]

        frontend.add_command(dof_order[2],
                              int(ago),int(antago),o80.Mode.QUEUE)

        ago = target_pressures+sin_wave[idx+idx_multi[3]]
        antago = target_pressures-sin_wave[idx+idx_multi[3]]

        frontend.add_command(dof_order[3],
                              int(ago),int(antago),o80.Mode.QUEUE)

        #print("calling pulse and wait ...")
        frontend.pulse_and_wait()
        #print(".... done")
    
def execute():

    segment_id = "o80_pam_robot"
    min_pressure= 2000
    max_pressure = 18000
    max_off_set_actions = 1000
    amp_action = (max_pressure-min_pressure)/2
    off_set_pressure = ((max_pressure-min_pressure)/2+min_pressure+max_off_set_actions)*0.7
    x = np.arange(-1,1,0.0001)
    frequency = 100 
    idx_multi = [21,37,47,53]
    step_off_set = [3,7,15,21]
    lin_increaser = np.arange(0,1,0.001)

    frontend = o80_pam.FrontEnd(segment_id)
    
    start_time_whole = time.time()
                          
    counter= 0
    for freq in range(1,10):
        print('freq=', freq)

        for phase in range(0,50):
            print('phase=',phase)

            start_phase = time.time()
            for off_set in range(0,max_off_set_actions,100):

                if counter > len(lin_increaser)-2:
                    counter = 0
                else:
                    counter += 1

                sin_wave = np.rint(np.sin(x*freq+phase)*amp_action+off_set)+\
                            lin_increaser[counter]*amp_action/4+\
                            lin_increaser[counter]**2*amp_action/4

                dof_order =[0,1,2,3]
                sin_all_dofs(frontend,
                             off_set_pressure,sin_wave,dof_order,idx_multi,step_off_set[0])

                dof_order =[1,2,3,0]
                sin_all_dofs(frontend,
                             off_set_pressure,sin_wave,dof_order,idx_multi,step_off_set[1]) 

                dof_order =[2,3,0,1]
                sin_all_dofs(frontend,
                             off_set_pressure,sin_wave,dof_order,idx_multi,step_off_set[2]) 

                dof_order =[3,0,1,2]
                sin_all_dofs(frontend,
                             off_set_pressure,sin_wave,dof_order,idx_multi,step_off_set[3])               
                start_pos = random.randint(12000,22000)

                frontend.add_command([start_pos]*4,[start_pos]*4,
                                  o80.Duration_us.seconds(2),
                                  o80.Mode.QUEUE)
                print("calling pulse and wait (2) ...")
                frontend.pulse_and_wait()
                print("... done")

            print('time for phase=', time.time()-start_phase)
            print('whole time needed so far =', time.time()-start_time_whole)
            print('reset')

            print('whole duration=',time.time()-start_time_whole)


execute()


