import json 
import os
from numpy.core.fromnumeric import transpose
import pytransform3d as tr
import numpy as np
from os.path import dirname, abspath, join
import zmq

# object names 
# ['frame_number', 'frame_rate', 'latency', 'my_frame_number', 'num_subjects', 
# 'on_time', 'subjectNames', 'subject_0', 'subject_1', 'subject_2', 'subject_3', 
# 'subject_4', 'subject_5', 'subject_6', 'subject_7', 'subject_8', 'subject_9', 
# 'time_stamp']


class ViconJson:
    def __init__(
        self,
        fname='testViconFrameTableRot.json',#'testViconFrame.txt'
        ip="10.42.2.29",
        port="5555",
        timeout_in_ms=5000
        ):
        self.zmq_connected = False
        self.sub = None
        self.context = None
        self.ip = ip
        self.port = port
        self.timeout_in_ms = timeout_in_ms
        # try connecting to zmq
        self.zmq_connect(self.ip,self.port,self.timeout_in_ms)
        # read frame from file when connection cannot be established
        if self.zmq_connected:
            self.json_obj = self.read_vicon_json_from_zmq()
            print('Vicon connected via zmq')
        else:
            self.json_obj = self.read_vicon_json_from_file(self.get_config_dir()+'/'+fname)
            print('Vicon initialised via test frame from file')
    
    # connecting and reading frames

    def read_vicon_json_from_zmq(self):
        if not self.zmq_connected:
            print('read_vicon_json_from_zmq: connect before reading')
            return []
        else: # read 
            self.json_obj = self.sub.recv_json()
            return self.json_obj
        
    def zmq_connect(self,ip,port,timeout):
        print('zmq_connect: connecting...')
        try:
            self.context = zmq.Context()
            self.sub=self.context.socket(zmq.SUB)
            self.sub.setsockopt(zmq.SUBSCRIBE, b"")
            self.sub.RCVTIMEO = self.timeout_in_ms # wait 5s for new message 
            msg  = 'tcp://'+str(self.ip)+':'+str(port)
            self.sub.connect(msg)  
            if self.sub.closed is True:
                print('zmq_connect(): could not connect')
                return
            # test read frame until frame available or timeout
            n=0
            while n<10 or not self.zmq_connected:
                self.json_obj = self.sub.recv_json()
                if self.json_obj != []:
                    self.zmq_connected = True
                n=n+1
            if self.zmq_connected:
                print('zmq_connect(): connected')
            else:
                print('zmq_connect(): could not read a frame')
                self.zmq_disconnect()
        except Exception as e:
            print('zmq_connect(): could not connect')
            return
        
    def zmq_disconnect(self):
        print('zmq_disconnect: disconnecting...')
        if self.zmq_connected:
            self.context.destroy()
            if self.sub.closed is True:
                self.zmq_connected = False
                print('zmq_disconnect: disconnected. Bye...')
            else:
                print('zmq_disconnect: disconnecting failed!')
        else:
            print('zmq_disconnect: already disconnected. Bye...')

    def read_vicon_json_from_file(self,fname):
        with open(fname) as json_file:
            r = json.load(json_file)
        return r

    def get_config_dir(self):
        return os.path.dirname(__file__)

    # measure distances

    def print_distances(self):
        print('table lengths')
        t_pos_1=self.get_table1_trans()
        t_pos_2=self.get_table2_trans()
        t_pos_3=self.get_table3_trans()
        t_pos_4=self.get_table4_trans()
        print('1->4 ',np.linalg.norm(t_pos_1-t_pos_4))
        print('2->3 ',np.linalg.norm(t_pos_2-t_pos_3))
        print('1->2 ',np.linalg.norm(t_pos_1-t_pos_2))
        print('4->3 ',np.linalg.norm(t_pos_4-t_pos_3))

        print('distance origin -> robot vicon marker')
        t_rob_origin = self.get_rel_transl('rll_ping_base','rll_muscle_base')
        print(t_rob_origin)


    # get object rotations and translations

    def get_table_rot(self):
        t_rot_mat = self.get_table_rot1()
        t_rot_xy_ = np.concatenate((t_rot_mat[0,:].T,t_rot_mat[1,:].T),axis=0)
        print('table rot ',t_rot_mat)
        return np.squeeze(np.asarray(t_rot_xy_))

    def get_robot_rot(self):
        r_rot_mat = self.get_robot_base_rot()
        # rotate robot base vicon frame to robot shoulder frame
        # 1) z axis 180 deg by flipping x & y
        r_tmp = r_rot_mat[1,:].copy()
        r_rot_mat[1,:] = r_rot_mat[0,:].copy()
        r_rot_mat[0,:] = r_tmp.copy()
        # rotate by 90 deg around z
        rot_90z = np.asarray([[0,-1,0],[1,0,0],[0,0,1]])
        r_rot_mat = r_rot_mat @ rot_90z
        # bring into xy_axes form
        r_rot_xy_ = np.concatenate((r_rot_mat[0,:].T,r_rot_mat[1,:].T),axis=0)
        return np.squeeze(np.asarray(r_rot_xy_))

    def get_robot_shoulder_pos(self):
        r_pos=self.get_robot_base_trans()
        tr_to_shoulder = np.asarray([-.255,.0785,0]) # measured on real robot
        return r_pos+tr_to_shoulder

    def get_table_pos(self):
        t_pos_1=self.get_table1_trans()
        t_pos_2=self.get_table2_trans()
        t_pos_3=self.get_table3_trans()
        t_pos_4=self.get_table4_trans()
        t = (t_pos_1+t_pos_2+t_pos_3+t_pos_4)/4
        return t

    # translations

    def get_rel_transl(self,key_origin,key_target):
        T_origin = self.get_transl(key_origin)
        T_target = self.get_transl(key_target)
        return ( np.asarray(T_target)-np.asarray(T_origin) )/1000

    def get_robot_base_trans(self):
        return self.get_rel_transl('rll_ping_base','rll_muscle_base')

    def get_table1_trans(self):
        return self.get_rel_transl('rll_ping_base','TT Platte_Eckteil 1')
    def get_table2_trans(self):
        return self.get_rel_transl('rll_ping_base','TT Platte_Eckteil 2')
    def get_table3_trans(self):
        return self.get_rel_transl('rll_ping_base','TT Platte_Eckteil 3')
    def get_table4_trans(self):
        return self.get_rel_transl('rll_ping_base','TT Platte_Eckteil 4')

    # rotations

    def get_rel_rot_mat(self,key_origin,key_target):
        R_mat_origin = self.get_rot_mat(key_origin)
        R_mat_target = self.get_rot_mat(key_target)
        return np.asarray(R_mat_target).T @ np.asarray(R_mat_origin)

    def get_robot_base_rot(self):
        return self.get_rel_rot_mat('rll_ping_base','rll_muscle_base')

    def get_table_rot1(self):
        return self.get_rel_rot_mat('rll_ping_base','TT Platte_Eckteil 1')
    def get_table_rot2(self):
        return self.get_rel_rot_mat('rll_ping_base','TT Platte_Eckteil 2')
    def get_table_rot3(self):
        return self.get_rel_rot_mat('rll_ping_base','TT Platte_Eckteil 3')
    def get_table_rot4(self):
        return self.get_rel_rot_mat('rll_ping_base','TT Platte_Eckteil 4')

    # access json methods

    def get_transl(self,key):
        idx = self.json_obj['subjectNames'].index(key)
        tr =  self.json_obj['subject_'+str(idx)]['global_translation'][0]
        return tr

    def get_rot_mat(self,key):
        idx = self.json_obj['subjectNames'].index(key)
        r =  self.json_obj['subject_'+str(idx)]['global_rotation']["matrix"][0]
        return r

    def get_rot_eulerxyz(self,key):
        idx = self.json_obj['subjectNames'].index(key)
        r =  self.json_obj['subject_'+str(idx)]['global_rotation']["eulerxyz"][0]
        return r

    def get_rot_quat(self,key):
        idx = self.json_obj['subjectNames'].index(key)
        r =  self.json_obj['subject_'+str(idx)]['global_rotation']["quaternion"][0]
        return r    