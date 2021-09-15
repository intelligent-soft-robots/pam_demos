import json 
import os
from numpy.core.fromnumeric import transpose
import pytransform3d as tr
import numpy as np
from os.path import dirname, abspath, join
import zmq
import json

# object names 
# ['frame_number', 'frame_rate', 'latency', 'my_frame_number', 'num_subjects', 
# 'on_time', 'subjectNames', 'subject_0', 'subject_1', 'subject_2', 'subject_3', 
# 'subject_4', 'subject_5', 'subject_6', 'subject_7', 'subject_8', 'subject_9', 
# 'time_stamp']


class ViconJson:
    def __init__(
        self,
        fname='testViconFrameTableRot.txt',#'testViconFrame.txt'
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
            self.j = self.read_vicon_json_from_zmq()
            print('Vicon connected via zmq')
        else:
            self.j = self.read_vicon_json_from_file(fname)
            print('Vicon initialised via test frame from file')
    
    # connecting and reading frames

    def read_vicon_json_from_zmq(self):
        if self.zmq_connected is False:
            print('read_vicon_json_from_zmq: connect before reading')
            return []
        else: # read 
            self.j = self.sub.recv_json()
            return self.j
        
    def zmq_connect(self,ip,port,timeout):
        print('zmq_connect: connecting...')
        try:
            self.context = zmq.Context()
            self.sub=self.context.socket(zmq.SUB)
            self.sub.setsockopt(zmq.SUBSCRIBE, b"")
            self.sub.RCVTIMEO = self.timeout_in_ms # wait only 5s for new message 
            msg  = 'tcp://'+str(self.ip)+':'+str(port)
            self.sub.connect(msg)  
            if self.sub.closed is True:
                print('zmq_connect(): could not connect')
                return
            # test read frame until frame available or timeout
            n=0
            while n<10 or not self.zmq_connected:
                self.j = self.sub.recv_json()
                if self.j is not []:
                    self.zmq_connected = True
                n=n+1
            if n<10:
                print('zmq_connect(): could not read a frame')
                self.zmq_disconnect()
            else:
                print('zmq_connect(): connected')
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
        script_dir = os.path.dirname(__file__)
        with open(os.path.join(script_dir, fname)) as json_file:
            r = json.load(json_file)
        return r

    # get object rotations and translations

    def get_table_rot(self):
        t_rot_mat = self.get_table_rot1()
        t_rot_xy_ = np.concatenate((t_rot_mat[0,:].T,t_rot_mat[1,:].T),axis=0)
        return np.squeeze(np.asarray(t_rot_xy_))

    def get_robot_rot(self):
        r_rot_mat = self.get_robot_base_rot()
        # rotate robot base vicon frame to robot shoulder frame
        # 1) z axis 180 deg by flipping x & y
        r_tmp = r_rot_mat[1,:].copy()
        r_rot_mat[1,:] = r_rot_mat[0,:].copy()
        r_rot_mat[0,:] = r_tmp.copy()
        # bring into xy_axes form
        r_rot_xy_ = np.concatenate((r_rot_mat[0,:].T,r_rot_mat[1,:].T),axis=0)
        return np.squeeze(np.asarray(r_rot_xy_))

    def get_robot_shoulder_pos(self):
        r_pos=self.get_robot_base_trans()
        tr_to_shoulder = np.asarray([.25,.075,0])
        return r_pos+tr_to_shoulder

    def get_table_pos(self):
        t_pos_1=self.get_table1_trans()
        t_pos_2=self.get_table2_trans()
        t_pos_3=self.get_table3_trans()
        t_pos_4=self.get_table4_trans()
        t_x = (t_pos_1[0]+t_pos_2[0]+t_pos_3[0]+t_pos_4[0])/4
        t_y = (t_pos_1[1]+t_pos_2[1]+t_pos_3[1]+t_pos_4[1])/4
        t_z = (t_pos_1[2]+t_pos_2[2]+t_pos_3[2]+t_pos_4[2])/4
        return np.asarray([t_x,t_y,t_z])

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
        return np.asmatrix(R_mat_target).T*np.asmatrix(R_mat_origin)

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
        idx = self.j['subjectNames'].index(key)
        tr =  self.j['subject_'+str(idx)]['global_translation'][0]
        return tr

    def get_rot_mat(self,key):
        idx = self.j['subjectNames'].index(key)
        r =  self.j['subject_'+str(idx)]['global_rotation']["matrix"][0]
        return r

    def get_rot_eulerxyz(self,key):
        idx = self.j['subjectNames'].index(key)
        r =  self.j['subject_'+str(idx)]['global_rotation']["eulerxyz"][0]
        return r

    def get_rot_quat(self,key):
        idx = self.j['subjectNames'].index(key)
        r =  self.j['subject_'+str(idx)]['global_rotation']["quaternion"][0]
        return r    