# TODOs

# 1 read vicon json from file (will be later exchanged through reading from stream using zmq)
# 2 express robot frame and table frame in terms of F_0 (marker at ping)

import json 
import os
from numpy.core.fromnumeric import transpose
import pytransform3d as tr
import numpy as np

# ['frame_number', 'frame_rate', 'latency', 'my_frame_number', 'num_subjects', 
# 'on_time', 'subjectNames', 'subject_0', 'subject_1', 'subject_2', 'subject_3', 
# 'subject_4', 'subject_5', 'subject_6', 'subject_7', 'subject_8', 'subject_9', 
# 'time_stamp']


class ViconJson:
    def __init__(
        self,
        fname='testViconFrame.txt'#'testViconFrameTableRot.txt'
        ):
        self.j = self.read_vicon_json_from_file(fname)
        
    def read_vicon_json_from_file(self,fname):
        script_dir = os.path.dirname(__file__)
        with open(os.path.join(script_dir, fname)) as json_file:
            r = json.load(json_file)
        return r

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

    # tbd
    def read_vicon_json(self):
        r={}
        return r
    