''' This program is to define UR robot with Robotiq gripper and camera '''
#import time
from PyQt5.QtCore import QObject

#import numpy as np
import logging
import math
import os
import sys
sys.path.append('..')
from urxe.robot import Robotiq_Two_Finger_Gripper
from urxe.robot import Robot
from utils.urdashboard import dashboard
from utils.urcamera import camera
from urxe import ursecmon
text_file_path = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(text_file_path, '..', 'urscripts', 'checkdistance.script'), 'r') as file:
    CheckdistanceScript = file.read()

#from urrobot import URRobot

#### Standard orientations.
# orient the tool point -Z axis of the base and keep the y aixs the same with y of base coordinate.
# Thus, TCP coordinate is the y rotated based coordinate system.
m3d_Zdown_cameraY = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
# orient the tool point -Z axis of the base and swap the xy (so that y of TCP ponts y of base) 
# Thus, TCP coordinate is .
m3d_Zdown_cameraX = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
m3d_Zdown_cameraXm = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
m3d_Zdown_cameraYm = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
# you can make "pose" object, by
# orient = m3d.Orientation(m3d_Zdown_cameraY)
# t = rob.robot.get_pose()
# t.orient = orient
 
######## How to use m3d.
# To rotate in the TCP frame,
# trans = self.robot.get_pose()  # here trans represents the transformed TCP coordinate.
# To rate in the robot base frame,
# trans = m3d.Transform()  # make a new m3d object, 
# Then, trans.orient.rotate_xt(), rotate_yt(), rotate_zt(), or rotate_t(ax, angle)

# Define your own robot to include camera and dashboard....
# edit all these basic functions to work.
class UR_cam_grip(QObject):
    # unit of position vector : meter.
    _TCP2CAMdistance = 0.12
    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]
    camtcp = [0, 0.04, 0.015, -math.pi/180*30, 0, 0]

    def __init__(self, name = 'UR3', fingertype=1, cameratype=1):
        super().__init__()
        # fingertype:
        #   0: No finger
        #   1: Robotiq finger
        # cameratype :
        #   0: No camera
        #   1: IP camera
        #   2: USB camera
        
        if '.' in name:
            IP = name
        else:
            raise ValueError('IP address should be given.')

        self.logger = logging.getLogger(IP)
        
        self.orientation = m3d_Zdown_cameraY

        try:
#            self.robot = urx.Robot(IP)
            self.robot = Robot(IP, use_rt=True, urFirm=(5.9))
        except TimeoutError:
            raise RobotException(f'Robot {IP} does not respond.')
        except ursecmon.ProtectiveStopException:
            print("Protective stoppped.. Connecting again.")
            self.robot = Robot(IP)

        if cameratype==2:
            self.camera = camera(IP='')
        elif cameratype==1:
            self.camera = camera(IP)
        else:
            pass

        if fingertype==1:
            self.finger = Robotiq_Two_Finger_Gripper(self.robot)
        else:
            pass

        #self.__TCP2CAMdistance = 0.15
        self.robot.IP = IP
        self.set_tcp(self.tcp)
        self.set_payload(1.35, (-0.003,0.01,0.037))
        self.dashboard = dashboard(self.robot)
#        if self.robot.secmon.is_protective_stopped():
#            self.dashboard.unlock()
        #self.finger.gripper_activate()
# Make this common function to work.

    def terminate(self):
        self.robot.close()
        try:
            if hasattr(self, 'camera'):
                self.camera.vidcap.release()
        except:
            pass

    def set_payload(self, *args, **kwargs):
        self.robot.set_payload(*args, **kwargs)

    def get_tcp(self):
        pose = self.robot.get_tcp()
        return pose

    def set_tcp(self, tcp):
        return self.robot.set_tcp(tcp)

    def getj(self):
        joints = self.robot.getj()
        return joints    
    
    def get_pos(self):
        return self.robot.get_pos()
    
    def get_pose(self):
        return self.robot.get_pose()

    def set_pose(self, *args, **kwargs):
        return self.robot.set_pose(*args, **kwargs)
    
    def movels(self, *args, **kwargs):
        return self.robot.movels(*args, **kwargs)
    
    def movel(self, *args, **kwargs):
        return self.robot.movel(*args, **kwargs)
    
    def movej(self, *args, **kwargs):
        return self.robot.movej(*args, **kwargs)
    
    def get_status(self):
        return self.dashboard.get_status()

    def unlock_stop(self):
        self.dashboard.unlock()

    def is_protective_stopped(self):
        return self.robot.secmon.is_protective_stopped()   

    def is_running(self):
        return self.robot.is_program_running()

    def set_orientation_rad(self, *args, **kwargs):
        self.robot.set_orientation(*args, **kwargs)
    
    def add_pose_base(self, *args, **kwargs):
        return self.robot.add_pose_base(*args, **kwargs)
    
    def translate(self, *args, **kwargs):
        return self.robot.translate(*args, **kwargs)
    
    def translate_tool(self, *args, **kwargs):
        return self.robot.translate_tool(*args, **kwargs)

    def bump(self, *args, **kwargs):
        self.robot.bump(*args, **kwargs)

