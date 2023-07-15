''' This program is to define UR robot with Robotiq gripper and camera '''
#import time
import sys
sys.path.append('..')
#from urxe import camera
from PyQt5.QtCore import QObject

import numpy as np

class RobotException(Exception):
    pass

class NoCameraException(Exception):
    pass

class NoFingerException(Exception):
    pass

# UR3
import math3d as m3d
#import numpy as np
import logging
import math
import time
import os

text_file_path = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(text_file_path, '..', 'urscripts', 'checkdistance.script'), 'r') as file:
    CheckdistanceScript = file.read()

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

import rtde_control as rc
import rtde_receive as rr
#import rtde_io as rio
from .robotiq_gripper_control import RobotiqGripper
from utils.urcamera import camera
from utils.urdashboard import dashboard
######## How to use m3d.
# To rotate in the TCP frame,
# trans = self.robot.get_pose()  # here trans represents the transformed TCP coordinate.
# To rate in the robot base frame,
# trans = m3d.Transform()  # make a new m3d object, 

# Then, trans.orient.rotate_xt(), rotate_yt(), rotate_zt(), or rotate_t(ax, angle)

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
        
        self.csys = m3d.Transform()

        if '.' in name:
            IP = name
        else:
            raise ValueError('IP address should be given.')

        self.logger = logging.getLogger(IP)
        
        self.orientation = m3d_Zdown_cameraY

        self.rc = rc.RTDEControlInterface(IP)
        self.rr = rr.RTDEReceiveInterface(IP)
        #self.rio = rio.RTDEIOInterface(IP)

        if cameratype==2:
            self.camera = camera(IP='')
        elif cameratype==1:
            self.camera = camera(IP)
        else:
            pass

        if fingertype==1:
            self.finger = RobotiqGripper(self.rc)
        else:
            pass

        #self.__TCP2CAMdistance = 0.15
        self.IP = IP
        self.set_tcp(self.tcp)
        self.set_payload(1.35, (-0.003,0.01,0.037))
        self.dashboard = dashboard(self.IP)

    def terminate(self):
        self.rc.stopScript()
        self.rc.disconnect()
        self.rr.disconnect()
#        self.rio.disconnect()
        try:
            if hasattr(self, 'camera'):
                self.camera.vidcap.release()
        except:
            pass

    def set_csys(self, transform):
        """
        Set reference coordinate system to use
        """
        self.csys = transform

    def get_tcp(self):
        pose = self.rc.getTCPOffset()
        return pose

    def set_payload(self, weight, dir):
        return self.rc.setPayload(weight, dir)

    def set_tcp(self, tcp):
        return self.rc.setTcp(tcp)

    def bump(self, x=0, y=0, z=0, backoff=0, wait=True):
        #data = CheckdistanceScript
        data = CheckdistanceScript.replace('__replace__', f'[{x}, {y}, {z}, 0, 0, 0]')
        data = data.replace('__backoff__', f'{backoff}')
        self.rc.sendCustomScript(data)
        while not self.rr.is_program_running():
            time.sleep(0.01)
        if wait:
            while self.rr.is_program_running():
                time.sleep(0.01)
        self.rc.stopScript()

    def getj(self):
        joints = self.rr.getActualQ()
        return joints    
    
    def getl(self):
        return self.rr.getActualTCPPose()
    
    def get_pos(self):
        pos = self.rr.getActualTCPPose()
        return m3d.Vector(pos[0:3])
    
    def get_pose(self):
        pose = self.getl()
        trans = self.csys.inverse * m3d.Transform(pose)
        return trans

    def set_pose(self, trans, acc=0.01, vel=0.01, wait=True):
        t = self.csys * trans
        self.movel(t.pose_vector.tolist(), acc=acc, vel=vel, wait=wait, relative=False)
        pose = self.get_pose()
        if pose is not None:
            return self.csys.inverse * m3d.Transform(pose)
    
    def movels(self, path, wait=True):
        # with tool poses that includes acceleration, speed and blend for each position
        # https://sdurobotics.gitlab.io/ur_rtde/api/api.html#rtde-control-interface-api
        self.rc.moveL(path, asynchronous=not wait)
        if wait:
            pose = self.get_pose()
            if pose is not None:
                    return self.csys.inverse * m3d.Transform(pose)
    
    def movel(self, pos_list, acc=0.01, vel=0.01, wait = True, relative=False):
        l = self.getl()
        if relative:
            tpose = [v + l[i] for i, v in enumerate(pos_list)]
        else:
            tpose = pos_list
            if len(pos_list)<6:
                tpose.append(l[3])
                tpose.append(l[4])
                tpose.append(l[5])
        print(tpose)
        self.rc.moveL(tpose, acceleration=acc, speed=vel, asynchronous=not wait)
        if wait:
            pose = self.get_pose()
            if pose is not None:
                    return self.csys.inverse * m3d.Transform(pose)

    
    def movej(self, q, vel=0.1, acc=0.1, wait=True):
        self.rc.moveJ(q, acceleration=acc, speed=vel, asynchronous=not wait)
    
    def get_status(self):
        return self.dashboard.get_status()

    def unlock_stop(self):
        self.dashboard.unlock()

    def is_protective_stopped(self):
        return self.rr.isProtectiveStopped()   

    def is_running(self):
        return self.rc.isProgramRunning()

    def set_orientation_rad(self, orient, acc=0.01, vel=0.01, wait=True):
        if not isinstance(orient, m3d.Orientation):
            orient = m3d.Orientation(orient)
        trans = self.get_pose()
        trans.orient = orient
        self.set_pose(trans, acc, vel, wait=wait)
    
    def add_pose_base(self, trans, acc=0.1, vel=0.1, wait=True):
        """
        Add transform expressed in base coordinate
        """
        pose = self.get_pose()
        return self.set_pose(trans * pose, acc, vel, wait=wait)
    
    def add_pose_tool(self, trans, acc=0.01, vel=0.01, wait=True):
        """
        Add transform expressed in tool coordinate
        """
        pose = self.get_pose()
        return self.set_pose(pose * trans, acc, vel, wait=wait)
    
    def translate(self, vect, acc=0.01, vel=0.01, wait=True):
        """
        move tool in base coordinate, keeping orientation
        """
        p = self.getl()
        p[0] += vect[0]
        p[1] += vect[1]
        p[2] += vect[2]
        return self.movel(p, vel=vel, acc=acc, relative=False, wait=wait)
    
    def translate_tool(self, vect, acc=0.01, vel=0.01, wait=True):
        """
        move tool in tool coordinate, keeping orientation
        """
        t = m3d.Transform()
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        t.pos += vect
        return self.add_pose_tool(t, acc, vel, wait=wait)

    def bump2(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, vel=0.01):
        speed = [x, y, z, rx, ry, rz]
        for i in range(len(speed)):
            speed[i] = speed[i] * vel
        self.rc.moveUntilContact(speed)
        self.rc.stopScript()
