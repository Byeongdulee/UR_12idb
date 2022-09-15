from PyQt5.QtCore import (pyqtSignal, QObject)

import time
import numpy

class RobotException(Exception):
    pass

# UR3
import os
import sys

import math3d as m3d
import numpy as np
import logging
import math
import time
from robot import Robotiq_Two_Finger_Gripper
from robot import Robot
from urdashboard import dashboard
from urcamera import camera
import camera_tools as ctool
import robot
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

def ind2sub(ind, array_shape):
    rows = int(ind / array_shape[1])
    cols = (int(ind) % array_shape[1]) # or numpy.mod(ind.astype('int'), array_shape[1])
    return (rows, cols)

def sub2ind(rows, cols, array_shape):
    return rows*array_shape[1] + cols

class UR3(QObject):
    # unit of position vector : meter.
    sigFinger = pyqtSignal(str)
    sigMoving = pyqtSignal(bool)
    sigFingerPosition = pyqtSignal(str)
    sigObject_onFinger = pyqtSignal(bool)
    sigRobotCommand = pyqtSignal(str)
    sigRobotPosition = pyqtSignal(numpy.ndarray)
    _TCP2CAMdistance = 0.12
    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]
    camtcp = [0, 0.04, 0.015, -math.pi/180*30, 0, 0]

    def __init__(self, name = 'UR3', finger=True):
        super(UR3, self).__init__()
# definition of Cartesian Axis of UR3 at 12idb.
# X : positive - Out board
# X : negative - In board
# Y : positive - Along X-ray
# Y : negative - Again X-ray
        if '.' in name:
            IP = name
        else:
            if name == 'UR3':
                IP="164.54.122.96"
            if name == 'UR5':
                IP = 'UR5-12IDC.xray.aps.anl.gov'
        self.logger = logging.getLogger(name)
        try:
#            self.robot = urx.Robot(IP)
            self.robot = Robot(IP)
        except TimeoutError:
            exit
        except robot.urrobot.ursecmon.ProtectiveStopException:
            print("Protective stoppped.. Connecting again.")
            self.robot = Robot(IP)
        if name == 'UR5':
            self.camera = camera(IP='')
        else:
            self.camera = camera(IP)
        if finger:
            self.finger = Robotiq_Two_Finger_Gripper(self.robot)
        else:
            self.finger = None
        #self.__TCP2CAMdistance = 0.15
        self.robot.IP = IP
        self.robot.set_tcp(self.tcp)
        self.robot.set_payload(1.35, (-0.003,0.01,0.037))
        self.dashboard = dashboard(self.robot)
        if self.robot.secmon.is_protective_stopped():
            self.dashboard.unlock()
        #self.finger.gripper_activate()
        self.name = name

    def terminate(self):
        self.robot.close()
        try:
            self.camera.vidcap.release()
        except:
            pass

    def get_status(self):
        return self.dashboard.get_status()

    def unlock_stop(self):
        self.dashboard.unlock()

    def is_protective_stopped(self):
        return self.robot.is_protective_stopped()   

    def is_running(self):
        return self.robot.is_program_running()

# check positions and orientations

    def get_orientation(self):
        # get rotation_vector.
        #v = self.robot.get_orientation().to_euler('xyz')*180/math.pi
        #return v.tolist()
        m = self.robot.get_orientation().get_rotation_vector().tolist()
        for i in range(0,2):
            m[i] = m[i]*180/math.pi
        return m

    def set_orientation(self, *ang):
        # set_orientation()
        # set_orientation(orientation_matrix)
        # set_orientation(0, -180, 0)
        if len(ang) < 2:
            if len(ang) ==0:
                orient = m3d.Orientation(m3d_Zdown_cameraY)
            elif len(ang) == 1: 
                orient = m3d.Orientation(ang[0])
            t = self.robot.get_pose()
            t.orient = orient
            self.robot.set_pose(t, 0.5, 0.5, wait=True)
            return t
        myang = [0, -180, 0]
        if len(ang) != 0:
            for i in range(len(ang)):
                myang[i] = ang[i]
        # set rotation_vector
        for i in range(0,2):
            myang[i] = myang[i]*math.pi/180
        #v0 = self.get_xyz().tolist()
        #v0 = [v0[0], v0[1], v0[2], myanglearray[0],myanglearray[1],myanglearray[2]]
        return self.robot.set_orientation(myang, acc=1, vel=0.5)
        #np0 = m3d.Transform(v0)
        #return self.robot.movel(np0, acc=0.5, vel=0.5)

    def get_xyz(self):
        # return the Cartesian position as a numpy array
        # if you want the Cartesian position as a list, use self.robot.getl()
        pose = self.robot.get_pose()
        return pose.get_pose_vector()

    def getj(self):
        joints = self.robot.getj()
        return joints

    def is_Z_aligned(self):
        v = self.robot.get_pose()
        vec = v.orient.vec_z
        if abs(vec[2]) < 0.999:
            return False
        else:
            return True

    def rotz_camera2x(self):
        v = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        t = m3d.Orientation(v)
        t.set_pos(self.robot.get_pos())
        self.robot.set_pose(t, 0.5, 0.5, wait=True)

    def rotz_camera2y(self):
        v = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        t = m3d.Orientation(v)
        t.set_pos(self.robot.get_pos())
        self.robot.set_pose(t, 0.5, 0.5, wait=True)
        
# Linear motions
    # absolute positinng
    def mvx(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[0] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def mvy(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[1] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def mvz(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[2] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def mvrx(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[3] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def mvry(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[4] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def mvrz(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[5] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def mv(self, pos, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        if pos[0] != None:
            v[0] = pos[0]
        if pos[1] != None:
            v[1] = pos[1]
        if pos[2] != None:
            v[2] = pos[2]        
        self.moveto(v, acc=acc, vel=vel, wait=wait)
    
    def movels(self, pos_list, radius = 0.01, acc=0.5, vel=0.5, wait=True):
        " movels([0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [0.2, 0.3, 0.2]], radius=0.1)"
        v = self.get_xyz().tolist()
        for i, vec in enumerate(pos_list):
            if len(vec) == 3:
                pos_list[i].append(v[3])
                pos_list[i].append(v[4])
                pos_list[i].append(v[5])
        pose = self.robot.movels(pos_list, acc=acc, vel=vel, radius=radius, wait=wait)
        return pose

    def moveto(self, position, command="movel", acc=0.5, vel=0.5, wait=True):
        if type(position) == list or numpy.ndarray:
            if len(position) == 3:
                v = self.get_xyz().tolist()
                position.append(v[3])
                position.append(v[4])
                position.append(v[5])
        if command == "movel":
            pose = self.robot.movel(position, acc=acc, vel=vel, wait=wait)
        if command == "movej":
            pose = self.robot.movex(command, position, acc=acc, vel=vel, wait=wait)
        return pose

# Relative motion in the base coordinate .
    def move2x(self, x, acc=0.5, vel=0.5, wait=True):
        # identical to mvrx
        self.robot.translate([x, 0, 0], acc=acc, vel=vel, wait=wait)
        
    def move2y(self, y, acc=0.5, vel=0.5, wait=True):
        # identical to mvry
        self.robot.translate([0, y, 0], acc=acc, vel=vel, wait=wait)

    def move2z(self, z, acc=0.5, vel=0.5, wait=True):
        # identical to mvrz
        self.robot.translate([0, 0, z], acc=acc, vel=vel, wait=wait)
    
# Relative motion in the TCP coordinate.
    def move2xTCP(self, x=0.05, acc=0.01, vel=0.01, wait=True):
        # relative motion along x based on TCP coordinate.
        self.robot.translate_tool([x, 0, 0], acc=acc, vel=vel, wait=wait)

    def move2yTCP(self, y=0.05, acc=0.01, vel=0.01, wait=True):
        # relative motion along y based on TCP coordinate.
        self.robot.translate_tool([0, y, 0], acc=acc, vel=vel, wait=wait)

    def move2zTCP(self, z=0.05, acc=0.01, vel=0.01, wait=True):
        # relative motion along z based on TCP coordinate.
        self.robot.translate_tool([0, 0, z], acc=acc, vel=vel, wait=wait)

# Rotation motions

# tcp : the sample coordinate
# tool : the reference coordinate of which base is at the robot base
# camera: the sample coordinate base on the camera.
    def rotx(self, val, coordinate='tcp', acc=0.1, vel=0.1):
        # rotate around X axis by val in degree
        val = val/180*math.pi
        if coordinate == 'tcp':
            t = self.robot.get_pose()
            t.orient.rotate_xt(val)
            return self.robot.set_pose(t, wait=True, acc=acc, vel=vel)
        if coordinate == 'base':
            t = m3d.Transform()
            t.orient.rotate_xt(val)
            return self.robot.add_pose_base(t, wait=True, acc=acc, vel=vel)

    def roty(self, val, coordinate='tcp', acc=0.1, vel=0.1):
        # rotate around Y axis by val in degree
        val = val/180*math.pi
        if coordinate == 'tcp':
            t = self.robot.get_pose()
            t.orient.rotate_yt(val)
            return self.robot.set_pose(t, wait=True, acc=acc, vel=vel)
        if coordinate == 'base':
            t = m3d.Transform()
            t.orient.rotate_yt(val)
            return self.robot.add_pose_base(t, wait=True, acc=acc, vel=vel)

    def rotz(self, val, coordinate='tcp', acc=0.1, vel=0.1):
        # rotate around Z by val in degree
        val = val/180*math.pi
        if coordinate == 'tcp':
            t = self.robot.get_pose()
            t.orient.rotate_zt(val)
            return self.robot.set_pose(t, wait=True, acc=acc, vel=vel)
        if coordinate == 'base':
            t = m3d.Transform()
            t.orient.rotate_zt(val)
            return self.robot.add_pose_base(t, wait=True, acc=acc, vel=vel)
        if coordinate == 'camera':
            self.robot.set_tcp(self.camtcp)
            t = self.robot.get_pose()
            t.orient.rotate_zt(val)
            m = self.robot.set_pose(t, wait=True, acc=acc, vel=vel)
            self.robot.set_tcp(self.tcp)
            return m

    def rotj(self, *ang):
        # rotate joints by relative angle in degree
        myang = [0, 0, 0, 0, 0, 0]
        if len(ang) != 0:
            for i in range(len(ang)):
                myang[i] = ang[i]*math.pi/180
        j = self.getj()
        for i in range(len(j)):
            j[i] = j[i] + myang[i]
        self.robot.movej(j)

    def rotate(self, rotpos, rotaxis, rot_angles, acc=0.5, vel=0.5):
        # rotate around a given TCP position around an axis by a relative amount.
        # rotpos : center of rotation
        # rotaxis : rotation axis
        # rot_angles: rotation angle in degree
        t = self.robot.get_pose()
        #rotaxis = -came*dir[0] + camn*dir[1]
        t.orient.rotate_t(rotaxis, math.pi/180*rot_angles)
        if type(rotpos) == m3d.transform.Transform:
            t.set_pos(rotpos.get_pos())
        else:
            t.set_pos(rotpos)
        #v = self.robot.get_pose()
        #v = v*t # rotate around tool z
        m = self.robot.set_pose(t, acc=acc, vel=vel, wait=True, command='movej', threshold=None)
    
    def get_euler(self):
        # get euler angles in extrinsic 'xyz' rotation.
        ov = self.robot.get_orientation()
        #v = ov.to_euler('RPY')*180/math.pi
        #print(v.tolist())
        v = ov.list
        beta = math.atan2(-v[2][0], math.sqrt(v[0][0]**2+v[1][0]**2))
        alpha = math.atan2(v[1][0]/math.cos(beta), v[0][0]/math.cos(beta))
        gamma = math.atan2(v[2][1]/math.cos(beta), v[2][2]/math.cos(beta))
        #print(alpha, beta, gamma)
        return [alpha*180/math.pi, beta*180/math.pi, gamma*180/math.pi]
    
    def set_euler(self, ang):
        # get euler angles in extrinsic 'xyz' rotation.
        # [0, 0, 0] will point Z axis of the base.
        for i in range(len(ang)):
            ang[i] = ang[i]*math.pi/180
        a = ang[0]
        b = ang[1]
        c = ang[2]
        t = m3d.Transform()
        r11 = math.cos(a)*math.cos(b)
        r12 = math.cos(a)*math.sin(b)*math.sin(c)-math.sin(a)*math.cos(c)
        r13 = math.cos(a)*math.sin(b)*math.cos(c)+math.sin(a)*math.sin(c)
        r21 = math.sin(a)*math.cos(b)
        r22 = math.sin(a)*math.sin(b)*math.sin(c)+math.cos(a)*math.cos(c)
        r23 = math.sin(a)*math.sin(b)*math.cos(c)-math.cos(a)*math.sin(c)
        r31 = -math.sin(b)
        r32 = math.cos(b)*math.sin(c)
        r33 = math.cos(b)*math.cos(c)
        v =[[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]
        #print(v)
        t.orient.set_array(v)
        #print(t.orient.get_rotation_vector())
        return self.robot.set_orientation(t.orient.get_rotation_vector(), acc=0.5, vel=0.5)

# Gripper functions
    def activate_gripper(self):
        self.finger.gripper_activate()

    def grab(self):
        self.finger.close_gripper()
        #self.finger.gripper_action(255)

    def release(self):
        #self.finger.open_gripper()
        self.finger.gripper_action(120)

    def loosen(self):
        #self.finger.open_gripper()
        self.finger.gripper_action(190)

# Camera functions.
    def tweak_around_camera_axis(self, ang, acc=0.5, vel=0.5):
        # rolling up and down around the tool X axis.
        if self.tweak_reference_axis_angle is None:
            vv = self.robot.get_orientation()
            t = vv.get_vec_y()
            l = math.sqrt(t[0]**2+t[1]**2)
            rotang = math.acos(t[1]/l)*180/math.pi
            self.tweak_reference_axis_angle = rotang
        if ang ==0:
            return
        self.robot.set_tcp(self.camtcp)
        self.rotz(ang)
        self.robot.set_tcp(self.tcp)
        vv = self.robot.get_orientation()
        t = vv.get_vec_y()
        l = math.sqrt(t[0]**2+t[1]**2)
        rotang = math.acos(t[1]/l)*180/math.pi-self.tweak_reference_axis_angle
        self.tweak_axis_angle = rotang
        print(f"XY plane is rotated {rotang} degree from the reference direction.")

    def undo_tweak_around_camera_axis(self):
        if not hasattr(self, 'tweak_axis_angle'):
            return
        if self.tweak_reference_axis_angle != None:
            self.tweak_around_camera_axis(self.tweak_axis_angle)
            self.tweak_reference_axis_angle = None

    def capture_camera(self):
        if not self.camera._running:
            self.camera.capture()
        v = self.get_xyz().tolist()
        mystr = ''
        for m in v:
            if len(mystr)==0:
                mystr = "%0.4f" % m
            else:
                mystr = "%s_%0.4f" % (mystr, m)
        self.camera.save(filename=mystr)


    def tilt_over(self, distance=0, ang = 30, dir=[1, 0], acc=0.25, vel=0.25):
        # tilt around a point at a "distance" away from the camera along the camera axis.
        # dir: [1, 0] = north direction on the camera feed.
        # dir: [0, 1] = east direction on the camera feed. 
        #yv = self.get_camera_vector()
        camv, camn, came = self.get_camera_vector()
        CAMVECTOR_ToolCoordinate = [0, math.sin(math.pi/6), math.cos(math.pi/6)]
        self.robot.set_tcp(self.tcp)
        #xv = self.robot.get_orientation().get_vec_x()
        #zv = [0, -1, 0]
        rotcenter = self.camtcp.copy()
        for i in range(0, 3):
            rotcenter[i] = rotcenter[i]+distance*CAMVECTOR_ToolCoordinate[i]
        self.robot.set_tcp(rotcenter)
        t = self.robot.get_pose()
        rotaxis = -came*dir[0] + camn*dir[1]
        t.orient.rotate_b(rotaxis, math.pi/180*ang)
        #v = self.robot.get_pose()
        #v = v*t # rotate around tool z
        m = self.robot.set_pose(t, acc=acc, vel=vel, wait=True, command='movej', threshold=None)
        self.robot.set_tcp(self.tcp)
        return m

    def tilt_over_back(self, distance=0, ang = 30):
        return self.tilt_over(distance=distance, ang = ang, dir=[-1, 0])


    def tilt_camera_down(self):
#       make camera face down regardless of the direction of camera.
        trans = self.robot.get_pose()
        v = trans.orient.get_vec_y()
        # rotate in the TCP coordinate.
        # in the TCP coordinate, camera is at 30 degree along y direction.
        # therefore, rotating around x axis by 30 degree brings camera along Z.
        trans.orient.rotate_xt(math.pi/180*30)
        dist = self._TCP2CAMdistance*math.cos(math.pi/6)
        trans.set_pos(trans.get_pos()-v*dist)
        self.robot.set_pose(trans, 0.1, 0.1, wait=True)
        return trans

    def tilt_y(self):
        # have camera on +y axis and tilt down to face floor.
        val = 30
        v = self.robot.get_pose()
        val = val/180*math.pi
        orient = m3d.Orientation(m3d_Zdown_cameraY)
        orient.rotate_xt(val)
        v.orient = orient

        yv = self.robot.get_orientation().get_vec_y()
        #v.orient.rotate_xb(val)
        v.pos[1] += -self._TCP2CAMdistance*math.cos(math.pi/6)
        m = self.robot.set_pose(v, acc=1, vel=0.5, wait=True, command='movel', threshold=None)

        #if yv[1]<0: # need to rotate camera_back
        #    m = self.rotz(-90, acc=1, vel=1)
        #    m = self.rotz(-90, acc=1, vel=1)
        return m
        #return self.robot.set_pose(v, acc=0.5, vel=0.5, wait=True, command='movel', threshold=None)

    def tilt_ym(self):
        # have camera on -y axis and tilt down to face floor.

        val = 30
        v = self.robot.get_pose()
        val = val/180*math.pi
        orient = m3d.Orientation(m3d_Zdown_cameraYm)
        orient.rotate_xt(val)
        v.orient = orient

        # yv = self.robot.get_orientation().get_vec_y()
        # val = 30
        # v = self.robot.get_pose()
        # val = val/180*math.pi
        # v.orient.rotate_xb(val)
        v.pos[1] += self._TCP2CAMdistance*math.cos(math.pi/6)
        #return self.robot.set_pose(v, acc=0.5, vel=0.5, wait=True, command='movel', threshold=None)
        m = self.robot.set_pose(v, acc=0.5, vel=0.5, wait=True, command='movel', threshold=None)
        # if yv[1]>0 and abs(yv[2])<0.001: # need to rotate camera_back
        #     m = self.rotz(90, acc=1, vel=1)
        #     m = self.rotz(90, acc=1, vel=1)
        return m

    def tilt_xm(self):
        # have camera on -x axis and tilt down to face floor.

        val = 30
        v = self.robot.get_pose()
        val = val/180*math.pi
        orient = m3d.Orientation(m3d_Zdown_cameraXm)
        orient.rotate_xt(val)
        v.orient = orient

        v.pos[0] += self._TCP2CAMdistance*math.cos(math.pi/6)
        return self.robot.set_pose(v, acc=0.5, vel=0.5, wait=True, command='movel', threshold=None)

    def tilt_x(self):
        # have camera on +x axis and tilt down to face floor.
        val = 30
        v = self.robot.get_pose()
        val = val/180*math.pi
        orient = m3d.Orientation(m3d_Zdown_cameraX)
        orient.rotate_xt(val)
        v.orient = orient
        v.pos[0] += -self._TCP2CAMdistance*math.cos(math.pi/6)
        return self.robot.set_pose(v, acc=0.5, vel=0.5, wait=True, command='movel', threshold=None)

    def tilt_back(self):
        if self.is_Z_aligned():
            print("Robot is already Z aligned.")
            return
        val = 30
        t = self.robot.get_pose()
        val = val/180*math.pi
        #t = m3d.Transform()
        t.orient.rotate_xt(-val)
        v = t.orient.get_vec_y()
        #self.robot.add_pose_tool(t, acc=0.5, vel=0.5, wait=True, threshold=None)
        #t = m3d.Transform()
        vect = v*self._TCP2CAMdistance*math.cos(val)
#        vect = [0, self._TCP2CAMdistance, -self._TCP2CAMdistance*math.sin(val)]
        if not isinstance(vect, m3d.Vector):
            vect = m3d.Vector(vect)
        t.pos += vect
        #print(vect)
        return self.robot.set_pose(t, acc=0.5, vel=0.5, wait=True, threshold=None)

    def update_camera_info(self):
        # qrcode sav 
        # size; 22.5mm
        # at 250mm away, 120 pixels.
        qrsize = 22.5
        timeouttime = 1
        if not self.camera._running:
            self.camera.capture()
        self.camera.decode()
        t0 = time.time()
        while (time.time()-t0)<timeouttime:
            if len(self.camera.QRposition)==0:
                if not self.camera._running:
                    self.camera.capture()
                self.camera.decode()
                if len(self.camera.QRsize) == 0:
                    self.camera.QRsize[0] = 0
#                else:
#                    self.camera.QRdistance = 250/self.camera.QRsize[0]*120
            else:
                if self.camera.QRsize[0]>0:
                    self.camera.QRdistance = 250/self.camera.QRsize[0]*120
                return True
        if len(self.camera.QRposition)==0:
            print("Time out")
            return False
        else:
            if self.camera.QRsize[0]>0:
                self.camera.QRdistance = 250/self.camera.QRsize[0]*120
            return True

    def update_camera2QR_info(self):
        timeouttime = 1
        if not self.camera._running:
            self.camera.capture()
        self.camera.decode2QR()
        t0 = time.time()
        while (time.time()-t0)<timeouttime:
            if len(self.camera.QRposition)==0:
                if not self.camera._running:
                    self.camera.capture()
                self.camera.decode2QR()
                if len(self.camera.QRposition) == 0:
                    self.camera.QRsize[0] = 0
            else:
                if self.camera.QRsize[0]>0:
                    self.camera.QRdistance = 250/self.camera.QRsize[0]*120
                return True
        if len(self.camera.QRposition)==0:
            print("Time out")
            return False
        else:
            if self.camera.QRsize[0]>0:
                self.camera.QRdistance = 250/self.camera.QRsize[0]*120
            return True

    def bring_QR_to_camera_center(self, referenceName = "2QR", acc=0.1, vel=0.1):
        if not self.camera._running:
            self.camera.capture()
        if referenceName == "2QR":
            self.camera.decode2QR()
        else:
            self.camera.decode()
        isDistanceIn = False
        isNorthIn = False
        isEastIn = False
        distance = 0.01
        edist = 0.01
        ndist = 0.01
        prevndir = 0
        prevedir = 0
        prevdir = 0
        timeout = 1
        target_distance = 180
        # if referenceName == "2QR":
        #     target_QRedgelength = 100   # this is the edgelength of the 2QR references.
        # else:
        #     target_QRedgelength = 120   # this is the edgelength of a QR code at the optimum distance from a camera.
        #     target_distance = 150 #20mm
        #if not hasattr(self.camera, 'QRposition') or not hasattr(self.camera, 'QRsize'):
        if not self.camera._running:
            self.camera.capture()
        if referenceName == "2QR":
            self.camera.decode2QR()
        else:
            self.camera.decode()
        if hasattr(self.camera, 'QRtiltangle'):
#            print(f"Tilt angle is {self.camera.QRtiltangle}")
            if abs(self.camera.QRtiltangle) < 100:
                self.rotate_around_Zaxis_camera(-self.camera.QRtiltangle)
            else:
                self.rotate_around_Zaxis_camera(180-self.camera.QRtiltangle)
        if hasattr(self.camera, 'QRposition') and hasattr(self.camera, 'QRsize'):
            if referenceName == "2QR":
                rtn = self.update_camera2QR_info()
            else:
                rtn = self.update_camera_info()
            if rtn == False:
                return False
            failcount = 0
            while not isDistanceIn or not isNorthIn or not isEastIn:
#                if not hasattr(self.camera, 'QRposition') or not hasattr(self.camera, 'QRsize'):
#                    continue
                if referenceName == "2QR":
                    rtn = self.update_camera2QR_info()
                else:
                    rtn = self.update_camera_info()
                if rtn == False:
                    return False
                if not hasattr(self.camera, 'QRdistance'):
                    break
#                print(self.camera.QRdistance)
                if len(self.camera.QRposition)>0 and len(self.camera.QRsize)>0:
                    if self.camera.QRsize[0] == 0:
                        continue
                    imsize = self.camera.image.size
                    qs = sum(self.camera.QRsize)/len(self.camera.QRsize)
                    qs = self.camera.QRdistance
                    if abs(qs-target_distance) <2:
                        distance = 0.0
                        isDistanceIn = True
                    if qs-target_distance < 0:
                        dir = -1
                    else:
                        dir = 1
                    distance = abs(qs-target_distance)/1000

                    if self.camera.QRposition[0]-imsize[0]/2 < 0:
                        edir = -1
                    else:
                        edir = 1
                    if self.camera.QRposition[1]-imsize[1]/2 < 0:
                        ndir = 1
                    else:
                        ndir = -1
                    # QRcode size is 22mm x 22mm
                    ndist = abs(self.camera.QRposition[1]-imsize[1]/2)/self.camera.QRsize[0]*0.022
                    edist = abs(self.camera.QRposition[0]-imsize[0]/2)/self.camera.QRsize[0]*0.022
                    #print(f"distance is {distance}")
                    #print(f"ndist is {ndist}")
                    #print(f"edist is {edist}")

                    if distance < 5:
                        if abs(edist)<0.002 and abs(ndist)<0.002:
                            print("Done. QR code is centered.")
                            break

                    if abs(self.camera.QRposition[0]-imsize[0]/2) <5:
                        edist = 0.0
                        isEastIn = True

                    if abs(self.camera.QRposition[1]-imsize[1]/2) <5:
                        ndist = 0.0
                        isNorthIn = True
                    print(dir*distance, "North = ", ndir*ndist, "East =", edir*edist)
                    self.move_toward_camera(distance=dir*distance, north=ndir*ndist, east=edir*edist, acc=acc, vel=vel)
                else:
                    print(dir*distance, "North2 = ", ndir*ndist, "East2 =", edir*edist)
                    self.move_toward_camera(distance=-dir*distance, north=-ndir*ndist, east=-edir*edist, acc=acc, vel=vel)
                    failcount = failcount+1
                    if failcount>5:
                        break
            print("QR code is centered.")
        else:
            print("Bring QR closer to the camera")

    def tilt_align(self):
        if b'Follow me' in self.camera.QRdata:
            h, pd, ang, tilt = ctool.decodefollowme(self)
            tilt = np.array(tilt)
            #height = 0.3796914766079877
            print(f"'Follow me' is found at {h}m below.")
            while not (tilt[0] ==0 and tilt[1]==0):
                North = tilt[0]
                East = tilt[1]
                signN = np.sign(tilt[0])
                signE = np.sign(tilt[1])
                if North != 0:
                    self.tilt_over(h, ang=signN*5, dir = [1, 0])
                if East != 0:
                    self.tilt_over(h, ang=signE*5, dir = [0, 1])
                h, pd, ang, tilt = ctool.decodefollowme(self)
                while h==0:
                    h, pd, ang, tilt = ctool.decodefollowme(self)
                tilt = np.array(tilt)

    def relocate_camera(self):
        # Locate camera at the shortest distance between the objec and base.
        # align Z
        # set camera position 0.2m away from the obj
        obj_pos, campos = self.get_obj_position_from_camera_center()
        #if not isinstance(orient, m3d.Orientation):
        orient = m3d.Orientation([0, -math.pi, 0]) # make camera point +y axis.
        #orient.rotate_zb(math.pi/2) # make camera point +x
        trans = self.robot.get_pose()
        trans.orient = orient
        trans.orient.rotate_zb(math.atan2(obj_pos[1], obj_pos[0])-math.pi/2)
        newpos = (obj_pos.length-0.2)/obj_pos.length*obj_pos
        trans.set_pos(newpos)
        self.robot.set_pose(trans, acc=0.1, vel=0.1)

    def get_obj_position_from_camera_center(self):
        # calculate the object position that is at the center of camera image and QRrefdistance away from the camera surface.
        cameravector, v1, v2 = self.get_camera_vector()
        campos = self.get_camera_position()
        QRrefdistance = 0.2
        pos = campos.pos + QRrefdistance*cameravector/cameravector.length
        return pos, campos

    def move_over_camera(self, distance, acc=0.5, vel=0.5):
        # move along the camera north direction.
        cameravector = self.robot.get_orientation().get_vec_y()
        cameravector = cameravector*distance
        rotv = self.get_xyz().tolist()
        v = [rotv[0]+cameravector[0],rotv[1]+cameravector[1],rotv[2]+cameravector[2],rotv[3],rotv[4],rotv[5]]
        self.moveto(v, acc=acc, vel=vel)

    def get_camera_position(self):
        self.robot.set_tcp(self.camtcp)
        t = self.robot.get_pose()
        self.robot.set_tcp(self.tcp)
        return t

    def get_camera_vector(self):
        cameravector_east = self.robot.get_orientation().get_vec_x()
        v = self.robot.get_orientation()
        v.rotate_b(cameravector_east, math.pi/3)
        cameravector = v.get_vec_y()
        v = self.robot.get_orientation()
        v.rotate_b(cameravector_east, -math.pi/6)        
        cameravector_north = v.get_vec_y()
        #b = cameravector[1]*math.cos(math.pi/6)+cameravector[2]*math.sin(math.pi/6)
        #c = -cameravector[1]*math.sin(math.pi/6)+cameravector[2]*math.cos(math.pi/6)
        #b2 = cameravector[1]*math.cos(-math.pi/6*2)+cameravector[2]*math.sin(-math.pi/6*2)
        #c2 = -cameravector[1]*math.sin(-math.pi/6*2)+cameravector[2]*math.cos(-math.pi/6*2)
        #cameravector[1] = b
        #cameravector[2] = c
        #cameravector_north[1] = b2
        #cameravector_north[2] = c2
        return cameravector, cameravector_north, -cameravector_east

    def move_toward_camera(self, distance, north=0.0, east=0.0, acc=0.5, vel=0.5):
        # north is the vertical direction on the camera image.
        # east is the right direction on the camera image.
        # When distance is 0, the TCP moves on the normal plane to the camera vector.
        cameravector, northv, eastv = self.get_camera_vector()
        cameravector = cameravector*distance
        cameravector = cameravector + east*eastv + north*northv
        rotv = self.get_xyz().tolist()
        v = [rotv[0]+cameravector[0],rotv[1]+cameravector[1],rotv[2]+cameravector[2],rotv[3],rotv[4],rotv[5]]
#        self.moveto(v, acc=acc, vel=vel)
        
        np = m3d.Transform(v)
        #self.robot.movej(np, acc=acc, vel=vel)
        self.robot.set_pose(np, acc=acc, vel=vel, wait=True, command="movej", threshold=None)

    def camera_y(self):
        trans = self.robot.get_pose()
        pos = trans.get_pos()
        if pos[1] < 0:
            orient = m3d.Orientation([math.pi/2, 0, 0]) # make camera point +y axis.
            orient.rotate_xb(math.pi/6)
        else:
            orient = m3d.Orientation([-math.pi/2, 0, 0]) # make camera point +y axis.
            orient.rotate_xb(-math.pi/6)
        #orient.rotate_zb(math.pi/2) # make camera point +x
        
        trans.orient = orient
        self.robot.set_pose(trans, acc=0.1, vel=0.1)

    def camera_x(self):
        trans = self.robot.get_pose()
        pos = trans.get_pos()
        if pos[0] < 0:
            orient = m3d.Orientation([0, -math.pi/2, 0]) # make camera point +y axis.
            orient.rotate_yb(-math.pi/6)
        else:
            orient = m3d.Orientation([0, math.pi/2, 0]) # make camera point +y axis.
            orient.rotate_yb(math.pi/6)
        #orient.rotate_zb(math.pi/2) # make camera point +x
        trans.orient = orient
        self.robot.set_pose(trans, acc=0.1, vel=0.1)

    def camera_out(self):
        #self.along(self.__TCP2CAMdistance*math.cos(math.pi/6))
        # if hasattr(self, '_pos_before_camtilt'):
        #     v0 = self._pos_before_camtilt
        #     np0 = m3d.Transform(v0)
        # else:
        v0 = self.get_xyz().tolist()
        tilt = []
        tilt.append([0.0, -3.034475506348704, 0.8129493162280801]) # y
        tilt.append([-2.6178584471090356, 0.0, 0.0]) #ym
        tilt.append([-1.9267699119689394, -1.9268792905340348, 0.5161854379935282]) #x
        tilt.append([-1.9268292866190757, 1.927014041189096, -0.516717552638397]) # xm
        # Y vector:
        yv = []
        # yv = self.robot.get_orientation().get_vec_y()
        yv.append([-0.00009, 0.86610, -0.49988])
        yv.append([-0.00003, -0.86596, -0.50012])
        yv.append([0.86601, 0.00012, -0.50003])
        yv.append([-0.86603, 0.00006, -0.49999])
        sh = self._TCP2CAMdistance*math.cos(math.pi/6)
        k = 0
        for t in tilt:
#            print(((v0[3]-t[0])**2+(v0[4]-t[1])**2+(v0[5]-t[2])**2))
            if ((v0[3]-t[0])**2+(v0[4]-t[1])**2+(v0[5]-t[2])**2) < 0.01:
                if k==0:
                    v0 = [v0[0], v0[1]+sh, v0[2], 0, math.pi, 0]
                    print('y')
                    break
                if k==1:
                    sh = -sh
                    v0 = [v0[0], v0[1]+sh, v0[2], 0, -math.pi, 0]
                    print('ym')
                    break
                if k==2:
                    v0 = [v0[0]+sh, v0[1], v0[2], 0, math.pi, 0]
                    print('x')
                    break
                if k==3:
                    sh = -sh
                    v0 = [v0[0]+sh, v0[1], v0[2], 0, math.pi, 0]
                    print('xm')
                    break
            k = k+1
#        v0 = [v0[0]+sh, v0[1], v0[2], 0, math.pi, 0]
        np0 = m3d.Transform(v0)

        self.robot.movel(np0, acc=0.5, vel=0.5)

    def rotate_around_Zaxis_camera(self, ang):
        # rotate around the camera axis, degree input.
        self.robot.set_tcp(self.camtcp)
        self.rotz(ang)
        self.robot.set_tcp(self.tcp)

    def get_inplane_angle_from_idealZ(self):
        t = self.robot.get_pose()
        zv = t.orient.get_vec_z()
        ang = math.atan2(zv[1], zv[0])+math.pi/2
        return ang

    def put_tcp2camera(self):
        self.robot.set_tcp(self.camtcp)
        pos = self.robot.get_pos()
        self.robot.set_tcp(self.tcp)
        self.tilt_back()
        self.robot.set_pos(pos, acc=0.1, vel=0.1)

    def put_camera2tcp(self):
        pos = self.robot.get_pos()
        self.tilt_camera_down()
        self.robot.set_tcp(self.camtcp)
        self.robot.set_pos(pos, acc=0.1, vel=0.1)
        self.robot.set_tcp(self.tcp)