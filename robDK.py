# UR3
from PyQt5.QtCore import (pyqtSignal, QObject)
from PyQt5.QtWidgets import QWidget
import os
import sys
import math3d as m3d
import time
import math

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT     # RoboDK API
import robodk.robomath as rkmath      # Robot toolbox

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

def ind2sub(ind, array_shape):
    rows = int(ind / array_shape[1])
    cols = (int(ind) % array_shape[1]) # or numpy.mod(ind.astype('int'), array_shape[1])
    return (rows, cols)

def sub2ind(rows, cols, array_shape):
    return rows*array_shape[1] + cols

class roboUR3(QObject):
    # unit of position vector : meter.
    # unit of angles: degree
    
    RDK = Robolink()
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    #target = RDK.Item('Target 3')
    #frameSam = RDK.Item('UR3e Base')
    #frameMag = RDK.Item('Magazine')

    tl = RDK.Item('Tool 1')

    #robot.setFrame(frameSam)
    
    sigFinger = pyqtSignal(str)
    sigMoving = pyqtSignal(bool)

    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]

    def __init__(self):
        super(roboUR3, self).__init__()
# definition of Cartesian Axis of UR3 at 12idb.
# X : positive - Out board
# X : negative - In board
# Y : positive - Along X-ray
# Y : negative - Again X-ray
#        self.robot = robot
        #self.finger.gripper_activate()
        self.name = "roboUR3"
    
    def terminate(self):
        pass
    
    def pose_2_m3d_Orientation(self):
        pose = self.robot.Pose()
        o = m3d.Orientation(pose.Rot33().list2())
        return o
    
    def Rot33_2_Pose(self, mat):
        pose = self.robot.Pose()
        pose.setVX(mat[0])
        if len(mat) >= 2:
            pose.setVY(mat[1])
            if len(mat) == 3:
                pose.setVZ(mat[2])   
        return pose
        
    def get_orientation(self):
        pose = self.robot.Pose()
        m = rkmath.Pose_2_UR(pose)
        del m[0:3]
        for i in range(0,3):
            m[i] = m[i]*180/math.pi
        return m
    
    def set_orientation(self, *ang):
        # usage:
        # rob = roboUR3()
        # rob.set_orientation()
        # rob.set_orientation([180, 0, 0])
        # rob.set_orientation(180, 0, 0)
        t = self.robot.Pose()
        if len(ang) < 2:
            if len(ang) == 0:
                self.set_orient_tcpZ(m3d_Zdown_cameraY[2], m3d_Zdown_cameraY[1])
                return self.robot.Pose()
            if len(ang) == 1: 
                ang = ang[0]
            if len(ang) == 2:
                print("Input Error")
                return False
        mypos = t.Pos()
        if len(ang) != 0:
            for i in range(len(ang)):
                mypos.append(ang[i]*math.pi/180)
            self.robot.MoveJ(rkmath.UR_2_Pose(mypos))
            return self.robot.Pose()
        # # set rotation_vector
        # for i in range(0,2):
        #     myang[i] = myang[i]*math.pi/180
        # return self.set_orientation(myang)

    def set_orient_tcpZ(self, axis=[0, 0, -1], ny=[]):
        # To align the Z axis of TCP to the axis (in base frame)
        axis = rkmath.normalize3(axis)
        pose = self.robot.Pose()
        vz = pose.VZ() # TCP vz vector in the base frame
        pos = pose.Pos()
        nz = axis
        if len(ny)==0:
            nx = rkmath.cross(vz, axis) # vx should point to nx
            ny = rkmath.cross(nx, axis)
        else:
            ny = rkmath.normalize3(ny)
        if rkmath.norm(ny) < 0.00001:
            ny = pose.VY()
        pose2 = rkmath.point_Zaxis_2_pose(pos, nz, ny)
        self.robot.MoveJ(pose2)


    def pos2trans(self, pos):
        return m3d.Transform(pos)
        
    def get_xyz(self):
        pose = self.robot.Pose()
        vect = rkmath.Pose_2_UR(pose)
        for i in range(3):
            vect[i] /= 1000 # unit conversion from mm to m.
        return vect

    #Why 0.999?
    def is_Z_aligned(self):
        v = self.robot.Pose()
        vec = v.VZ()
        if abs(vec[2]) < 0.999:
            return False
        else:
            return True
    
    #What is the difference between these last two functions?

# Linear motions
    # absolute positinng in 'base' frame.
    # value should be in a meter unit.

    #Position, any axis, absolute units
    def mvTxyzabs_ax(self, val, ax):
        # val in mm unit
        v = self.robot.Pose()
        pos = v.Pos()
        pos[ax] = val
        v.setPos(pos)
        self.robot.MoveL(v)
    
    #Position, any axis, relative units
    def mvTxyzrel_ax(self, val, ax):
        # val in mm unit
        v = self.robot.Pose()
        pos = v.Pos()
        pos[ax] = pos[ax]+val
        v.setPos(pos)
        self.robot.MoveL(v)
    
    #Orientation, any axis, absolute units
    def mvRxyzabs_ax(self, val, ax):
        # val should be in degree.
        pose = self.robot.Pose()
        txyzrxyz = rkmath.Pose_2_TxyzRxyz(pose)
        txyzrxyz[ax] = val*math.pi/180
        self.robot.MoveL(rkmath.TxyzRxyz_2_Pose(txyzrxyz))

    #Orientation, any axis, relative units
    def mvRxyzrel_ax(self, val, ax):
        # val should be in degree.
        pose = self.robot.Pose()
        txyzrxyz = rkmath.Pose_2_TxyzRxyz(pose)
        txyzrxyz[ax] = txyzrxyz[ax] + val*math.pi/180
        self.robot.MoveL(rkmath.TxyzRxyz_2_Pose(txyzrxyz))

    def mvx(self, val):
        self.mvTxyzabs_ax(val*1000, 0)

    def mvy(self, val):
        self.mvTxyzabs_ax(val*1000, 1)

    def mvz(self, val):
        self.mvTxyzabs_ax(val*1000, 2)

    def mvrx(self, val):
        # val should be in degree.
        self.mvRxyzabs_ax(val, 3)
        # v = self.get_xyz().tolist()
        # v[3] = val
        # self.movel(v)

    def mvry(self, val):
        self.mvRxyzabs_ax(val, 4)
        # v = self.get_xyz().tolist()
        # v[4] = val
        # self.movel(v)

    def mvrz(self, val):
        self.mvRxyzabs_ax(val, 5)
        # v = self.get_xyz().tolist()
        # v[5] = val
        # self.movel(v)

    #What is mv?
    def mv(self, pos, frame='base'):
        # pos = [x, y, z] in m unit.
        v = self.get_xyz()
        for i in range(3):
            v[i] = pos[i]*1000
        if frame=='base':
            pose = rkmath.UR_2_Pose(v)
            self.robot.MoveL(pose)
        else: # in tcp frame
            self.robot.MoveL(pose*rkmath.transl(pos[0], pos[1], pos[2]))

    """
    def movels(self, pos_list):
        " movels([0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [0.2, 0.3, 0.2]], radius=0.1)"
        
        v = self.get_xyz()
        for i, vec in enumerate(pos_list):
            if len(vec) == 3:
                pos_list[i].append(v[3])
                pos_list[i].append(v[4])
                pos_list[i].append(v[5])
        
        self.robot.moveL(pos_list)
        #return pose
    """

    # relative positioning in the base or tcp frames .
    def translate(self, v, frame='base'):
        for i in range(3):
            v[i] = v[i]*1000
        pose = self.robot.Pose()
        if frame == 'tcp':
            self.robot.MoveL(pose*rkmath.transl(v[0], v[1], v[2]))
        if frame == 'base':
            pos = pose.Pos()
            for i in range(3):
                pos[i] = pos[i] + v[i]
            print(pos)
            pose.setPos(pos)
            print(pose)
            self.robot.MoveL(pose)

    # relative move in the base frame.
    def move2x(self, x):
        # identical to mvrx
        self.translate([x, 0, 0], frame='base')
    
    def move2y(self, y):
        # identical to mvry
        self.translate([0, y, 0], frame='base')

    def move2z(self, z):
        # identical to mvrz
        self.translate([0, 0, z], frame='base')
    
    # relative positioning in the TCP coordinate.
    def move2xTCP(self, x=0.05, acc=0.01, vel=0.01, wait=True):
        # relative motion along x based on TCP coordinate.
        self.translate([x, 0, 0], frame='tcp')
        #p = self.robot.getl()
        #p[0] += x
        #self.robot.movel(p, acc=acc, vel=vel, wait=wait)

    def move2yTCP(self, y=0.05, acc=0.01, vel=0.01, wait=True):
        # relative motion along y based on TCP coordinate.
        self.translate([0, y, 0], frame='tcp')
        #p = self.robot.getl()
        #p[1] += y
        #self.robot.movel(p, acc=acc, vel=vel, wait=wait)

    def move2zTCP(self, z=0.05, acc=0.01, vel=0.01, wait=True):
        # relative motion along z based on TCP coordinate.
        self.translate([0, 0, z], frame='tcp')
#        p = self.robot.getl()
#        p[2] += z
#        self.robot.movel(p, acc=acc, vel=vel, wait=wait)

#    def down(self, z=0.05, acc=0.01, vel=0.01, wait=True):
#        self.up(-z, acc, vel, wait)

    def getj(self):
        j = self.robot.Joints()
        vect = j.tolist()
        for i in range(6):
            vect[i] *= vect[i]/180*math.pi
        return vect
    # @property
    # def whereisFinger(self): 
    #     return self._fingerpos 
    # @whereisFinger.setter 
    # def whereisFinger(self, pos): 
    #     self._fingerpos = pos 
    
    def m3d_2_pose(self, path):
        p = path.get_pose_vector()
        ang = path.orient.to_euler('xyz')
        path = p.tolist()
        path[3] = ang[0]
        path[4] = ang[1]
        path[5] = ang[2]
        return path

    def setTarget(self, pos):
        pose = self.robot.Pose()
#        print(pos)
        pose.setPos(pos)
        return pose

    def getUR3Pos(self, p):
        # p is position in UR3
        myp = rkmath.Pose_2_UR(p)
#        print(myp)
        pose = rkmath.TxyzRxyz_2_Pose(myp)
        #tg = self.setTarget(myp)
        return pose

    def movel(self, UR3pos):
        p = self.getUR3Pos(UR3pos)
        self.robot.MoveL(p)

    # UR robot uses axis-angle represation for a rotation.
    # unit vector u for the axis of the orientation.
    # angle theta (in radian) for the angle.
    # use get_rotation and set_rotation for this representation..
    # here, axis the vector in 'base' frame.
    # How to visualize the orientation:
    # First, align the TCP xyz frame to Base xyz frame 
    # Then, then rotate TCP frame around the axis.

    def get_rotation(self):
        ur = rkmath.Pose_2_UR(self.robot.Pose())
        ang = rkmath.norm(ur[3:])*180/math.pi
        axis = rkmath.normalize3(ur[3:])
        # ang : in radian
        # axis unit vector
        return (ang, axis)

    def set_rotation(self, ang, axis):
        # ang should be degree.
        ur = rkmath.Pose_2_UR(self.robot.Pose())
        #print(ur)
        axis = rkmath.normalize3(axis)
        ang = ang*math.pi/180
        for i in range(3):
            ur[3+i] = ang*axis[i]
        #print(ur)
        self.robot.MoveJ(rkmath.UR_2_Pose(ur))

    # relatve rotation
    def rotx(self, val):
        # val in degree
        self.rotate('x', val)

    def roty(self, val):
        # val in degree
        self.rotate('y', val)

    def rotz(self, val):
        # val in degree
        self.rotate('z', val)
    
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

    def rotate(self, axis, val, frame='tcp'):
        # val in degree.
        # this is identical to mvRxyzrel_ax
        val = val*math.pi/180
        pose = self.robot.Pose()
        print("Starting pose: ", pose.Rot33())
        if isinstance(axis, str):
            if axis == 'x':
                r = rkmath.rotx(val)
                axis = [1, 0, 0]
            if axis == 'y':
                r = rkmath.roty(val)
                axis = [0, 1, 0]
            if axis == 'z':
                r = rkmath.rotz(val)
                axis = [0, 0, 1]
        if len(axis)==3:
            if frame == 'base':
                # calculate coordinate of an axis of a base frame in a tcp frame
                axis = pose.inv().Rot33()*axis
            # make a rotation axis
            v = [0, 0, 0, val*axis[0], val*axis[1], val*axis[2]]
            # convert this delta_rot into a pose
            r = rkmath.UR_2_Pose(v)
        # translate pose by r
        npose = pose*r
        self.robot.MoveJ(npose)

    def get_euler(self):
        # get euler angles in extrinsic 'xyz' rotation.
        # KUKA representation: [x, y, z, A, B, C] (mm and euler angle (deg))
        pose = self.robot.Pose()
        XYZABC = rkmath.Pose_2_KUKA(pose)
        return (XYZABC[3], XYZABC[4], XYZABC[5])
    
    def set_euler(self, ang):
        # get euler angles in extrinsic 'xyz' rotation.
        # [0, 0, 0] will point Z axis of the base.
        # KUKA representation: [x, y, z, A, B, C] (mm and euler angle (deg))
        pose = self.robot.Pose()
        XYZABC = rkmath.Pose_2_KUKA(pose)
        for i in range(3):
            XYZABC[2+i] = ang[i]
        self.robot.MoveJ(rkmath.KUKA_2_Pose(XYZABC))
        # for i in range(len(ang)):
        #     ang[i] = ang[i]*math.pi/180
        # a = ang[0]
        # b = ang[1]
        # c = ang[2]
        # t = m3d.Transform()
        # r11 = math.cos(a)*math.cos(b)
        # r12 = math.cos(a)*math.sin(b)*math.sin(c)-math.sin(a)*math.cos(c)
        # r13 = math.cos(a)*math.sin(b)*math.cos(c)+math.sin(a)*math.sin(c)
        # r21 = math.sin(a)*math.cos(b)
        # r22 = math.sin(a)*math.sin(b)*math.sin(c)+math.cos(a)*math.cos(c)
        # r23 = math.sin(a)*math.sin(b)*math.cos(c)-math.cos(a)*math.sin(c)
        # r31 = -math.sin(b)
        # r32 = math.cos(b)*math.sin(c)
        # r33 = math.cos(b)*math.cos(c)
        # v =[[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]
        # #print(v)
        # t.orient.set_array(v)
        # #print(t.orient.get_rotation_vector())
        # return self.robot.set_orientation(t.orient.get_rotation_vector(), acc=0.5, vel=0.5)

 
    def grab(self):
        self.sam.setParentStatic(self.tl)

    def release(self):
        self.sam.setParentStatic(self.MagMag)

    def loosen(self):
        self.release()