''' This program is to define UR robot with Robotiq gripper and camera '''
#import time
from PyQt5.QtCore import QObject

import numpy as np
import logging
import math
import math3d as m3d
import os
from enum import Enum
import sys
sys.path.append('..')
from common.urdashboard import dashboard
from common.urcamera import camera
from urxe import ursecmon
text_file_path = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(text_file_path, '..', 'urscripts', 'checkdistance.script'), 'r') as file:
    CheckdistanceScript = file.read()

## Exception handling....
class RobotException(Exception):
    pass

class NoCameraException(Exception):
    pass

class NoFingerException(Exception):
    pass


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

class SafetyMode(Enum):
    IS_NORMAL_MODE = 1
    IS_REDUCED_MODE = 2
    IS_PROTECTIVE_STOPPED = 3
    IS_RECOVERY_MODE = 4
    IS_SAFEGUARD_STOPPED = 5
    IS_SYSTEM_EMERGENCY_STOPPED = 6
    IS_ROBOT_EMERGENCY_STOPPED = 7
    IS_EMERGENCY_STOPPED = 8
    IS_VIOLATION = 9
    IS_FAULT = 10
    IS_STOPPED_DUE_TO_SAFETY = 11

class SafetyStatus(Enum):
    SAFETY_STATUS_SYSTEM_THREE_POSITION_ENABLING_STOP	= 13
    SAFETY_STATUS_AUTOMATIC_MODE_SAFEGUARD_STOP	= 12
    SAFETY_STATUS_UNDEFINED_SAFETY_MODE	= 11
    SAFETY_STATUS_VALIDATE_JOINT_ID	= 10
    SAFETY_STATUS_FAULT	= 9
    SAFETY_STATUS_VIOLATION	= 8
    SAFETY_STATUS_ROBOT_EMERGENCY_STOP	= 7
    SAFETY_STATUS_SYSTEM_EMERGENCY_STOP	=6
    SAFETY_STATUS_SAFEGUARD_STOP	=5
    SAFETY_STATUS_RECOVERY	=4
    SAFETY_STATUS_PROTECTIVE_STOP	=3
    SAFETY_STATUS_REDUCED	= 2
    SAFETY_STATUS_NORMAL	=1

class RobotMode(Enum):
    ROBOT_MODE_NO_CONTROLLER	= -1	
    ROBOT_MODE_DISCONNECTED		= 0	
    ROBOT_MODE_CONFIRM_SAFETY	= 1	
    ROBOT_MODE_BOOTING			= 2	
    ROBOT_MODE_POWER_OFF		= 3	
    ROBOT_MODE_POWER_ON			= 4	
    ROBOT_MODE_IDLE				= 5	
    ROBOT_MODE_BACKDRIVE		= 6	
    ROBOT_MODE_RUNNING			= 7	
    ROBOT_MODE_UPDATING_FIRMWARE = 8	

class UR(QObject):
    # unit of position vector : meter.
    _TCP2CAMdistance = 0.12
    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]
    camtcp = [0, 0.0433, 0.015, -math.pi/180*30, 0, 0]

    def __init__(self, name = 'UR3', package='urxe', grippertype=1, cameratype=1, use_rtde=False):
        super().__init__()
        # grippertype:
        #   0: No gripper
        #   1: Robotiq gripper
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

        ## load package.........
        if package == 'urxe':
            from urxe.robot import RobotiqGripper
            from urxe.robot import Robot
        if package == 'rtde':
            from rtde.robot import Robot
            from rtde.robot import RobotiqGripper
            use_rtde = True

        try:
            self.robot = Robot(IP,use_rtde=use_rtde)
        except TimeoutError:
            raise RobotException(f'Robot {IP} does not respond.')
        except ursecmon.ProtectiveStopException:
            print("Protective stoppped.. Connecting again.")
            self.robot = Robot(IP,use_rtde=use_rtde)

        if cameratype==2:
            self.camera = camera(IP='')
        elif cameratype==1:
            self.camera = camera(IP)
        else:
            pass

        if grippertype==1:
            self.gripper = RobotiqGripper(self.robot)
        else:
            pass

        #self.__TCP2CAMdistance = 0.15
        self.robot.IP = IP
        try:
            self.set_tcp(self.tcp)
        except:
            pass
        try:
            self.set_payload(1.35, (-0.003,0.01,0.037))
        except:
            pass
        self.dashboard = dashboard(self.robot)

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
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.set_pose(*args, **kwargs)
    
    def movels(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.movels(*args, **kwargs)
    
    def movel(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.movel(*args, **kwargs)
    
    def movej(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.movej(*args, **kwargs)
    
    def get_safety_mode(self):
        return self.robot.get_safety_mode()
    
    def get_status(self):
        # getting saftey status from dashboard.
        return self.dashboard.get_status()

    def unlock_stop(self):
        self.dashboard.unlock()

    def is_protective_stopped(self):
        return self.robot.secmon.is_protective_stopped()   

    def is_running(self):
        return self.robot.is_program_running()

    def set_orientation_rad(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        self.robot.set_orientation(*args, **kwargs)
    
    def add_pose_base(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.add_pose_base(*args, **kwargs)
    
    def translate(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.translate(*args, **kwargs)
    
    def translate_tool(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        return self.robot.translate_tool(*args, **kwargs)

    def bump(self, *args, **kwargs):
        mode = self.get_safety_mode()
        if mode>2:
            raise RobotException(SafetyStatus(mode))
        self.robot.bump(*args, **kwargs)
    
    def set_tool_communication(self, obj): #tool_setting is a dict
        # rob.set_tool_communication(rob.gripper)
        tool_setting = obj._comm_setting
        self.robot.set_tool_communication(enabled=True,baud_rate=tool_setting["baud_rate"],
                                          parity=tool_setting["parity"],
                                          stop_bits=tool_setting["stop_bits"],
                                          rx_idle_chars=tool_setting["rx_idle_chars"],
                                          tx_idle_chars=tool_setting["tx_idle_chars"])        
    def set_tool_communication_off(self):
        self.robot.set_tool_communication(enabled=False)

    def get_movement_state(self) -> str:
        """Gets robot movement status by checking robot joint values.
        Return (str) READY if robot is not moving
                     BUSY if robot is moving
        """
        current_location = self.getj()
        current_location = [
            "%.2f" % value for value in current_location
        ]  # rounding to 3 digits
        # print(current_location)
        if not hasattr(self, 'robot_current_joint_angles'):
            self.robot_current_joint_angles = current_location
            current_location = self.getj()
            current_location = [
                "%.2f" % value for value in current_location
            ]  # rounding to 3 digirobot_current_joint_anglests

        if self.robot_current_joint_angles == current_location:
            movement_state = "READY"
        else:
            movement_state = "BUSY"

        self.robot_current_joint_angles = current_location

        return movement_state

    def calc_position_in_base(self, pos):
        # pos is a coordinate in tcp coordinate.
        # returns a coordinate in the base coordinate.
        trans = self.get_pose()
        if not isinstance(pos, m3d.Transform):
            pos = m3d.Transform([pos[0], pos[1], pos[2], 0, 0, 0])
        n = pos*trans
        return n

    def calc_position_in_tool(self, pos):
        # pos is a coordinate in base coordinate.
        # returns the coordinate in the tool coordinate.
        trans = self.get_pose()
        try:
            v = trans.get_pos().array
        except:
            v = trans.get_pose_vector()
        trans.invert()
        if not isinstance(pos, m3d.Transform):
            pos = m3d.Transform([pos[0], pos[1], pos[2], v[3], v[4], v[5]])
        n = pos*trans
        return n

    def get_orientation(self):
        # get rotation_vector.
        try:
            m = self.get_pose().orient.get_rotation_vector().array.tolist()
        except:
            m = self.get_pose().orient.get_rotation_vector().tolist()
        for i in range(0,2):
            m[i] = m[i]*180/math.pi
        return m

    def set_orientation(self, *ang):
        # set_orientation()
        # set_orientation(orientation_matrix)
        # set_orientation(0, -180, 0)
        if len(ang) < 2:
            if len(ang) ==0:
                orient = m3d.Orientation(self.orientation)
            elif len(ang) == 1: 
                orient = m3d.Orientation(ang[0])
            t = self.get_pose()
            t.orient = orient
            self.set_pose(t, 0.5, 0.5, wait=True)
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
        return self.set_orientation_rad(myang, acc=1, vel=0.5)

    def get_xyz(self):
        # return the Cartesian position as a numpy array
        # if you want the Cartesian position as a list, use self.robot.getl()
        pose = self.get_pose()
        try:
            return pose.get_pos().array
        except:
            return pose.get_pose_vector()

    ## new functions
    def is_Z_aligned(self):
        v = self.get_pose()
        vec = v.orient.vec_z
        if abs(vec[2]) < 0.999:
            return False
        else:
            return True
# Linear motions
    # absolute positinng
    def move2x(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[0] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def move2y(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[1] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def move2z(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[2] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def move2rx(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[3] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def move2ry(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[4] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def move2rz(self, val, acc=0.5, vel=0.5, wait=True):
        v = self.get_xyz().tolist()
        v[5] = val
        self.moveto(v, acc=acc, vel=vel, wait=wait)

    def move_poses(self, pos_list, radius = 0.01, acc=0.5, vel=0.5, wait=True):
        " movels([0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [0.2, 0.3, 0.2]], radius=0.1)"
        v = self.get_xyz().tolist()
        for i, vec in enumerate(pos_list):
            if len(vec) == 3:
                pos_list[i].append(v[3])
                pos_list[i].append(v[4])
                pos_list[i].append(v[5])
        pose = self.movels(pos_list, acc=acc, vel=vel, radius=radius, wait=wait)
        return pose

    def moveto(self, position, command="movel", acc=0.5, vel=0.5, wait=True):
        if type(position) == list or np.ndarray:
            if len(position) == 3:
                v = self.get_xyz().tolist()
                position.append(v[3])
                position.append(v[4])
                position.append(v[5])
        if command == "movel":
            pose = self.movel(position, acc=acc, vel=vel, wait=wait)
        if command == "movej":
            pose = self.movej(position, acc=acc, vel=vel, wait=wait)
        return pose

# Relative motion in the base coordinate .
    def mvr2x(self, x, acc=0.5, vel=0.5, wait=True):
        # identical to mvrx
        self.translate([x, 0, 0], acc=acc, vel=vel, wait=wait)
        
    def mvr2y(self, y, acc=0.5, vel=0.5, wait=True):
        # identical to mvry
        self.translate([0, y, 0], acc=acc, vel=vel, wait=wait)

    def mvr2z(self, z, acc=0.5, vel=0.5, wait=True):
        # identical to mvrz
        self.translate([0, 0, z], acc=acc, vel=vel, wait=wait)
    
# Relative motion in the TCP coordinate.
    def mvr2xTCP(self, x=0.05, acc=0.5, vel=0.5, wait=True):
        # relative motion along x based on TCP coordinate.
        self.translate_tool([x, 0, 0], acc=acc, vel=vel, wait=wait)

    def mvr2yTCP(self, y=0.05, acc=0.5, vel=0.5, wait=True):
        # relative motion along y based on TCP coordinate.
        self.translate_tool([0, y, 0], acc=acc, vel=vel, wait=wait)

    def mvr2zTCP(self, z=0.05, acc=0.5, vel=0.5, wait=True):
        # relative motion along z based on TCP coordinate.
        self.translate_tool([0, 0, z], acc=acc, vel=vel, wait=wait)

# Rotation motions

# tcp : the sample coordinate
# tool : the reference coordinate of which base is at the robot base
# camera: the sample coordinate base on the camera.
    # rotx: where x stands for the tcp x axis. Note that camera is out along the tcp y axis.
    def rotx(self, val, coordinate='tcp', wait=True, acc=0.1, vel=0.3):
        # rotate around X axis by val in degree
        val = val/180*math.pi
        if coordinate == 'tcp':
            t = self.get_pose()
            t.orient.rotate_xt(val)
            return self.set_pose(t, wait=wait, acc=acc, vel=vel)
        if coordinate == 'base':
            t = m3d.Transform()
            t.orient.rotate_xt(val)
            return self.add_pose_base(t, wait=wait, acc=acc, vel=vel)
        if coordinate == 'camera':
            self.set_tcp(self.camtcp)
            t = self.get_pose()
            t.orient.rotate_xt(val)
            m = self.set_pose(t, wait=wait, acc=acc, vel=vel)
            self.set_tcp(self.tcp)
            return m

    def roty(self, val, coordinate='tcp', wait=True, acc=0.1, vel=0.3):
        # rotate around Y axis by val in degree
        val = val/180*math.pi
        if coordinate == 'tcp':
            t = self.get_pose()
            t.orient.rotate_yt(val)
            return self.set_pose(t, wait=wait, acc=acc, vel=vel)
        if coordinate == 'base':
            t = m3d.Transform()
            t.orient.rotate_yt(val)
            return self.add_pose_base(t, wait=wait, acc=acc, vel=vel)
        if coordinate == 'camera':
            self.set_tcp(self.camtcp)
#            m = self.rotate([0, 1, 0], val)
            t = self.get_pose()
            t.orient.rotate_yt(val)
            m = self.set_pose(t, wait=wait, acc=acc, vel=vel)
            self.set_tcp(self.tcp)
            return m

    def rotz(self, val, coordinate='tcp', wait=True, acc=0.1, vel=0.3):
        # rotate around Z by val in degree
        val = val/180*math.pi
        if coordinate == 'tcp':
            t = self.get_pose()
            t.orient.rotate_zt(val)
            return self.set_pose(t, wait=wait, acc=acc, vel=vel)
        if coordinate == 'base':
            t = m3d.Transform()
            t.orient.rotate_zt(val)
            return self.add_pose_base(t, wait=wait, acc=acc, vel=vel)
        if coordinate == 'camera':
            self.set_tcp(self.camtcp)
            t = self.get_pose()
            t.orient.rotate_zt(val)
            m = self.set_pose(t, wait=wait, acc=acc, vel=vel)
            self.set_tcp(self.tcp)
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
        self.movej(j)

    def rotate(self, rotaxis, rot_angles, coordinate='tcp', wait=True, acc=0.5, vel=0.5):
        # rotate around an axis by a relative amount.
        # rotaxis : rotation axis in the given coordination system.
        # rot_angles: rotation angle in degree
        # if you want to rotate around the tcp x axis with the tcp [0,0,0] as center:
        #   rob.rotate([1, 0, 0], 10)
        # if you want to rotate around the base x axis with the tcp [0,0,0] as center:
        #   t = rob.robot.get_pose()
        #   rob.rotate(t.orient.get_vect_x(), 10)
        if coordinate == 'tcp': # center of rotation is tcp.
            t = self.get_pose()
            t.orient.rotate_t(rotaxis, math.pi/180*rot_angles)
            return self.set_pose(t, acc=acc, vel=vel, wait=wait, command='movej', threshold=None)
        if coordinate == 'base': # center of rotation is the [0,0,0] of the base.
            t = m3d.Transform()
            t.orient.rotate_zt(math.pi/180*rot_angles)
            return self.add_pose_base(t, wait=wait, acc=acc, vel=vel)

    def rotate_ref(self, rotpos, rotaxis, rot_angles, acc=0.5, vel=0.5):
        # rotate around a given TCP position around an axis by a relative amount.
        # rotpos : center of rotation
        # rotaxis : rotation axis
        # rot_angles: rotation angle in degree
        t = self.get_pose()
        #rotaxis = -came*dir[0] + camn*dir[1]
        t.orient.rotate_t(rotaxis, math.pi/180*rot_angles)
        if type(rotpos) == m3d.transform.Transform:
            t.set_pos(rotpos.get_pos())
        else:
            t.set_pos(rotpos)
        #v = self.robot.get_pose()
        #v = v*t # rotate around tool z
        m = self.set_pose(t, acc=acc, vel=vel, wait=True, command='movej', threshold=None)
    
    def get_euler(self):
        # get euler angles in extrinsic 'xyz' rotation.
        ov = self.get_pose().orient
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
        return self.set_orientation_rad(t.orient.get_rotation_vector(), acc=0.5, vel=0.5)
# special functions

    def measureheight(self): # measure height by bumping along -z direction.
        back_up = 0.0
        v0 = self.get_xyz()
        self.bump(z=-1, backoff=back_up)
        v1 = self.get_xyz()
        v = v0[2]-(v1[2]-back_up)
        self.move2z(v0[2])
        #v1 = rob.get_xyz()
        #v = v0-v1
        #print(v0, v1)
        #distance = v[2]+0.01
        return v

######## How to use m3d.
# To rotate in the TCP frame,
# trans = self.get_pose()  # here trans represents the transformed TCP coordinate.
# To rate in the robot base frame,
# trans = m3d.Transform()  # make a new m3d object, 

# Then, trans.orient.rotate_xt(), rotate_yt(), rotate_zt(), or rotate_t(ax, angle)

# def ind2sub(ind, array_shape):
#     rows = int(ind / array_shape[1])
#     cols = (int(ind) % array_shape[1]) # or numpy.mod(ind.astype('int'), array_shape[1])
#     return (rows, cols)

# def sub2ind(rows, cols, array_shape):
#     return rows*array_shape[1] + cols

# This class add advanced methods to the UR_grip class.
# for example, combining camera, dashboard, and robot motion all together.
class UR_grip(UR):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

# Gripper functions
    def activate_gripper(self):
        if not hasattr(self, 'gripper'):
            raise NoFingerException('No gripper defined.')
        
        self.gripper.gripper_activate()

    def grab(self):
        if not hasattr(self, 'gripper'):
            raise NoFingerException('No gripper defined.')
        self.gripper.close_gripper()
        #self.gripper.gripper_action(255)

    def release(self):
        if not hasattr(self, 'gripper'):
            raise NoFingerException('No gripper defined.')
        self.gripper.gripper_action(120)

    def loosen(self):
        if not hasattr(self, 'gripper'):
            raise NoFingerException('No gripper defined.')
        self.gripper.gripper_action(190)

# This class add advanced methods to the UR_cam_grip class.
# for example, combining camera, dashboard, and robot motion all together.

class UR_cam_grip(UR_grip):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

# Camera functions.
    def is_camera_facedown(self):
        vec, _, _ = self.get_camera_vector()
        if abs(vec[2]) < 0.999:
            return False
        else:
            return True

    def rotz_camera2x(self):
        v = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        t = m3d.Orientation(v)
        t.set_pos(self.get_pos())
        self.set_pose(t, 0.5, 0.5, wait=True)

    def rotz_camera2y(self):
        v = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        t = m3d.Orientation(v)
        t.set_pos(self.get_pos())
        self.set_pose(t, 0.5, 0.5, wait=True)

    def tweak_around_camera_axis(self, ang, acc=0.5, vel=0.5):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')

        # rolling up and down around the tool X axis.
        if self.tweak_reference_axis_angle is None:
            vv = self.get_pose().orient
            t = vv.get_vec_y()
            l = math.sqrt(t[0]**2+t[1]**2)
            rotang = math.acos(t[1]/l)*180/math.pi
            self.tweak_reference_axis_angle = rotang
        if ang ==0:
            return
        self.set_tcp(self.camtcp)
        self.rotz(ang)
        self.set_tcp(self.tcp)
        vv = self.get_pose().orient
        t = vv.get_vec_y()
        l = math.sqrt(t[0]**2+t[1]**2)
        rotang = math.acos(t[1]/l)*180/math.pi-self.tweak_reference_axis_angle
        self.tweak_axis_angle = rotang
        print(f"XY plane is rotated {rotang} degree from the reference direction.")

    def undo_tweak_around_camera_axis(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')

        if not hasattr(self, 'tweak_axis_angle'):
            return
        if self.tweak_reference_axis_angle != None:
            self.tweak_around_camera_axis(self.tweak_axis_angle)
            self.tweak_reference_axis_angle = None

    def capture_camera(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        
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
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')

        # tilt around a point at a "distance" away from the camera along the camera axis.
        # dir: [1, 0] = north direction on the camera feed.
        # dir: [0, 1] = east direction on the camera feed. 
        #yv = self.get_camera_vector()
        camv, camn, came = self.get_camera_vector()
        CAMVECTOR_ToolCoordinate = [0, math.sin(math.pi/6), math.cos(math.pi/6)]
        self.set_tcp(self.tcp)
        #xv = self.robot.get_orientation().get_vec_x()
        #zv = [0, -1, 0]
        rotcenter = self.camtcp.copy()
        for i in range(0, 3):
            rotcenter[i] = rotcenter[i]+distance*CAMVECTOR_ToolCoordinate[i]
        self.set_tcp(rotcenter)
        t = self.get_pose()
        rotaxis = -came*dir[0] + camn*dir[1]
        t.orient.rotate_b(rotaxis, math.pi/180*ang)
        #v = self.get_pose()
        #v = v*t # rotate around tool z
        m = self.set_pose(t, acc=acc, vel=vel, wait=True, command='movej', threshold=None)
        self.set_tcp(self.tcp)
        return m

    def tilt_over_back(self, distance=0, ang = 30):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')

        return self.tilt_over(distance=distance, ang = ang, dir=[-1, 0])

    def camera_face_down(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        if self.is_Z_aligned():
            return self.rotx(30, coordinate='camera')

    def camera_face_back(self):
        if self.is_camera_facedown():
            return self.rotx(-30, coordinate='camera')

    def move_over_camera(self, distance, acc=0.5, vel=0.5):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        # move along the camera north direction.
        cameravector = self.get_pose().orient.get_vec_y()
        cameravector = cameravector*distance
        rotv = self.get_xyz().tolist()
        v = [rotv[0]+cameravector[0],rotv[1]+cameravector[1],rotv[2]+cameravector[2],rotv[3],rotv[4],rotv[5]]
        self.moveto(v, acc=acc, vel=vel)

    def get_camera_position(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        self.set_tcp(self.camtcp)
        t = self.get_pose()
        self.set_tcp(self.tcp)
        return t

    def get_camera_vector(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        cameravector_east = self.get_pose().orient.get_vec_x()
        v = self.get_pose().orient
        v.rotate_b(cameravector_east, math.pi/3)
        cameravector = v.get_vec_y()
        v = self.get_pose().orient
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
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
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
        self.set_pose(np, acc=acc, vel=vel, wait=True, command="movej", threshold=None)

    def roll_around_camera(self, val, distance, dir='y'):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        newtcp = []
        for v in self.camtcp:
            newtcp.append(v)
        newtcp[2] = distance
        self.set_tcp(newtcp)
        v = self.get_pose().orient
        if dir=='y':
            self.roty(val, coordinate='tcp', acc=0.5, vel=0.5)
        else:
            self.rotx(val, coordinate='tcp', acc=0.5, vel=0.5)
        self.set_tcp(self.tcp)

    def camera_y(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        trans = self.get_pose()
        pos = trans.get_pos()
        if pos[1] < 0:
            orient = m3d.Orientation([math.pi/2, 0, 0]) # make camera point +y axis.
            orient.rotate_xb(math.pi/6)
        else:
            orient = m3d.Orientation([-math.pi/2, 0, 0]) # make camera point +y axis.
            orient.rotate_xb(-math.pi/6)
        #orient.rotate_zb(math.pi/2) # make camera point +x
        
        trans.orient = orient
        self.set_pose(trans, acc=0.1, vel=0.1)

    def camera_x(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        trans = self.get_pose()
        pos = trans.get_pos()
        if pos[0] < 0:
            orient = m3d.Orientation([0, -math.pi/2, 0]) # make camera point +y axis.
            orient.rotate_yb(-math.pi/6)
        else:
            orient = m3d.Orientation([0, math.pi/2, 0]) # make camera point +y axis.
            orient.rotate_yb(math.pi/6)
        #orient.rotate_zb(math.pi/2) # make camera point +x
        trans.orient = orient
        self.set_pose(trans, acc=0.1, vel=0.1)

    def camera_out(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
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

        self.movel(np0, acc=0.5, vel=0.5)

    def rotate_around_Zaxis_camera(self, ang):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        # rotate around the camera axis, degree input.
        self.set_tcp(self.camtcp)
        self.rotz(ang)
        self.set_tcp(self.tcp)

    def get_inplane_angle_from_idealZ(self):
        t = self.get_pose()
        zv = t.orient.get_vec_z()
        ang = math.atan2(zv[1], zv[0])+math.pi/2
        return ang

    # putting the tooltip to the current camera position
    def grippertip2camera(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        self.prev_tcp = self.get_tcp()
        self.set_tcp(self.camtcp)
        pose = self.get_pose()
        self.set_tcp(self.tcp)
        self.set_pose(pose, vel=0.2, acc=0.1)
        self.set_tcp(self.prev_tcp)

    # putting the camera to the current tooltip position
    def camera2grippertip(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        self.prev_tcp = self.get_tcp()
        self.set_tcp(self.tcp)
        pose = self.get_pose()
        self.set_tcp(self.camtcp)
        self.set_pose(pose, vel=0.2, acc=0.1)
        self.set_tcp(self.prev_tcp)
    
    # special case of grippertip2camera. This is to make the gripper tip face down.
    def put_tcp2camera(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        self.set_tcp(self.camtcp)
        pos = self.get_pos()
        self.set_tcp(self.tcp)
        self.tilt_back()
        self.set_pos(pos, acc=0.1, vel=0.1)

    # special case of camera2grippertip. This is to make the camera face down.
    def put_camera2tcp(self):
        if not hasattr(self, 'camera'):
            raise NoCameraException('No camera defined.')
        pos = self.get_pos()
        self.tilt_camera_down()
        self.set_tcp(self.camtcp)
        self.set_pos(pos, acc=0.1, vel=0.1)
        self.set_tcp(self.tcp)

