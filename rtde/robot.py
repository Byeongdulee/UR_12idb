''' This program is to define UR robot with Robotiq gripper and camera '''
#import time
import sys

from robotiq_gripper_control import RobotiqGripper

class RobotException(Exception):
    pass

# UR3
import math3d as m3d
import numpy as np
import logging
import math
import time
import os
text_file_path = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(text_file_path, '..', 'urscripts', 'checkdistance.script'), 'r') as file:
    CheckdistanceScript = file.read()
sys.path.append(os.path.join(text_file_path, '..', 'common'))

from robUR import SafetyMode


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
#from .robotiq_gripper_control import RobotiqGripper
######## How to use m3d.
# To rotate in the TCP frame,
# trans = self.robot.get_pose()  # here trans represents the transformed TCP coordinate.
# To rate in the robot base frame,
# trans = m3d.Transform()  # make a new m3d object, 

# Then, trans.orient.rotate_xt(), rotate_yt(), rotate_zt(), or rotate_t(ax, angle)

class Robot():
    # unit of position vector : meter.
    _TCP2CAMdistance = 0.12
    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]
    camtcp = [0, 0.04, 0.015, -math.pi/180*30, 0, 0]

    def __init__(self, name = 'UR3', use_rtde=True):
        super().__init__()
        # grippertype:
        #   0: No gripper
        #   1: Robotiq gripper
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
        if use_rtde:
            self.rr = rr.RTDEReceiveInterface(IP)
        if SafetyMode(self.get_safety_mode()) is not SafetyMode.IS_NORMAL_MODE:
            print("Robot is not at NORMAL_MODE.")
        else:
            self.rc = rc.RTDEControlInterface(IP)
            self.set_tcp(self.tcp)
            self.set_payload(1.35, (-0.003,0.01,0.037))
        #self.rio = rio.RTDEIOInterface(IP)

        #self.__TCP2CAMdistance = 0.15
        self.IP = IP
    
    def get_safety_mode(self):
        return self.rr.getSafetyMode()

    def terminate(self):
        self.rc.stopScript()
        self.rc.disconnect()
        self.rr.disconnect()
#        self.rio.disconnect()

    def set_csys(self, transform):
        """
        Set reference coordinate system to use
        """
        self.csys = transform

    def get_tcp(self):
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        pose = self.rc.getTCPOffset()
        return pose

    def set_payload(self, weight, dir):
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        return self.rc.setPayload(weight, dir)

    def set_tcp(self, tcp):
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        return self.rc.setTcp(tcp)

    def bump(self, x=0, y=0, z=0, backoff=0, wait=True):
        #data = CheckdistanceScript
        data = CheckdistanceScript.replace('__replace__', f'[{x}, {y}, {z}, 0, 0, 0]')
        data = data.replace('__backoff__', f'{backoff}')
        data = data.replace('__rep_force__', '0')
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        self.rc.sendCustomScript(data)
        while not self.rr.is_program_running():
            time.sleep(0.01)
        if wait:
            while self.rr.is_program_running():
                time.sleep(0.01)
        self.rc.stopScript()

    def screw(self, z=0, rz=0, forcelimit=10, backoff=0, wait=True):
        #data = CheckdistanceScript
        data = CheckdistanceScript.replace('__replace__', f'[0, 0, {z}, 0, 0, {rz}]')
        data = data.replace('__backoff__', f'{backoff}')
        data = data.replace('__rep_force__', f'{forcelimit}')
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        self.rc.sendCustomScript(data)
        while not self.rr.is_program_running():
            time.sleep(0.01)
        if wait:
            while self.rr.is_program_running():
                time.sleep(0.01)
        self.rc.stopScript()

    def write_output(self, address=0, val=0, wait=False):
        """
        write digital value
        """
        address = str(address)
#        val = str(val)
        if isinstance(val, float):
            myprogram = "write_output_float_register(%s, %s)\n"%(address, str(val))
            #myprogram = "def script_test():\n\twrite_output_float_register(%s, %s)\nend\nrun program\n"%(address, str(val))
        if isinstance(val, int):
            myprogram = "def script_test():\n\twrite_output_integer_register(%s, %s)\nend\nrun program\n"%(address, str(val))
        self.rc.sendCustomScript(myprogram)
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
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
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
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        self.rc.moveL(tpose, acceleration=acc, speed=vel, asynchronous=not wait)
        if wait:
            pose = self.get_pose()
            if pose is not None:
                    return self.csys.inverse * m3d.Transform(pose)

    
    def movej(self, q, vel=0.1, acc=0.1, wait=True):
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        self.rc.moveJ(q, acceleration=acc, speed=vel, asynchronous=not wait)
    
    def is_protective_stopped(self):
        return self.rr.isProtectiveStopped()   

    def is_running(self):
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
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
        if not hasattr(self, 'rc'):
            raise RobotException("Robot is not at NORMAL_MODE.")
        self.rc.moveUntilContact(speed)
        self.rc.stopScript()

    def gripper_action(self, value):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        urscript = self._get_urscript()

        # Move to the position
        sleep = 2.0
        urscript._set_gripper_position(value)
        urscript._sleep(sleep)

        # Send the script
#        print(urscript())
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)

    def set_gripper_force(self, force):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        self.force = force
        urscript = self._get_urscript()
        urscript._set_gripper_force(self.force)
        # Move to the position
        sleep = 2.0
        urscript._sleep(sleep)

        # Send the script
#        print(urscript())
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)

    def _get_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript(self.socket_host,
                                    self.socket_port,
                                    self.socket_name)

        urscript._sleep(0.1)

        return urscript

    def _get_finger_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript(self.socket_host,
                                    self.socket_port,
                                    self.socket_name)
        # Wait on activation to avoid USB conflicts
    #        urscript._sleep(0.1)

        return urscript
    
    def _get_new_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript(self.socket_host,
                                    self.socket_port,
                                    self.socket_name)

        # Set input and output voltage ranges
        urscript._set_analog_inputrange(0, 0)
        urscript._set_analog_inputrange(1, 0)
        urscript._set_analog_inputrange(2, 0)
        urscript._set_analog_inputrange(3, 0)
        urscript._set_analog_outputdomain(0, 0)
        urscript._set_analog_outputdomain(1, 0)
        urscript._set_tool_voltage(0)
        urscript._set_runstate_outputs()

        # Set payload, speed and force
        urscript._set_payload(self.payload)
        urscript._set_gripper_speed(self.speed)
        urscript._set_gripper_force(self.force)

        # Initialize the gripper
        urscript._set_robot_activate()
        urscript._set_gripper_activate()

        # Wait on activation to avoid USB conflicts
        urscript._sleep(0.1)

        return urscript
    
    def gripper_activate(self):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        # When Robotiq wrist camera is used with Robotiq handE gripper,
        # urscript._set_tool_voltage(0) may be needed. See urx/robotiq_two_finger_gripper/_get_new_urscript
        # lines 170 ~ 176.
        urscript = self._get_new_urscript()

        # Move to the position
        sleep = 2.0
        urscript._set_gripper_position(0)
        urscript._sleep(sleep)

        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)

    def get_position(self):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        urscript = self._get_finger_urscript()
        urscript._sleep(0.1)
  
        # Move to the position
        urscript._get_gripper_position()
        urscript._sync()
        urscript._sleep(0.1)
        self.robot.send_program(urscript())
    #        time.sleep(1)
        time.sleep(0.3)
        data = self.robot.secmon.get_all_data()
#        print(data['MasterBoardData']['analogOutput0'])
        #print(self.robot.rtmon.state.output_double_register_0)
        
        try:
            output = (data['MasterBoardData']['analogOutput0']*254)/5
        except:
            output = -1
        return output
    
    def open_gripper(self):
        self.gripper_action(0)

    def close_gripper(self):
        self.gripper_action(255)

class RobotiqGripper(RobotiqGripper):
    def __init__(self,*args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def gripper_action(self, value):
        """
        Activate the gripper to a given value from 0 to 255
        
        0 is open
        255 is closed
        """
        value = value/255*60
        return self.move(self, value)


    def set_gripper_force(self, force):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        force = force/255*100
        return self.set_force(force)
    
    def gripper_activate(self):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        return self.activate()

    def get_position(self):
        """
        Activate the gripper to a given value from 0 to 255

        0 is open
        255 is closed
        """
        urscript = self._get_finger_urscript()
        urscript._sleep(0.1)
  
        # Move to the position
        urscript._get_gripper_position()
        urscript._sync()
        urscript._sleep(0.1)
        self.robot.send_program(urscript())
    #        time.sleep(1)
        time.sleep(0.3)
        data = self.robot.secmon.get_all_data()
#        print(data['MasterBoardData']['analogOutput0'])
        #print(self.robot.rtmon.state.output_double_register_0)
        
        try:
            output = (data['MasterBoardData']['analogOutput0']*254)/5
        except:
            output = -1
        return output
    
    def open_gripper(self):
        return self.open()

    def close_gripper(self):
        return self.close()  