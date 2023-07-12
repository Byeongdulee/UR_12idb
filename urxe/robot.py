import sys
import os
text_file_path = os.path.dirname(os.path.abspath(__file__))
# get the python-urx from below:
# https://github.com/Byeongdulee/python-urx
sys.path.append(os.path.join(text_file_path, '..', '..', 'python-urx'))

with open(os.path.join(text_file_path, 'urscripts', 'checkdistance.script'), 'r') as file:
    CheckdistanceScript = file.read()

from urx import robot, urrobot, robotiq_two_finger_gripper
from urxe import ursecmon, urrtmon
#import urmon_parser
#import urrtde
import math3d as m3d
import logging
import time
import numpy as np

class URRobot(urrobot.URRobot):

    """
    Python interface to socket interface of UR robot.
    programs are send to port 3002
    data is read from secondary interface(10Hz?) and real-time interface(125Hz) (called Matlab interface in documentation)
    Since parsing the RT interface uses som CPU, and does not support all robots versions, it is disabled by default
    The RT interfaces is only used for the get_force related methods
    Rmq: A program sent to the robot i executed immendiatly and any running program is stopped
    """

    def __init__(self, host, use_rt=False, urFirm=None):
        self.logger = logging.getLogger("urx")
        self.host = host
        self.urFirm = urFirm
        self.csys = None

        self.logger.debug("Opening secondary monitor socket")
        self.secmon = ursecmon.SecondaryMonitor(self.host)  # data from robot at 10Hz

        self.rtmon = None
        if use_rt:
            self.rtmon = self.get_realtime_monitor()
        # precision of joint movem used to wait for move completion
        # the value must be conservative! otherwise we may wait forever
        self.joinEpsilon = 0.01
        # It seems URScript is  limited in the character length of floats it accepts
        self.max_float_length = 6  # FIXME: check max length!!!

        self.secmon.wait()  # make sure we get data from robot before letting clients access our methods

class Robot(robot.Robot):
    def __init__(self, host, use_rt=False, urFirm=None) -> None:
        URRobot.__init__(self, host, use_rt=use_rt, urFirm=urFirm)
        self.csys = m3d.Transform()
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger("myrobot")
        self.urFirm = (5.9)
        #self.secmon = ursecmon.SecondaryMonitor(self.host)  # data from robot at 10Hz
        #self.rtmon = urrtde.URRTMonitor(self.host)
        #self.rtmon = urrtmon.URRTMonitor(self.host, self.urFirm)
        #self.rtmon.start()

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
        v = trans.get_pose_vector()
        trans.invert()
        if not isinstance(pos, m3d.Transform):
            pos = m3d.Transform([pos[0], pos[1], pos[2], v[3], v[4], v[5]])
        n = pos*trans
        return n

    def get_tcp(self):
        pose = self.secmon.get_tcp()
        return pose

    def set_tcp(self, tcp):
        """
        set robot flange to tool tip transformation
        """
        if isinstance(tcp, m3d.Transform):
            tcp = tcp.pose_vector
        URRobot.set_tcp(self, tcp)
        _tcp = [0, 0, 0, 0, 0, 0]
        while not (np.round(np.array(_tcp), 5) == np.round(np.array(tcp), 5)).all():
            _tcp = self.get_tcp()

    def bump(self, x=0, y=0, z=0, backoff=0, wait=True):
        #data = CheckdistanceScript
        data = CheckdistanceScript.replace('__replace__', f'[{x}, {y}, {z}, 0, 0, 0]')
        data = data.replace('__backoff__', f'{backoff}')
        self.send_program(data)
        while not self.is_program_running():
            time.sleep(0.01)
        if wait:
            while self.is_program_running():
                time.sleep(0.01)

class RobotiqScript12ID(robotiq_two_finger_gripper.RobotiqScript): 
    # should make a variable named "rq_pos" on the teach pendent to be able to run this.
    def __init__(self, host, port, sname):
       super(RobotiqScript12ID, self).__init__(socket_host=host, socket_port=port, socket_name=sname)

    def _get_gripper_position(self):
        self._socket_get_var2var(robotiq_two_finger_gripper.POS, 'rq_pos ', self.socket_name, prefix='global ')
        self._sync()
        msg = "set_analog_out({}, 1-rq_pos/255)".format(0)
        self.add_line_to_program(msg)
        msg = "write_output_float_register(0, rq_pos)"
        self.add_line_to_program(msg)
        self.add_line_to_program('textmsg("gripper=", rq_pos)')


SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 63352
SOCKET_NAME = "gripper_socket"

class Robotiq_Two_Finger_Gripper12ID(object):

    def __init__(self,robot, payload=0.85, speed=255, force=150, socket_host=SOCKET_HOST,socket_port=SOCKET_PORT, socket_name=SOCKET_NAME):
        self.robot = robot
        self.payload = payload
        self.speed = speed
        self.force = force
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name

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
        urscript = RobotiqScript12ID(self.socket_host,
                                    self.socket_port,
                                    self.socket_name)

        urscript._sleep(0.1)

        return urscript

    def _get_finger_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript12ID(self.socket_host,
                                    self.socket_port,
                                    self.socket_name)
        # Wait on activation to avoid USB conflicts
    #        urscript._sleep(0.1)

        return urscript
    
    def _get_new_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript12ID(self.socket_host,
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
            output = (1-data['MasterBoardData']['analogOutput0'])*10
        except:
            output = -1
        return output
    
    def open_gripper(self):
        self.gripper_action(0)

    def close_gripper(self):
        self.gripper_action(255)