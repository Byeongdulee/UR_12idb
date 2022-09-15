import sys
# get the python-urx from below:
# https://github.com/Byeongdulee/python-urx
sys.path.append(r"python-urx")
sys.path.append(r"../python-urx")
from urx import robot, urscript, urrobot, robotiq_two_finger_gripper
import urrtde
import math3d as m3d
import logging
import time

class Robot(robot.Robot):
    def __init__(self, host) -> None:
        urrobot.URRobot.__init__(self, host, use_rt=False, urFirm=None)
        self.csys = m3d.Transform()
        FORMAT = '%(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger("myrobot")

        #self.secmon = ursecmon.SecondaryMonitor(self.host)  # data from robot at 10Hz
        self.rtmon = urrtde.URRTMonitor(self.host)
        #self.rtmon = urrtmon.URRTMonitor(self.host)
        self.rtmon.start()

class URScript(urscript.URScript):
    def __init__(self):
        super().__init__(self)
    
    def _socket_get_var2var(self, var, varout, socket_name, prefix = ''):
        msg = "{}{} = socket_get_var(\"{}\",\"{}\")".format(prefix, varout, var, socket_name)
        self.add_line_to_program(msg)

    def _socket_send_byte(self, byte, socket_name):
        msg = "socket_send_byte(\"{}\",\"{}\")".format(str(byte), socket_name)  # noqa
        self.add_line_to_program(msg)
        self._sync()

# class URrobot(urrobot.URRobot):
#     def __init__(self, host):
#         FORMAT = '%(message)s'
#         logging.basicConfig(format=FORMAT)
#         self.logger = logging.getLogger("myrobot")

#         self.host = host
#         self.csys = None
#         self.secmon = ursecmon.SecondaryMonitor(self.host)  # data from robot at 10Hz
#         self.rtmon = urrtde.URRTMonitor(self.host)
#         self.rtmon.start()


class RobotiqScript12ID(robotiq_two_finger_gripper.RobotiqScript): 
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

class Robotiq_Two_Finger_Gripper(robotiq_two_finger_gripper.Robotiq_Two_Finger_Gripper):

    def __init__(self,
                 robot,
                 payload=0.85,
                 speed=255,
                 force=50,
                 socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        super().__init__(robot,
                 payload=0.85,
                 speed=255,
                 force=50,
                 socket_host=socket_host,
                 socket_port=socket_port,
                 socket_name=socket_name)

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
    #        data = self.robot.secmon.get_all_data()
    #        return data['MasterBoardData']['analogOutput0']
        time.sleep(0.3)
        try:
            output = (1-self.robot.rtmon.state.output_double_register_0/255)*10
        except:
            output = -1
        return output
