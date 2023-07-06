
import logging
import os
import time

from urx.urscript import URScript

SOCKET_HOST = "127.0.0.1"
SOCKET_PORT = 29999
SOCKET_NAME = "popup"


class PopScript(URScript):

    def __init__(self,
                 socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        super(PopScript, self).__init__()

        # Reset connection to gripper
        self._socket_close(self.socket_name)
        self._socket_open(self.socket_host,
                          self.socket_port,
                          self.socket_name)
    def _close_popup(self):
        self._socket_send_string("close popup", self.socket_name)
        self._socket_send_byte(10, self.socket_name)

    def _unlock_protective_stop(self):
        self._socket_send_string("unlock protective stop", self.socket_name)
        self._socket_send_byte(10, self.socket_name)

    def _sock_close(self):
        self._socket_close(self.socket_name)

class Popinfo(object):

    def __init__(self,
                 robot,
                 socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        self.robot = robot
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        self.logger = logging.getLogger(u"Popup")


    def _get_close_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = PopScript(socket_host=self.socket_host,
                                 socket_port=self.socket_port,
                                 socket_name=self.socket_name)
        urscript._close_popup()
        urscript._sock_close()

        return urscript

    def _get_unlock_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = PopScript(socket_host=self.socket_host,
                                 socket_port=self.socket_port,
                                 socket_name=self.socket_name)
        urscript._unlock_protective_stop()
        urscript._sock_close()

        return urscript
        
    def close_popup(self):
        urscript = self._get_close_urscript()

        # Move to the position
        sleep = 1.0
        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)
    def unlock(self):
        urscript = self._get_unlock_urscript()

        # Move to the position
        sleep = 1.0
        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(sleep)