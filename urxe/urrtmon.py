'''
Module for implementing a UR controller real-time monitor over socket port 30003.
Confer http://support.universal-robots.com/Technical/RealTimeClientInterface
Note: The packet lenght given in the web-page is 740. What is actually received from the controller is 692. 
It is assumed that the motor currents, the last group of 48 bytes, are not send.
Originally Written by Morten Lind
Parsing for Firmware 5.9 is added by Byeongdu Lee
'''
import logging
import socket
import struct
import time
import threading
from copy import deepcopy

import numpy as np

from common import m3d  # centralized math3d (4.x compat applied in common.m3d)

__author__ = "Byeongdu Lee"
__copyright__ = "Copyright 2022, Argonne National Laboratory"
__credits__ = ["Updated urx by Morten Lind, Olivier Roulet-Dubonnet for a newer OS"]
__license__ = "LGPLv3"


class RTMonitorTimeout(RuntimeError):
    """No realtime packet arrived within the wait timeout.

    Its own class so a caller can tell "the realtime link has stalled" from
    the RobotExceptions that mean the robot refused or failed a move. The
    remedy differs: this one needs the connection re-established, not the
    motion retried.
    """


class URRTMonitor(threading.Thread):

    # Struct for revision of the UR controller giving 692 bytes
    rtstruct692 = struct.Struct('>d6d6d6d6d6d6d6d6d18d6d6d6dQ')

    # for revision of the UR controller giving 540 byte. Here TCP
    # pose is not included!
    rtstruct540 = struct.Struct('>d6d6d6d6d6d6d6d6d18d')

    rtstruct5_1 = struct.Struct('>d1d6d6d6d6d6d6d6d6d6d6d6d6d6d6d1d6d1d1d1d6d1d6d3d6d1d1d1d1d1d1d1d6d1d1d3d3d')
    rtstruct5_9 = struct.Struct('>d6d6d6d6d6d6d6d6d6d6d6d6d6d6d1d6d1d1d1d6d1d6d3d6d1d1d1d1d1d1d1d6d1d1d3d3d1d')

    def __init__(self, urHost, urFirm=None):
        threading.Thread.__init__(self)
        self.logger = logging.getLogger(self.__class__.__name__)
        self.daemon = True
        self._stop_event = True
        self._dataEvent = threading.Condition()
        self._dataAccess = threading.Lock()
        self._rtSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._rtSock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._urHost = urHost
        self.urFirm = urFirm
        # Package data variables
        self._timestamp = None
        self._ctrlTimestamp = None
        self._qActual = None
        self._qTarget = None
        self._tcp = None
        self._tcp_force = None
        self._joint_temperature = None
        self._joint_voltage = None
        self._joint_current = None
        self._main_voltage = None
        self._robot_voltage = None
        self._robot_current = None
        self._qdTarget = None
        self._qddTarget = None
        self._iTarget = None
        self._mTarget = None
        self._qdActual = None
        self._tcp_speed = None
        self._tcp = None
        self._robot_mode = None
        self._safety_mode = None
        self._joint_modes = None
        self._digital_outputs = None
        self._program_state = None
        self._safety_status = None

        self.__recvTime = 0
        self._last_ctrl_ts = 0
        # self._last_ts = 0
        self._buffering = False
        self._buffer_lock = threading.Lock()
        self._buffer = []
        self._csys = None
        self._csys_lock = threading.Lock()

    def set_csys(self, csys):
        with self._csys_lock:
            self._csys = csys

    def __recv_bytes(self, nBytes):
        ''' Facility method for receiving exactly "nBytes" bytes from
        the robot connector socket.'''
        # Record the time of arrival of the first of the stream block
        recvTime = 0
        pkg = b''
        while len(pkg) < nBytes:
            chunk = self._rtSock.recv(nBytes - len(pkg))
            if not chunk:
                # Peer closed the connection: recv returns b'' forever after
                # that, so without this the loop would spin. Raise instead, so
                # run() reconnects.
                raise ConnectionError("30003 realtime socket closed by the controller")
            pkg += chunk
            if recvTime == 0:
                recvTime = time.time()
        self.__recvTime = recvTime
        return pkg

    #: How long any wait=True getter will sit for a fresh realtime packet
    #: before giving up. The controller streams at 125 Hz, so a healthy link
    #: delivers one every 8 ms; a second is three orders of magnitude of
    #: slack and still fails fast enough to be reported rather than endured.
    WAIT_TIMEOUT_S = 1.0

    def wait(self, timeout=None):
        """Block until the next realtime packet is parsed.

        Raises RTMonitorTimeout rather than waiting forever. The unbounded
        version wedged every caller permanently whenever the controller
        stopped feeding this socket while still holding it open: the receive
        thread sits in recv() looking healthy, nothing ever reaches
        notifyAll(), and the waiter never returns.

        That is not a theoretical failure. robUR.movel/movej/movels each call
        get_safety_mode() -- which lands here -- *before* sending a move, so a
        stalled socket silently swallowed every motion in the system, with no
        error, no timeout and the robot never twitching. A caller that gets an
        exception can report it, retry, or reconnect; one that blocks forever
        can do none of those.
        """
        if timeout is None:
            timeout = self.WAIT_TIMEOUT_S
        with self._dataEvent:
            if not self._dataEvent.wait(timeout):
                raise RTMonitorTimeout(
                    "No realtime packet from the UR controller in %.1f s. The "
                    "socket on port 30003 is open but not delivering; the "
                    "connection needs re-establishing." % timeout)

    def q_actual(self, wait=False, timestamp=False):
        """ Get the actual joint position vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qActual
            else:
                return self._qActual
    getActual = q_actual

    def qd_actual(self, wait=False, timestamp=False):
        """ Get the actual joint velocity vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qdActual
            else:
                return self._qdActual

    def q_target(self, wait=False, timestamp=False):
        """ Get the target joint position vector."""
        if wait:
            self.wait()
        with self._dataAccess:
            if timestamp:
                return self._timestamp, self._qTarget
            else:
                return self._qTarget
    getTarget = q_target

    def tcp_pose(self, wait=False, timestamp=False, ctrlTimestamp=False):
        """ Return the tool pose values."""
        if wait:
            self.wait()
        with self._dataAccess:
            tcf = self._tcp
            if ctrlTimestamp or timestamp:
                ret = [tcf]
                if timestamp:
                    ret.insert(-1, self._timestamp)
                if ctrlTimestamp:
                    ret.insert(-1, self._ctrlTimestamp)
                return ret
            else:
                return tcf
    getTCP = tcp_pose

    def tcp_force(self, wait=False, timestamp=False):
        """ Get the tool force. The returned tool force is a
        six-vector of three forces and three moments."""
        if wait:
            self.wait()
        with self._dataAccess:
            # tcf = self._fwkin(self._qActual)
            tcp_force = self._tcp_force
            if timestamp:
                return self._timestamp, tcp_force
            else:
                return tcp_force
    getTCPForce = tcp_force

    def joint_temperature(self, wait=False, timestamp=False):
        """ Get the joint temperature."""
        if wait:
            self.wait()
        with self._dataAccess:
            joint_temperature = self._joint_temperature
            if timestamp:
                return self._timestamp, joint_temperature
            else:
                return joint_temperature
    getJOINTTemperature = joint_temperature

    def joint_voltage(self, wait=False, timestamp=False):
        """ Get the joint voltage."""
        if wait:
            self.wait()
        with self._dataAccess:
            joint_voltage = self._joint_voltage
            if timestamp:
                return self._timestamp, joint_voltage
            else:
                return joint_voltage
    getJOINTVoltage = joint_voltage

    def joint_current(self, wait=False, timestamp=False):
        """ Get the joint current."""
        if wait:
            self.wait()
        with self._dataAccess:
            joint_current = self._joint_current
            if timestamp:
                return self._timestamp, joint_current
            else:
                return joint_current
    getJOINTCurrent = joint_current

    def main_voltage(self, wait=False, timestamp=False):
        """ Get the Safety Control Board: Main voltage."""
        if wait:
            self.wait()
        with self._dataAccess:
            main_voltage = self._main_voltage
            if timestamp:
                return self._timestamp, main_voltage
            else:
                return main_voltage
    getMAINVoltage = main_voltage

    def robot_voltage(self, wait=False, timestamp=False):
        """ Get the Safety Control Board: Robot voltage (48V)."""
        if wait:
            self.wait()
        with self._dataAccess:
            robot_voltage = self._robot_voltage
            if timestamp:
                return self._timestamp, robot_voltage
            else:
                return robot_voltage
    getROBOTVoltage = robot_voltage

    def robot_current(self, wait=False, timestamp=False):
        """ Get the Safety Control Board: Robot current."""
        if wait:
            self.wait()
        with self._dataAccess:
            robot_current = self._robot_current
            if timestamp:
                return self._timestamp, robot_current
            else:
                return robot_current
    getROBOTCurrent = robot_current

    #: Realtime packet size -> controller firmware, for when the caller did
    #: not say. Sizes are what the controller actually puts on the wire and
    #: grow with each generation, so they identify it without asking: a CB3
    #: on 3.x sends 692 (or 540 on older), an e-Series on 5.x sends 1108+.
    #: PolyScope 5.26 sends 1452.
    _FIRMWARE_BY_PKGSIZE = ((1108, 5.9), (692, 3.1), (540, 3.0))

    def _firmware_for(self, pkgsize):
        """The firmware to parse this packet as. None if it is too short.

        urFirm when the caller supplied one, otherwise inferred from the
        packet. Inferring matters because robUR builds Robot() without a
        urFirm, which used to send every packet down the `else` branch and
        parse a 1452-byte 5.x packet with the 86-field 3.x layout -- then read
        unp[101] off it. That raised IndexError on the first packet of every
        session, killing the receive thread before it ever reached
        notifyAll(), which is what made get_safety_mode() -- and so every
        move, since movel/movej/movels all call it first -- hang forever.
        """
        if self.urFirm is not None:
            return self.urFirm
        for size, firm in self._FIRMWARE_BY_PKGSIZE:
            if pkgsize >= size:
                return firm
        return None

    def __recv_rt_data(self):
        head = self.__recv_bytes(4)
        # Record the timestamp for this logical package
        timestamp = self.__recvTime
        pkgsize = struct.unpack('>i', head)[0]
        self.logger.debug(
            'Received header telling that package is %s bytes long',
            pkgsize)
        payload = self.__recv_bytes(pkgsize - 4)
        firm = self._firmware_for(pkgsize)
        if firm is None:
            self.logger.warning(
                'Error, Received packet of length smaller than 540: %s ', pkgsize)
            return
        if firm >= 5.1:
            # 5.1 and 5.9 share a size; 5.9 adds fields the 5.1 layout stops
            # short of, and reading a 5.1 controller with it only costs
            # trailing values nothing asks for on that firmware.
            struct_ = self.rtstruct5_9 if firm >= 5.9 else self.rtstruct5_1
        elif firm >= 3.1:
            struct_ = self.rtstruct692
        else:
            struct_ = self.rtstruct540
        if struct_.size > len(payload):
            self.logger.warning(
                'Packet of %s bytes is too short for the %s layout (%s bytes); '
                'ignoring it.', pkgsize, firm, struct_.size)
            return
        unp = struct_.unpack(payload[:struct_.size])


        with self._dataAccess:
            self._timestamp = timestamp
            # it seems that packet often arrives packed as two... maybe TCP_NODELAY is not set on UR controller??
            # if (self._timestamp - self._last_ts) > 0.010:
            # self.logger.warning("Error the we did not receive a packet for {}s ".format( self._timestamp - self._last_ts))
            # self._last_ts = self._timestamp
            self._ctrlTimestamp = unp[0]
            if self._last_ctrl_ts != 0 and (
                    self._ctrlTimestamp -
                    self._last_ctrl_ts) > 0.010:
                self.logger.warning(
                    "Error the controller failed to send us a packet: time since last packet %s s ",
                    self._ctrlTimestamp - self._last_ctrl_ts)
            self._last_ctrl_ts = self._ctrlTimestamp
            self._qActual = np.array(unp[31:37])
            self._qdActual = np.array(unp[37:43])
            self._qTarget = np.array(unp[1:7])
            self._tcp_force = np.array(unp[67:73])
            self._tcp = np.array(unp[73:79])            
            self._joint_current = np.array(unp[43:49])
            self._safety_mode = unp[101]
            if firm >= 3.1:
                self._joint_temperature = np.array(unp[86:92])
                self._joint_voltage = np.array(unp[124:130])
                self._main_voltage = unp[121]
                self._robot_voltage = unp[122]
                self._robot_current = unp[123]

            if firm >= 5.9:
                self._qdTarget = np.array(unp[7:13])
                self._qddTarget = np.array(unp[13:19])
                self._iTarget = np.array(unp[19:25])
                self._mTarget = np.array(unp[25:31])
                self._tcp_speed = np.array(unp[61:67])
                self._joint_current = np.array(unp[49:55])
                self._joint_voltage = np.array(unp[124:130])
                self._robot_mode = unp[94]
                self._joint_modes = np.array(unp[95:101])
                self._digital_outputs = unp[130]
                self._program_state = unp[131]
                self._safety_status = unp[138]

            if self._csys:
                with self._csys_lock:
                    # might be a godd idea to remove dependancy on m3d
                    tcp = self._csys.inverse * m3d.Transform(self._tcp)
                self._tcp = tcp.pose_vector
        if self._buffering:
            with self._buffer_lock:
                self._buffer.append(
                    (self._timestamp,
                     self._ctrlTimestamp,
                     self._tcp,
                     self._qActual))  # FIXME use named arrays of allow to configure what data to buffer

        with self._dataEvent:
            self._dataEvent.notifyAll()

    def start_buffering(self):
        """
        start buffering all data from controller
        """
        self._buffer = []
        self._buffering = True

    def stop_buffering(self):
        self._buffering = False

    def try_pop_buffer(self):
        """
        return oldest value in buffer
        """
        with self._buffer_lock:
            if len(self._buffer) > 0:
                return self._buffer.pop(0)
            else:
                return None

    def pop_buffer(self):
        """
        return oldest value in buffer
        """
        while True:
            with self._buffer_lock:
                if len(self._buffer) > 0:
                    return self._buffer.pop(0)
            time.sleep(0.001)

    def get_buffer(self):
        """
        return a copy of the entire buffer
        """
        with self._buffer_lock:
            return deepcopy(self._buffer)

    def get_all_data(self, wait=True):
        """
        return all data parsed from robot as a dict
        """
        if wait:
            self.wait()
        with self._dataAccess:
            return dict(
                timestamp=self._timestamp,
                ctrltimestamp=self._ctrlTimestamp,
                qActual=self._qActual,
                qTarget=self._qTarget,
                qdActual=self._qdActual,
                qdTarget=self._qdTarget,
                tcp=self._tcp,
                tcp_force=self._tcp_force,
                tcp_speed=self._tcp_speed,
                joint_temperature=self._joint_temperature,
                joint_voltage=self._joint_voltage,
                joint_current=self._joint_current,
                joint_modes=self._joint_modes,
                robot_mode=self._robot_mode,
                safety_mode=self._safety_mode,
                main_voltage=self._main_voltage,
                robot_voltage=self._robot_voltage,
                robot_current=self._robot_current,
                digital_outputs=self._digital_outputs,
                program_state=self._program_state,
                safety_status=self._safety_status)
    getALLData = get_all_data

    def stop(self):
        # print(self.__class__.__name__+': Stopping')
        self._stop_event = True

    def close(self):
        self.stop()
        self.join()

    #: Seconds of silence on the 30003 stream before the socket is treated as
    #: dead and reconnected. The controller streams at 125 Hz (8 ms/packet), so
    #: two seconds of nothing is three orders of magnitude past healthy.
    RT_RECV_TIMEOUT_S = 2.0

    def _open_rt_socket(self):
        """A fresh, connected 30003 socket with a recv timeout. A new socket
        each time because a dropped one cannot be reconnected, and because this
        controller hands a fresh connection a new burst of data where the old
        one has gone silent (see the reconnect loop in run)."""
        self._close_rt_socket()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.RT_RECV_TIMEOUT_S)
        sock.connect((self._urHost, 30003))
        self._rtSock = sock

    def _close_rt_socket(self):
        try:
            if self._rtSock is not None:
                self._rtSock.close()
        except Exception:
            pass

    def run(self):
        """Keep the 30003 realtime stream alive, reconnecting when it stalls.

        This controller's realtime interface intermittently goes quiet on a
        connection that stays open -- the socket does not error, it just stops
        delivering. Left alone, the monitor's socket dies while fresh ones
        still work, so every get_safety_mode (and thus every move) fails until
        the server is restarted. So a stall (recv timeout, set on the socket)
        or any socket/parse error is caught here: wake any waiter so it gets
        its timeout rather than blocking, drop the dead socket, reconnect, and
        carry on. A fresh connection gets a fresh stream.
        """
        self._stop_event = False
        backoff = 0.5
        while not self._stop_event:
            try:
                self._open_rt_socket()
                backoff = 0.5                       # reset once a connection is up
                while not self._stop_event:
                    self.__recv_rt_data()
            except Exception as e:
                self.logger.warning(
                    "Realtime monitor lost the 30003 stream (%s); reconnecting.", e)
                with self._dataEvent:               # waiters get a timeout, not silence
                    self._dataEvent.notifyAll()
                self._close_rt_socket()
                waited = 0.0
                while not self._stop_event and waited < backoff:
                    time.sleep(0.05)
                    waited += 0.05
                backoff = min(backoff * 2.0, 5.0)   # back off, capped
        self._close_rt_socket()
        with self._dataEvent:
            self._dataEvent.notifyAll()
