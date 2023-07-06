''' This program is to use the UR3 robot at 12IDB of APS'''
import sys
sys.path.append(r"..")
sys.path.append(r"UR_12ID")
sys.path.append(r"UR_12ID/ini")

QRPVavailable = False
try:
    import epics
    import multi_sample12IDB as bl
    QR_PV = epics.PV(bl.QR_PV)
    QRPVavailable = True
except:
    pass
from PyQt5.QtCore import (pyqtSignal, QObject)
from PyQt5.QtWidgets import QWidget

import time
import numpy
# conda install -c anaconda freetype
# conda install -c conda-forge ez_setup
#beamlinePV = '12idc:'
#beamlinePV2 = "12idb:"
#trans_motor_PV = 17
#vert_motor_PV = 18
#magazine_motor_PV = 20
#fingerPV = "9440:1:bo_0"

class RobotException(Exception):
    pass

# UR3
import os
import math3d as m3d
import numpy as np
import math
import time
import json
import robUR3
from urcamera import Detection as atDET
from urcamera import cal_AT2pose
import camera_tools as ctool

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

class UR3(robUR3.UR):
    # unit of position vector : meter.
    sigFinger = pyqtSignal(str)
    sigMoving = pyqtSignal(bool)
    sigFingerPosition = pyqtSignal(str)
    sigObject_onFinger = pyqtSignal(bool)
    sigRobotCommand = pyqtSignal(str)
    sigRobotPosition = pyqtSignal(numpy.ndarray)
    #Waypointmagup_q=[5.192646026611328, -0.6902674001506348, 0.557411019002096, -1.440483884220459, -1.5736210981952112, -1.0272372404681605]
    Waypointmagup_q = [3.915731906890869, -1.0986860555461426, 1.227795426045553, -1.702268739739889, -1.5722954908954065, 0.7979311943054199]
    Waypoint_QR_p = [ 2.06744440e-01,  2.35091449e-01,  1.53515533e-01,  2.88421769e+00, -1.24519515e+00, -3.20548384e-05]
    _TCP2CAMdistance = 0.12
#    tcp = [0.0,0.0,0.167,0.0,0.0,0.0]
    tcp = [0.0,0.0,0.15,0.0,0.0,0.0]
#    camtcp = [-0.001, 0.04, 0.015, -math.pi/180*30, 0, 0]
    camtcp = [0, 0.04, 0.015, -math.pi/180*30, 0, 0]
    april_tag_on_heater = {'size': 0.01}

    def __init__(self, name = 'UR3', fingertype=1, cameratype=2):
# definition of Cartesian Axis of UR3 at 12idb.
# X : positive - Out board
# X : negative - In board
# Y : positive - Along X-ray
# Y : negative - Again X-ray

        try:
            #with open('../RobotList/list_of_robots.json') as json_file:
            jsname = 'list_of_robots.json'
            if os.path.exists('RobotList'):
                fn = os.path.join('RobotList', jsname)
            if os.path.exists('../RobotList'):
                fn = os.path.join('../RobotList', jsname)
            with open(fn) as json_file:
                IPlist = json.load(json_file)
            IP = IPlist[name]
        except FileNotFoundError:
            print("Please provide the IP number of your robot control box.")
            return
        except KeyError:
            print(f"{name} does not exist in ../RobotList/list_of_robots.json")
            return

        super(UR3, self).__init__(IP, fingertype=fingertype, cameratype=cameratype)

        self.name = name
        self.ini_name = '%s.ini'%name
        self._path_ini_name = '%spath.ini'%name
        self.isSampleOnStage = False
        self.currentFrame = 0
        self.tweak_reference_axis_angle = None
        self.readini()
        if hasattr(self, 'camera'):
            self.camera.AT_physical_size = self.april_tag_on_heater['size']
        
    def init_waypoints(self):
        Waypointmagdn_p = self.pos_sample1
        Waypointmagup_p = [Waypointmagdn_p[0],Waypointmagdn_p[1],Waypointmagdn_p[2],
                Waypointmagdn_p[3],Waypointmagdn_p[4],Waypointmagdn_p[5]]
        Waypointmagup_p[2] = Waypointmagdn_p[2] + self.vert_magZ/1000
        Waypointdown_p = self.pos_samplestage
        Waypointup_p = [Waypointdown_p[0],Waypointdown_p[1],Waypointdown_p[2],
                Waypointdown_p[3],Waypointdown_p[4],Waypointdown_p[5]]
        Waypointup_p[2] = Waypointup_p[2] + self.vert_samZ/1000

        if hasattr(self, 'pos_samplestage2'):
            Waypointdown2_p = self.pos_samplestage2
            Waypointup2_p = [Waypointdown2_p[0],Waypointdown2_p[1],Waypointdown2_p[2],
                    Waypointdown2_p[3],Waypointdown2_p[4],Waypointdown2_p[5]]
            Waypointup2_p[2] = Waypointup2_p[2] + self.vert_samZ/1000
            self.samup2_p = m3d.Transform(Waypointup2_p)
            self.samdn2_p = m3d.Transform(Waypointdown2_p)

        Waypoint_middle_p = [Waypointup_p[0],Waypointup_p[1],Waypointup_p[2],
                Waypointup_p[3],Waypointup_p[4],Waypointup_p[5]]
        Waypoint_middle_p[0] = Waypoint_middle_p[0] + self.trans_X/1000
        self.Waypointmagdn_p = self.pos_sample1
        self.Waypointmagup_p = Waypointmagup_p
        self.middl_p = m3d.Transform(Waypoint_middle_p)
        self.samup_p = m3d.Transform(Waypointup_p)
        self.samdn_p = m3d.Transform(Waypointdown_p)
        self.QR_p = m3d.Transform(self.Waypoint_QR_p)
        #self.QR_p = self.middl_p # for now default QR reading position will be here.
        self.set_magazine(0, 0)
        #self.magdn_p = magdn_p
        #self.magup_p = magup_p

    def camera_face_down(self):
        pos = self.robot.get_pose()
        if self.whereisFinger() == "nowhere":
            self.camera2z()
        if self.whereisFinger() == 'samplestage':
            self.tilt_xm()
        if self.whereisFinger() == 'samplestage2':
            self.tilt_xm()
        if self.whereisFinger() == 'magazine':
            self.camera2z()
        self.set_tcp(self.camtcp)
        npos = self.robot.get_pose()
        pos.orient = npos.orient
        self.robot.set_pose(pos, vel=0.3,acc=0.2)
        self.set_tcp(self.tcp)

    def set_sampledown2_from_sampledown(self, pos, relative=True):
        if relative == True:
            tmp = self.samdn_p.get_pose_vector().tolist()
            for ind in range(len(tmp)):
                tmp[ind] = tmp[ind]+pos[ind]
            pos = tmp
        self.samdn2_p = m3d.Transform(pos)

    def set_current_as_sampledownXYonly(self):
        #self.samdn_p = self.robot.get_pose()
        newp = self.robot.getl()
        for i in range(len(newp)):
            if i==2:
                continue
            self.pos_samplestage[i] = newp[i]
#        self.pos_samplestage = self.samdn_p
        Waypointdown_p = self.pos_samplestage
        Waypointup_p = [Waypointdown_p[0],Waypointdown_p[1],Waypointdown_p[2],
                Waypointdown_p[3],Waypointdown_p[4],Waypointdown_p[5]]
#        Waypointup_p[2] = Waypointup_p[2] + self.vert_samZ/1000
        Waypointup_p[2] = self.samup_p.pos.list[2]
        self.samdn_p = m3d.Transform(Waypointdown_p)
        self.samup_p = m3d.Transform(Waypointup_p)

    def set_current_as_sampledown(self):
        self.samdn_p = self.robot.get_pose()
        self.pos_samplestage = self.robot.getl()
#        self.pos_samplestage = self.samdn_p
        Waypointdown_p = self.pos_samplestage
        Waypointup_p = [Waypointdown_p[0],Waypointdown_p[1],Waypointdown_p[2],
                Waypointdown_p[3],Waypointdown_p[4],Waypointdown_p[5]]
#        Waypointup_p[2] = Waypointup_p[2] + self.vert_samZ/1000
        Waypointup_p[2] = self.samup_p.pos.list[2]
        self.samup_p = m3d.Transform(Waypointup_p)

    def set_current_as_magazinedown(self):
        self.magdn_p = self.robot.get_pose()
        self.pos_sample1 = self.robot.getl()
        print(f"Position of the first sample on Magazine is {self.pos_sample1}.")
        Waypointmagdn_p = self.pos_sample1
        Waypointmagup_p = [Waypointmagdn_p[0],Waypointmagdn_p[1],Waypointmagdn_p[2],
                Waypointmagdn_p[3],Waypointmagdn_p[4],Waypointmagdn_p[5]]
        Waypointmagup_p[2] = Waypointmagdn_p[2] + self.vert_magZ/1000
        #self.logger.debug("Position: %d", self.magup_p)
        self.magup_p = m3d.Transform(Waypointmagup_p)
        #self.logger.debug("Is changed to %d", self.magup_p)

    def writeini(self):
        w = []
        #print(self.ini_name, " will be overritten.")
        with open(self.ini_name) as f:
            for line in f:
                (desc, val) = line.split(',')
                (key, val) = val.split(':')
                myval = getattr(self, key.strip())
                ismultiple = False
                if isinstance(myval, list):
                    if len(myval) > 1:
                        ismultiple = True
                if ismultiple == False:
                    if isinstance(myval, str):
                        mstr = "%s, %s : %s\n" % (desc.strip(), key.strip(), myval)
                    else:
                        mstr = "%s, %s : %0.4f\n" % (desc.strip(), key.strip(), myval)
                else:
                    s = ""
                    for ind in myval:
                        s = s + "%0.4f "%ind
                    mstr = "%s, %s : %s\n" % (desc.strip(), key.strip(), s)
                #print(mstr)
                w.append(mstr)
        f = open(self.ini_name, "w")
        f.writelines(w)
        f.close()

    def readini(self):
        d = {}
        with open(self.ini_name) as f:
            for line in f:
                if len(line) < 1:
                    break
                (desc, val) = line.split(',')
                (key, val) = val.split(':')
                vallist = val.split()
                if len(vallist) == 1:
                    val = vallist[0].strip()
                    try:
                        val = float(val)
                    except:
                        pass
                else:
                    val = []
                    for i in vallist:
                        val.append(float(i))
                d[key.strip()] = val
        for key in d:
            setattr(self, key, d[key])
        self.init_waypoints()
        self.getCurrentFrameNumber()
        self.read_pathini()
        return d

    def read_pathini(self):
        d = {}
        with open(self._path_ini_name) as f:
            for line in f:
#                print(line)
                if len(line) < 1:
                    break
 #               print(line.split(','))
                (desc, val) = line.split(',')
 #               print(desc)
 #               print(val)
                (key, val) = val.split(':')
                vallist = val.split()
                if len(vallist) == 1:
                    val = float(val)
                else:
                    val = []
                    for i in vallist:
                        val.append(float(i))
                d[key.strip()] = val
        for key in d:
            setattr(self, key, d[key])
        return d

    def set_magazine(self, xN, yN):
        d = self.Waypointmagdn_p
        #u = self.Waypointmagdn_p
        newdn = [d[0],d[1],d[2],d[3],d[4],d[5]]
        newup = [d[0],d[1],d[2]+self.vert_magZ/1000, d[3],d[4],d[5]]
        newdn[0] = d[0] + xN*self.magXgap/1000
        newdn[1] = d[1] + yN*self.magYgap/1000
        newup[0] = d[0] + xN*self.magXgap/1000
        newup[1] = d[1] + yN*self.magYgap/1000
#        print(newdn)
#        print(xN, yN)
        self.magdn_p = m3d.Transform(newdn)
        self.magup_p = m3d.Transform(newup)

    def get_magazine(self):
        newdn = self.magdn_p.get_pose_vector()
        xd = (newdn[0]-self.Waypointmagdn_p[0])
        yd = (newdn[1]-self.Waypointmagdn_p[1])
        if self.magXgap != 0:
            i = round(xd/(self.magXgap/1000), 0)
        else:
            i = 0
        if self.magYgap != 0:
            j = round(yd/(self.magYgap/1000), 0)
        else:
            j = 0
        return (i, j)

    def transport_from_samplestage_up_to_magazine_up(self):
        #try:
        self.robot.movels([self.path3, self.path2, self.path1],radius=0.01, acc=0.5, vel=0.5)
        self.robot.movej(self.middl_q, acc=0.5, vel=1)
        # inverse kinematics toward the magazine works....
        self.robot.set_pose(self.magup_p, acc=0.5, vel=1, command='movej')
        #except ParsingException as ex::

    def transport_from_magazine_up_to_QR(self):
        self.robot.movel(self.QR_p, acc=0.5, vel=1)

    def transport_from_sample_up_to_sample2_up(self):
        tmp = self.samup_p.get_pose_vector().tolist()
        tmp2 = self.samdn2_p.get_pose_vector().tolist()
        tmp2[2] = tmp[2]
        self.samup2_p = m3d.Transform(tmp2)
        self.robot.movel(self.samup2_p, acc=0.5, vel=1)

    def transport_from_sample2_up_to_sample_up(self):
        self.robot.movel(self.samup_p, acc=0.5, vel=1)

    def transport_from_samplestage_up_to_default(self):
        self.robot.movels([self.samup_p, self.path3, self.path2, self.path1],radius=0.01, acc=0.5, vel=0.5)
        self.robot.movej(self.middl_q, acc=0.5, vel=1)

    def transport_from_magazine_up_to_samplestage_up(self):
        self.robot.movej(self.middl_q, acc=0.5, vel=1)
        self.robot.movels([self.path1, self.path2, self.path3, self.samup_p],radius=0.01, acc=0.5, vel=0.5)

    def transport_from_default_to_samplestage_up(self):
#        self.robot.movej(self.middl_q, acc=0.5, vel=0.5)
        self.robot.movels([self.path1, self.path2, self.path3, self.samup_p],radius=0.01, acc=0.5, vel=0.5)

    def picksample(self):
        if self.currentFrame < 0.0:
            return -1
        try:
            self.sigMoving.emit(True)
        except:
            pass
        self.release()
        self.transfingertomagazine()
        # # check where the finger is now..
        # wh = self.whereisFinger()
        # if wh == 'samplestage':
        #     self.transport_from_samplestage_up_to_magazine_up()
        # if wh == ('magazine', 'middle'):
        #     self.transport_from_default_to_magazine_up()
        # pos = self.robot.getl()
        # if (pos[1]>-0.03) and (pos[0]>0.0): # finger is at a quadrant where magazine is.
        #     if pos[2] < (self.magup_p.pos[2]-0.01):
        #         self.robot.up(self.magup_p.pos[2]-pos[2], acc=1.4, vel=1.0)
        # else:
        #     self.robot.movej(self.middl_q, acc=0.5, vel=0.5)

        # self.robot.set_pose(self.magup_p, acc=0.5, vel=0.5, command='movej')

        # going down to pick up sample
        self.sigFinger.emit("Frame %i is picked up.."%self.currentFrame)
        self.pickup()
        # self.robot.movel(self.magdn_p, acc=0.5, vel=0.5)
        # self.grab()
        # self.robot.movel(self.magup_p, acc=0.5, vel=0.5)
        # transport to the sample stage via a middle point
        self.sigFinger.emit("Being transported..")
#        self.robot.movej(self.middl_q, acc=0.5, vel=1.0) # inverse_kinematics calculation failed.
        #so I have to find out the joint position of middl.
        self.transport_from_magazine_up_to_samplestage_up()
#        self.robot.movels([self.middl_p, self.samup_p], radius=0.01, acc=0.5, vel=0.5) 
        return 0

    def scan_QR_all(self, mysample=[]):
        if not QRPVavailable:
            return mysample
        sam = []
        if len(mysample) == 0:
            mysample = range(0,self.mag_index)
        for ind in mysample:
            print(ind, " This is the index")
            QR_PV.put("")
            val = self.picksample_forQR(ind)
            if val == 0:
                time.sleep(2)
                qr = QR_PV.get()
#                print(ind, qr)
                sam.append([ind, qr])
                self.returnsample_fromQR()
            else:
#                print(ind)
                sam.append([ind, None])
        self.release()
#        print(sam)
        return sam

    def picksample_forQR(self, number):
        if self.currentFrame < 0.0:
            return -1
        try:
            self.sigMoving.emit(True)
        except:
            pass
        self.release()
        self.moveMagazine2FrameN(number)
        self.transfingertomagazine()

        # going down to pick up sample
        self.sigFinger.emit("Frame %i is picked up.."%self.currentFrame)
        self.pickup()
        if self.sample_onFinger:
            # transport to the sample stage via a middle point
            self.sigFinger.emit("Being transported..")
            self.transport_from_magazine_up_to_QR()
    #        self.robot.movels([self.middl_p, self.samup_p], radius=0.01, acc=0.5, vel=0.5) 
            return 0
        else:
            return -1

    def returnsample_fromQR(self):
        self.transfingertomagazine()
        self.putsampledown()

    # @property
    # def whereisFinger(self): 
    #     return self._fingerpos 
    # @whereisFinger.setter 
    # def whereisFinger(self, pos): 
    #     self._fingerpos = pos 

    def relocate_sample_to_sample2(self):
        if not (self.whereisFinger() == 'samplestage'):
            self.transfingertosamplestage()
        self.pickup()
        self.transport_from_sample_up_to_sample2_up()
        self.putsampledown()

    def relocate_sample2_to_sample(self):
        if not (self.whereisFinger() == 'samplestage2'):
            self.transfingertosamplestage2()        
        self.pickup()
        self.transport_from_sample2_up_to_sample_up()
        self.putsampledown()

    def getsample(self):
        r = self.picksample()
        if r<0:
            return r
        self.putsampledown()
        self.transport_from_samplestage_up_to_default()
        # wait at the middle position for data acquisition
        self.sigFinger.emit("Frame %i is loaded. Waiting for data acquisition.."%self.currentFrame)
# #        self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
#         self.robot.movel(self.middl_p, acc=0.5, vel=0.5)
#         self.robot.movej(self.middl_q, acc=0.5, vel=0.5)
        try:
            self.sigMoving.emit(False)
        except:
            pass
        self.isSampleOnStage = True
        #self.whereisFinger() = 'samplestage'
        return 0

    def returnsample(self, z=0.15, in_offset = None):
        try:
            self.sigMoving.emit(True)
        except:
            pass
        self.release()
        self.transfingertosamplestage()
        # if self.whereisFinger() == 'middle':
        #     self.transport_from_default_to_samplestage_up()
        # if self.whereisFinger() == 'magazine':
        #     self.transport_from_magazine_up_to_samplestage_up()
#        if not self.whereisFinger() in ('samplestage', 'middle'):
#            self.robot.movej(self.middl_q, acc=0.5, vel=0.5)
#            self.robot.movel(self.middl_p, acc=1.4, vel=1.0)
        # pick up sample from the sample stage
        self.sigFinger.emit("Returning the sample first..")
#        self.robot.movels([self.samup_p, self.samdn_p], radius = 0.01, acc=0.5, vel=0.5)
#        #self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
#        self.grab()
#        self.robot.movel(self.sampup_p, acc=0.5, vel=0.5)
        self.pickup()

        # transport to the magazine via the middle point
        self.sigFinger.emit("Being transported..")
        self.transport_from_samplestage_up_to_magazine_up()
        # self.robot.movels([self.samup_p, self.middl_p], radius = 0.01, acc=0.5, vel=0.5)
        # #self.robot.movel(self.middl_p, acc=0.5, vel=0.5)
        # self.robot.movej(self.middl_q, acc=0.5, vel=0.5)

        # # inverse kinematics toward the magazine works....
        # self.robot.set_pose(self.magup_p, acc=0.5, vel=0.5, command='movej')
        
        #rob.movep(magup_p, acc=0.5, vel=0.5, radius=0.05)

#        self.sigFinger.emit("Returned..")
        val = self.putsampledown(z=0.150, offset=in_offset)
        self.sigFinger.emit("Successfully returned..")
        # self.robot.down((self.vert_magZ-1.5)/1000, acc=1.4, vel=1.0)
        # #self.robot.movel(self.magdn_p, acc=1.4, vel=1.0)
        # self.release()

        # # move up and stop.
        # self.robot.movel(self.magup_p, acc=1.4, vel=1.0)

        self.isSampleOnStage = False
        try:
            self.sigMoving.emit(False)
        except:
            pass
        return val

    def moveMagazine2NextFrame(self):
        self.currentFrame = self.currentFrame + 1
        self.moveMagazine2FrameN(self.currentFrame)

    def moveMagazine2FrameN(self, number):
        self.currentFrame = number
        newN = self.mag_index[number]
        xN, yN = ind2sub(int(newN), (int(self.numXFrame), int(self.numYFrame)))
        self.set_magazine(xN, yN)
    
    def mountNextFrame(self):
        self.currentFrame = self.currentFrame + 1
        self.mountFrameN(self.currentFrame)

    def mountFrameN(self, number):
        if self.isSampleOnStage is True:
            #self.sigFinger.emit("Sample is being returned.")
            self.returnsample()
        if number > self.numFrame:
            print("%i is larger than total number of frames"%number)
            return -2
        self.sigFinger.emit("Moving the finger over to next frame.")
        self.moveMagazine2FrameN(number)
        #self.sigFinger.emit("Frame %i is being picked up."%self.currentFrame)
        rtn = self.getsample()
#        if rtn == 0:
#            self.sigFinger.emit("Frame %i is loaded successfully."%self.currentFrame)
        return rtn

    def setCurrentasFirstFrame(self):
        self.currentFrame = 1

    def getCurrentFrameNumber(self):
        i, j = self.get_magazine()
        newN = sub2ind(i, j, (int(self.numXFrame), int(self.numYFrame)))
        self.currentFrame = self.mag_index.index(newN)

    def pickuptest(self, gap=0):

        self.release()
        self.movefingerup_totransport()
        #self.transportfinger_to_magazine()
        self.transfingertomagazine()
        # if not self.whereisFinger() == 'magazine':
        #     # go to pick position
        #     self.robot.movej(self.middl_q, acc=0.5, vel=1.0)
        #     #self.robot.movels([self.middl_p, self.magup_p], acc=0.5, vel=0.5, radius=0.05)

        # self.robot.set_pose(self.magup_p, acc=0.5, vel=0.5, command='movej')

        # going down to pick up sample
        self.pickup()
        # self.sigFinger.emit("Moving the finger down to pick up.")
        # self.robot.movel(self.magdn_p, acc=1.4, vel=1.0)
        # self.sigFinger.emit("Grabing.")
        # self.grab()
        # self.sigFinger.emit("Moving up to transport.")
        # self.robot.movel(self.magup_p, acc=1.4, vel=1.0)

        time.sleep(5)
        self.sigFinger.emit("Moving down to return..")
        self.putsampledown()
        # self.robot.movel(self.magdn_p, acc=1.4, vel=1.0)
        # self.sigFinger.emit("Release fingers.")
        # self.release()

        # # move up and stop.
        # self.sigFinger.emit("Moving fingers up.")
        # self.robot.movel(self.magup_p, acc=1.4, vel=1.0)

    def goto_default(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'goto_default'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        pos = self.robot.getl()
        pm = self.magup_p.get_pos()
        ps = self.samup_p.get_pos()
        minzval = min(pm[2], ps[2])
        if pos[2] < minzval:
            pos[2] = minzval
            self.robot.movel(pos, acc=0.5, vel=0.5)
        self.robot.movej(self.middl_q, acc=0.5, vel=0.75)
#        self.whereisFinger()

#    def goto_magazine(self):
#        pos = self.robot.getl()
#        if (pos[1] > -0.2) and (pos[0]>0.35) and (pos[2]>0.15):
#            pass
#        else:
#            self.goto_default()
#        self.robot.set_pose(self.magup_p, acc=0.5, vel=0.75, command="movej")

    def pickup(self):
        run = 0
        if self.whereisFinger() == "nowhere":
            print("current figner position is at nowhere.")
        if self.whereisFinger() == 'samplestage':
            self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
            run = 1
        if self.whereisFinger() == 'samplestage2':
            self.robot.movel(self.samdn2_p, acc=0.5, vel=0.5)
            run = 3
        if self.whereisFinger() == 'magazine':
            self.robot.movel(self.magdn_p, acc=0.5, vel=0.5)
            run = 2
        self.grab()
        if run == 1:
            self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        if run == 2:
            self.robot.movel(self.magup_p, acc=0.5, vel=0.5)
        if run == 3:
            self.robot.movel(self.samup2_p, acc=0.5, vel=0.5)
        
        val = self.finger.get_position()
        
        if val < 0.5:
            self.sample_onFinger = False
            self.sigObject_onFinger.emit(False)
        else:
            self.sample_onFinger = True
            self.sigObject_onFinger.emit(True)
#        self.whereisFinger()

    def dropofftest(self):
        #print(self.whereisFinger(), "This is a finger position")
        #if not self.whereisFinger() == 'middle':
        #    self.robot.movel(self.middl_p, acc=0.5, vel=0.5)
        #print(self.whereisFinger(), "This is a finger position")
        if self.whereisFinger() == 'samplestage':
            self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
            self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
            self.grab()

            # transport to the magazine via the middle point
            self.sigFinger.emit("Being transported..")
            self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
            time.sleep(5)
            self.putsampledown()
        elif self.whereisFinger() == 'samplestage2':
            self.robot.movel(self.samup2_p, acc=0.5, vel=0.5)
            self.robot.movel(self.samdn2_p, acc=0.5, vel=0.5)
            self.grab()

            # transport to the magazine via the middle point
            self.sigFinger.emit("Being transported..")
            self.robot.movel(self.samup2_p, acc=0.5, vel=0.5)
            time.sleep(5)
            self.putsampledown()
        else:
            return


    def movefingerup_totransport(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'movefingerup_totransport'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        if self.whereisFinger() == 'samplestage':
            self.robot.movel(self.samup_p, acc=0.5, vel=0.5)
        if self.whereisFinger() == 'samplestage2':
            self.robot.movel(self.samup2_p, acc=0.5, vel=0.5)
        if self.whereisFinger() == 'magazine':
            self.robot.movel(self.magup_p, acc=0.5, vel=0.5)
#        self.whereisFinger()

    def movefingerdown_tosamplestage(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'movefingerdown_tosamplestage'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        self.robot.movel(self.samdn_p, acc=0.5, vel=0.5)
#        self.whereisFinger()

    def movefingerdown_tomagazine(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'movefingerdown_tomagazine'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        self.robot.movel(self.magdn_p, acc=0.5, vel=0.5)
#        self.whereisFinger()
    
    def transfingertosamplestage(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transfingertosamplestage'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        #        if self.whereisFinger() == 'samplestage':
#            self.transport_from_samplestage_up_to_samplestage_up()
        if self.whereisFinger() == 'middle':
            self.transport_from_default_to_samplestage_up()
        if self.whereisFinger() == 'samplestage2':
            self.transport_from_sample2_up_to_sample_up()
        if self.whereisFinger() == 'nowhere':
            self.goto_default()
            self.transport_from_default_to_samplestage_up()
        if self.whereisFinger() == 'magazine':
            self.transport_from_magazine_up_to_samplestage_up()
#        self.whereisFinger()
    def transfingertosamplestage2(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transfingertosamplestage'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        #        if self.whereisFinger() == 'samplestage':
#            self.transport_from_samplestage_up_to_samplestage_up()
        if self.whereisFinger() == 'middle':
            self.transport_from_default_to_samplestage_up()
        if self.whereisFinger() == 'samplestage':
            self.transport_from_sample_up_to_sample2_up()
        if self.whereisFinger() == 'nowhere':
            self.goto_default()
            self.transport_from_default_to_samplestage_up()
        if self.whereisFinger() == 'magazine':
            self.transport_from_magazine_up_to_samplestage_up()
#        self.whereisFinger()

    def transfingertomagazine(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transfingertomagazine'
        self.sigRobotCommand.emit(self.pastcommand)
        self.sigRobotPosition.emit(self.pastpos)
        if self.whereisFinger() == 'samplestage':
            self.transport_from_samplestage_up_to_magazine_up()
        if self.whereisFinger() == 'middle':
            self.transport_from_default_to_magazine_up()
        if self.whereisFinger() == 'nowhere':
            self.goto_default()
            self.transport_from_default_to_magazine_up()
        if self.whereisFinger() == 'magazine':
            self.robot.set_pose(self.magup_p, acc=0.5, vel=1, command='movej')
        #print(self.whereisFinger(), "This is in transfingertomagazine")

    def transport_from_default_to_magazine_up(self):
        self.pastpos = self.get_xyz()
        self.pastcommand = 'transport_from_default_to_magazine_up'
        self.robot.set_pose(self.magup_p, acc=0.5, vel=1, command='movej')

    def putsampledown(self, z=0.150, offset=None):
        # robot arm will go down by abs(z)-offset
        #
        if offset == None:
            if hasattr(self, 'vert_offset'):
                offset = self.vert_offset*0.001
            else:
                offset = 0.001
        #print(self.whereisFinger())
        #print(self.samdn2_p.get_pose_vector().tolist(), "sampdn2")
        #print(self.samdn_p.get_pose_vector().tolist(), "sampdn")
        if self.whereisFinger() == "nowhere":
            print("current figner position is at nowhere.")
        if self.whereisFinger() == 'samplestage':
            run = 1
            cposz = self.robot.get_pos()
            z = abs(cposz[2] - self.samdn_p.pos[2])-offset
        if self.whereisFinger() == 'samplestage2':
            run = 3
            cposz = self.robot.get_pos()
            z = abs(cposz[2] - self.samdn2_p.pos[2])-offset
            print(f"z to go is {z}")
        if self.whereisFinger() == 'magazine':
            run = 2
            cposz = self.robot.get_pos()
            z = abs(cposz[2] - self.magdn_p.pos[2])-offset

        self.move2z(-z, acc=0.05, vel=0.05, wait=True) # No force measurement, just drop.
        self.loosen() # drop sample
        time.sleep(0.1)
        #try:
        self.robot.down(0.002) # slide down further to make sure that the rubber on the gripper does not stick to the sample plate.
        #except:
        #    return -1
        self.release()
        time.sleep(0.1)
        if run == 1:
            self.robot.movel(self.samup_p, acc=0.5, vel=1)
        if run == 2:
            self.robot.movel(self.magup_p, acc=0.5, vel=1)
        if run == 3:
            self.robot.movel(self.samup2_p, acc=0.5, vel=1)
#        self.whereisFinger()
        return 0
#        self.robot.movel(pos_org, acc=0.4, vel=0.2, wait=True)

    def dropsample(self, z=-1):
        self.robot.bump(z=z, backoff=0.005) # No force measurement, just drop.
#        status = False
#        while (not status):
#            status = self.robot.is_program_running()
#            time.sleep(0.01)
        self.release()

    def whereisFinger(self):
        pos = self.robot.get_pose()

        t = pos.pos - self.magup_p.pos
        t[2] = 0 # make Z 0. only compare (x, y)
        if (pos.pos[0] > 0) and (pos.pos[1]>-0.2): #somewhere sample area
            self.sigFingerPosition.emit('magazine')
            return 'magazine'

        t = pos.pos - self.samup_p.pos
        t[2] = 0 # make Z 0. only compare (x, y)
        if t.length < 0.06:
            self.sigFingerPosition.emit('samplestage')
            return 'samplestage'
        
        if hasattr(self, 'samdn2_p'):
            t = pos.pos - self.samdn2_p.pos
            t[2] = 0 # make Z 0. only compare (x, y)
            if t.length < 0.06:
                self.sigFingerPosition.emit('samplestage2')
                return 'samplestage2'

        ang = self.robot.getj()
        v = numpy.asarray(self.middl_q)
        if sum((v-ang)*(v-ang)) < 0.01:
            return 'middle'

        t = pos.pos - self.middl_p.pos
        t[2] = 0 # make Z 0. only compare (x, y)
        if t.length < 0.06:
            self.sigFingerPosition.emit('middle')
            return 'middle'
        else:
            self.sigFingerPosition.emit('nowhere')
            return "nowhere"


    def update_camera_info(self):
        # qrcode sav 
        # size; 22.5mm
        # at 250mm away, 120 pixels.
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
        elif referenceName == "AT":
            try:
                r = self.camera.decodeAT()
            except:
                r = self.camera.decoded
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
        elif referenceName == "AT":
            try:
                r = self.camera.decodeAT()
            except:
                r = self.camera.decoded
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

    def relocate_camera(self, distance2go = 0.2):
        # Locate camera at the shortest distance between the objec and base.
        # align Z
        # set camera position 0.2m away from the obj
        obj_pos, campos = self.get_obj_position_from_camera_center(distance2go)
        #if not isinstance(orient, m3d.Orientation):
        orient = m3d.Orientation([0, -math.pi, 0]) # make camera point +y axis.
        #orient.rotate_zb(math.pi/2) # make camera point +x
        trans = self.robot.get_pose()
        trans.orient = orient
        trans.orient.rotate_zb(math.atan2(obj_pos[1], obj_pos[0])-math.pi/2)
        newpos = (obj_pos.length-0.2)/obj_pos.length*obj_pos
        trans.set_pos(newpos)
        self.robot.set_pose(trans, acc=0.1, vel=0.1)

    def get_obj_position_from_camera_center(self, distance2go):
        # calculate the object position that is at the center of camera image and QRrefdistance away from the camera surface.
        cameravector, v1, v2 = self.get_camera_vector()
        campos = self.get_camera_position()
        pos = campos.pos + distance2go*cameravector/cameravector.length
        return pos, campos

## April tag functions
    def orient2aprilTag(self):
        if not hasattr(self.camera, 'decoded'):
            return False
        r = self.camera.decoded
        if not isinstance(r, atDET):
            print("No aprilTag in the camera. Capture it and try again.")
            return
        euler, t, pos = cal_AT2pose(r)

        self.move_toward_camera(distance=0, north=-t[1][0], east=t[0][0], acc=0.1, vel=0.2)

        p = self.robot.get_pose()
        self.prev_pose = p.copy()
        self.prev_tcp = self.robot.get_tcp()

        p.orient.rotate_zt(euler[2]/180*math.pi)
        p.orient.rotate_yt(-euler[1]/180*math.pi)
        p.orient.rotate_xt(-euler[0]/180*math.pi)
        #print(p)
        self.robot.set_pose(p, wait=True, acc=0.1, vel=0.2, command="movej")


# #        self.move2xTCP(-t[0])
# #        self.move2yTCP(-t[1])
#         self.set_tcp(self.tcp)
        return euler, t, p

    def center_aprilTag(self):
        if not hasattr(self.camera, 'decoded'):
            return False
        r = self.camera.decoded
        if not isinstance(r, atDET):
            print("No aprilTag in the camera. Capture it and try again.")
            return
        euler, t, pos = cal_AT2pose(r)
        dx = self.camera.imgH/2-r.center[0]
        dy = self.camera.imgV/2-r.center[1]
        dX = -dx/self.camera.camera_f*t[2].tolist()[0]
        dY = dy/self.camera.camera_f*t[2].tolist()[0]
        self.move_toward_camera(distance=0, north=dY, east=dX, acc=0.5, vel=0.5)    