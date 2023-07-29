#from msilib.schema import RadioButton
#from types import NoneType
from PyQt5 import QtCore, QtGui
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QObject, QThreadPool, QRunnable
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox 
from PyQt5 import uic
#from roboUR3 import roboUR3
#from common.robUR import UR
#import argparse
import cv2
import math
import traceback, sys
import time
import numpy as np
import os
text_file_path = os.path.dirname(os.path.abspath(__file__))

class workerCV(QtCore.QThread):
    ImageUpdate = pyqtSignal(QtGui.QImage)
    def __init__(self, rob):
        super(workerCV, self).__init__()
        self.rob = rob
    def run(self):
        self.ThreadActive = True
        ret, frame = self.rob.camera.capture()
        while self.ThreadActive:
            ret, frame = self.rob.camera.capture()
            if ret:
                img = frame.copy()
                self.drawlines(img)
                Image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                #FlippedImage = cv2.flip(Image, 1)
                ConvertToQtFormat = QImage(Image, Image.shape[1],
                    Image.shape[0], QImage.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(640, 480, QtCore.Qt.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)
                time.sleep(0.02)
    def stop(self):
        self.ThreadActive = False
        self.quit()

    def drawlines(self, data):
        try:
            r = self.rob.camera.decodeAT()
        except OSError:
            return
#        self.rob.camera.decoded = r
        if type(r)==type(None):
            return
        if not hasattr(r, 'corners'):
            return
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(data, ptA, ptB, (0, 255, 0), 2)
        cv2.line(data, ptB, ptC, (0, 255, 0), 2)
        cv2.line(data, ptC, ptD, (0, 255, 0), 2)
        cv2.line(data, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(data, (cX, cY), 5, (0, 0, 255), -1)
#https://gist.github.com/ksvbka/1f26ada0c6201c2cf19f59b100d224a9
class WorkerSignals(QObject):
    '''
    Defines the signals available from a running worker thread.
    Supported signals are:
    - finished: No data
    - error:`tuple` (exctype, value, traceback.format_exc() )
    - result: `object` data returned from processing, anything
    - progress: `tuple` indicating progress metadata
    '''
    finished = pyqtSignal()
    error = pyqtSignal(tuple)
    result = pyqtSignal(object)
    progress = pyqtSignal(tuple)

class workerRobot(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super(workerRobot, self).__init__()
        self.rob = rob
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self):
        try:
            result = self.fn(*self.args, **self.kwargs)
        except:
            traceback.print_exc()
            exctype, value = sys.exc_info()[:2]
            self.signals.error.emit((exctype, value, traceback.format_exc()))
        else:
            self.signals.result.emit(result)  # Return the result of the processing
        finally:
            self.signals.finished.emit()  # Done

class tweakRobot(QMainWindow):
    def __init__(self, rob):
        super(tweakRobot, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        guiName = os.path.join(text_file_path, 'ui', "TweakRobot.ui")
        self.rob = rob
        self.ui = uic.loadUi(guiName)
        self.ui.Xm.clicked.connect(self.goxm)
        self.ui.Xp.clicked.connect(self.goxp)
        self.ui.Ym.clicked.connect(self.goym)
        self.ui.Yp.clicked.connect(self.goyp)
        self.ui.Zm.clicked.connect(self.gozm)
        self.ui.Zp.clicked.connect(self.gozp)
        self.ui.Rxm.clicked.connect(self.goRxm)
        self.ui.Rxp.clicked.connect(self.goRxp)
        self.ui.Rym.clicked.connect(self.goRym)
        self.ui.Ryp.clicked.connect(self.goRyp)
        self.ui.Rzm.clicked.connect(self.goRzm)
        self.ui.Rzp.clicked.connect(self.goRzp)
        self.ui.Send.clicked.connect(self.send_coords)
        self.ui.Camera.toggled.connect(lambda:self.btnstate(self.ui.Camera))
        self.ui.TCP.toggled.connect(lambda:self.btnstate(self.ui.TCP))
        self.ui.actionUnlock.triggered.connect(self.unlock)
        self.ui.actionEstimate_Dtag.triggered.connect(self.estimateDtag)
        self.ui.actionSet_Dtag_as_Camera_TCP.triggered.connect(self.setDtagAsTCP)
        self.ui.actionCenter_AprilTag.triggered.connect(self.centerAT)
        self.ui.actionShow.triggered.connect(self.showfeed)
        self.ui.actionStop_Feed.triggered.connect(self.CancelFeed)
        self.ui.actionCamera_Face_Down.triggered.connect(self.camera_downface)
        self.ui.actionClear_Feed.triggered.connect(self.clearImage)
        self.ui.actionCamera_to_face_at_AprilTag.triggered.connect(self.faceAtag)
        self.ui.actionMove_the_gripper_tip_to_the_camera_center.triggered.connect(self.putTCP2CM)
        self.ui.actionMove_the_camera_center_to_the_gripper_tip.triggered.connect(self.putCM2TCP)
        self.ui.actionMeasure_height.triggered.connect(self.measureheight)
        self.ui.actionPrint_Cartesian_Coordinates.triggered.connect(self.printXYZ)
        self.ui.actionPrint_Joints_Coordinates.triggered.connect(self.printJoints)
        self.ui.actionZ_align.triggered.connect(self.zalign)
        self.ui.actionReturn_back_to_previous_pose.triggered.connect(self.goback)
        self.updatepos()
        self.ui.show()
        self.threadpool = QThreadPool()

    def unlock(self):
        self.rob.unlock_stop()

    def goback(self):
        if hasattr(self.rob, 'prev_pose'):
            t = self.rob.get_tcp()
            self.rob.set_tcp(self.rob.prev_tcp)
            self.rob.set_pose(self.rob.prev_pose, vel=0.2, acc=0.1)
            self.rob.set_tcp(t)
        else:
            print("There was no prev_pose, yet. This only works with AprilTag.")
    
    def putCM2TCP(self):
        self.threadrun(self.rob.camera2grippertip)
    
    def putTCP2CM(self):
        self.threadrun(self.rob.grippertip2camera)
    
    def measureheight(self):
        # Pass the function to execute
        self.threadrun(self.rob.measureheight)

        #self.rob.measureheight()
    def threadrun(self, fn, *args, **kwargs):
        worker = workerRobot(fn) # Any other args, kwargs are passed to the run function
        worker.signals.result.connect(self.print_output)
        worker.signals.finished.connect(self.thread_complete)
        worker.signals.progress.connect(self.progress_fn)
        # Execute
        self.threadpool.start(worker)

    def progress_fn(self, progress):
        p, m = (progress)
        print("%d%% done %s" % (p, m))

    def execute_this_fn(self, progress_callback):
        for n in range(0, 5):
            time.sleep(1)
            progress_callback.emit((n*100/4, 'blabla'))

        return "Done."

    def print_output(self, s):
        try:
            s = str(s)
            self.ui.label_output.setText(s)
        except:
            print(s)

    def thread_complete(self):
        print("THREAD COMPLETE!")

    def camera_downface(self):
        self.threadrun(self.rob.tilt_camera_down)

    def ImageUpdateSlot(self, Image):
        self.ui.labelFeed.setPixmap(QtGui.QPixmap.fromImage(Image))
        
    def CancelFeed(self):
        try:
            self.worker.stop()
        except:
            return False
        while(1):
            self.ui.labelFeed.clear()
            if self.ui.labelFeed.pixmap() is None:
                break
            time.sleep(0.01)

    def clearImage(self):
        self.ui.labelFeed.setAutoFillBackground(True)
        self.ui.labelFeed.clear()
        #print(self._size)
        #self.resize(self._size)

    def estimateDtag(self):
        #self.rob.camera.capture()
        try:
            r = self.rob.camera.decoded
            print(r)
        except TypeError:
            print("No AprilTag is found.")
            return
        D = self.rob.camera.getATdistance(r)
        print(f"distance of the tag is {D}.")
        self.D = D

    def setDtagAsTCP(self):
        if hasattr(self, 'D'):
            self.ui.Camera.setChecked(True)
            cam_tcp = self.rob.camtcp.copy()
            dy = -math.sin(cam_tcp[3])*self.D
            dz = math.cos(cam_tcp[3])*self.D
            cam_tcp[1] += dy
            cam_tcp[2] += dz
            self.rob.set_tcp(cam_tcp)
            print(f"New TCP is : {cam_tcp}.")

    def zalign(self):
        self.ui.TCP.setChecked(True)
        self.rob.set_tcp(self.rob.tcp)
        self.rob.set_orientation()
        
    def faceAtag(self):
        #self.rob.camera.capture()
        self.estimateDtag()
        self.setDtagAsTCP()
#        self.CancelFeed()
        self.rob.orient2aprilTag()
#        self.showfeed()
#        self.rob.camera.decodeAT()
#        self.CancelFeed()
#        self.rob.center_aprilTag()
#        self.showfeed()

    def centerAT(self):
        #self.rob.camera.capture()
        try:
            r = self.rob.camera.decoded
        except TypeError:
            print("No AprilTag is found.")
            return
        self.threadrun(self.rob.center_aprilTag)

    def showfeed(self):
        #self._size = self.size()
        self.worker = workerCV(self.rob)
        self.worker.ImageUpdate.connect(self.ImageUpdateSlot)
        self.worker.start()
        pass

    def btnstate(self, btn):
        if btn.text() == "Camera":
            if btn.isChecked() == True:
                self.rob.set_tcp(self.rob.camtcp)
            else:
                self.rob.set_tcp(self.rob.tcp)

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.CancelFeed()
        self.rob.terminate()
        return super().closeEvent(a0)

    def updatepos(self):
        pos = self.rob.get_xyz()
        self.ui.labelX.setText("%0.4f"%pos[0])
        self.ui.labelY.setText("%0.4f"%pos[1])
        self.ui.labelZ.setText("%0.4f"%pos[2])
        orient_euler = self.rob.get_euler()
        self.ui.labelRx.setText("%0.4f"%orient_euler[0])
        self.ui.labelRy.setText("%0.4f"%orient_euler[1])
        self.ui.labelRz.setText("%0.4f"%orient_euler[2])
        orient_euler = self.rob.get_euler()
        #j = self.rob.robot.Joints()
        joints = self.rob.getj()
        self.ui.labela1.setText("%0.4f"%joints[0])
        self.ui.labela2.setText("%0.4f"%joints[1])
        self.ui.labela3.setText("%0.4f"%joints[2])
        self.ui.labela4.setText("%0.4f"%joints[3])
        self.ui.labela5.setText("%0.4f"%joints[4])
        self.ui.labela6.setText("%0.4f"%joints[5])

    def printXYZ(self):
        pos = np.around(self.rob.get_xyz(), 4).tolist()
        print(pos)

    def printJoints(self):
        pos = np.around(self.rob.getj(), 4).tolist()
        print(pos)

    def goxm(self):
        try:    
            val = 0.001*float(self.ui.Xstep.text())
            if self.ui.AddtoTCP.isChecked():
                tcp = self.rob.get_tcp()
                tcp[0]-=val
                self.rob.set_tcp(tcp)
                print(f"New tcp is {tcp}")
                return
            if self.ui.Base.isChecked():
                self.rob.mvr2x(-val, wait=False)
            else:
                self.rob.mvr2xTCP(-val, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.") 

    def goxp(self):
        try:
            val = 0.001*float(self.ui.Xstep.text())
            if self.ui.AddtoTCP.isChecked():
                tcp = self.rob.get_tcp()
                tcp[0]+=val
                self.rob.set_tcp(tcp)
                print(f"New tcp is {tcp}")
                return
            if self.ui.Base.isChecked():
                self.rob.mvr2x(val, wait=False)
            else:
                self.rob.mvr2xTCP(val, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
    
    def goym(self):
        try:
            val = 0.001*float(self.ui.Ystep.text())
            if self.ui.AddtoTCP.isChecked():
                tcp = self.rob.get_tcp()
                tcp[1]-=val
                self.rob.set_tcp(tcp)
                print(f"New tcp is {tcp}")
                return
            if self.ui.Base.isChecked():
                self.rob.mvr2y(-val, wait=False)
            else:
                self.rob.mvr2yTCP(-val, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
        
    def goyp(self):
        try:
            val = 0.001*float(self.ui.Ystep.text())
            if self.ui.AddtoTCP.isChecked():
                tcp = self.rob.get_tcp()
                tcp[1]+=val
                self.rob.set_tcp(tcp)
                print(f"New tcp is {tcp}")
                return
            if self.ui.Base.isChecked():
                self.rob.mvr2y(val, wait=False)
            else:
                self.rob.mvr2yTCP(val, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
    
    def gozm(self):
        try:
            val = 0.001*float(self.ui.Zstep.text())
            if self.ui.AddtoTCP.isChecked():
                tcp = self.rob.get_tcp()
                tcp[2]-=val
                self.rob.set_tcp(tcp)
                print(f"New tcp is {tcp}")
                return
            if self.ui.Base.isChecked():
                self.rob.mvr2z(-val, wait=False)
            else:
                self.rob.mvr2zTCP(-val, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
        
    def gozp(self):
        try:
            val = 0.001*float(self.ui.Zstep.text())
            if self.ui.AddtoTCP.isChecked():
                tcp = self.rob.get_tcp()
                tcp[2]+=val
                self.rob.set_tcp(tcp)
                print(f"New tcp is {tcp}")
                return
            if self.ui.Base.isChecked():
                self.rob.mvr2z(val, wait=False)
            else:
                self.rob.mvr2zTCP(val, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")

    def goRxm(self):
        self.goRxp(-1)

    def goRxp(self, sign=1.0):
        if sign==False:
            sign = 1
        try:
            val = float(self.ui.RxStep.text())
            #self.rob.robot.rx = val/360*3.141592
            if self.ui.Base.isChecked():
                frame='base'
            else:
                frame='tcp'
#            if self.ui.Camera.isCheched():
#                frame = 'camera'
            self.rob.rotx(sign*val, frame, wait=False)
            self.updatepos()

        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
    
    def goRym(self):
        self.goRyp(-1)
        
    def goRyp(self, sign=1.0):
        if sign==False:
            sign = 1
        try:
            val = float(self.ui.RyStep.text())
            if self.ui.Base.isChecked():
                frame='base'
            else:
                frame='tcp'
#            if self.ui.Camera.isCheched():
#                frame = 'camera'
            self.rob.roty(sign*val, frame, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
    
    def goRzm(self):
        self.goRzp(-1)

    def goRzp(self, sign=1):
        if sign==False:
            sign = 1
        try:
            val = float(self.ui.RzStep.text())
            if self.ui.Base.isChecked():
                frame='base'
            else:
                frame='tcp'
#            if self.ui.Camera.isCheched():
#                frame = 'camera'
            self.rob.rotz(sign*val, frame, wait=False)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
    
    def send_coords(self):
        try:
            coords_string = self.ui.Coordinates.text()
            coords_string.strip()
            coords_string_array = coords_string.split(',')
            coords = []
            pose = self.rob.robot.Pose()
            for i in range(3):
                coords.append(float(coords_string_array[i]))
                coords[i] = coords[i] * 1000
            if self.ui.Quaternion.isChecked() == False:
                for i in range(3, 6):
                    coords.append(float(coords_string_array[i]))     
                if self.ui.Euler.isChecked():
                    coords[3], coords[5] = coords[5], coords[3]
                    if self.ui.Base.isChecked():
                        #KUKA uses euler representation
                        pose = rkmath.KUKA_2_Pose(coords)
                    else:
                        r_pose = rkmath.KUKA_2_Pose(coords) # [mm, deg] for Euler to pose
                        pose = pose*r_pose
                else:
                    if self.ui.Base.isChecked():
                        pose = rkmath.UR_2_Pose(coords)
                    else:
                        r_pose = rkmath.UR_2_Pose(coords)
                        pose = pose*r_pose
            else:
                for i in range(3, 7):
                    coords.append(float(coords_string_array[i]))
                if self.ui.Base.isChecked():
                    pose = rkmath.quaternion_2_pose(coords[3:7])
                    pose.setPos(coords[0:3])
                else:
                    r_pose = rkmath.quaternion_2_pose(coords[3:7])
                    r_pose.setPos(coords[0:3])
                    pose = pose*r_pose
            self.rob.robot.MoveJ(pose)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Target Reach Error", "This target cannot be reached by the robot")
        except ValueError:
            if coords_string == "":
                QMessageBox.information(self, "No values entered", "No values have been entered")
            else:
                QMessageBox.warning(self, "Non-float values entered", "Non-float values have been entered")
                self.ui.Coordinates.clear()
        except IndexError:
            QMessageBox.warning(self, "Invalid set of coordinates", "The coordinates entered were invalid")
           
if __name__ == "__main__":
#    import logging
#    logging.basicConfig(level=logging.INFO)

    app = QApplication(sys.argv)
    if sys.argv[1] == "roboDK":
        from urxe.robDK import roboUR3
        from robodk.robolink import TargetReachError
        import robodk.robomath as rkmath
        rob = roboUR3()
    else:
#        rob = UR(sys.argv[1])
        import robot12idb as rb
        rob = rb.UR3(sys.argv[1])
    a = tweakRobot(rob)
    app.exec_()
    rob.terminate()
    sys.exit(0)