from msilib.schema import RadioButton
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox 
from PyQt5 import uic
#from roboUR3 import roboUR3
from robDK import roboUR3
from robUR3 import UR3
import argparse

import sys
from robodk.robolink import TargetReachError
import robodk.robomath as rkmath

class tweakRobot(QMainWindow):
    def __init__(self, rob):
        super(tweakRobot, self).__init__()
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        guiName = "TweakRobot.ui"
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
        self.updatepos()
        self.ui.show()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
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

    def goxm(self):
        try:    
            val = 0.001*float(self.ui.Xstep.text())
            if self.ui.Base.isChecked():
                self.rob.move2x(-val)
            else:
                self.rob.move2xTCP(-val)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.") 

    def goxp(self):
        try:
            val = 0.001*float(self.ui.Xstep.text())
            if self.ui.Base.isChecked():
                self.rob.move2x(val)
            else:
                self.rob.move2xTCP(val)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
    
    def goym(self):
        try:
            val = 0.001*float(self.ui.Ystep.text())
            if self.ui.Base.isChecked():
                self.rob.move2y(-val)
            else:
                self.rob.move2yTCP(-val)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
        
    def goyp(self):
        try:
            val = 0.001*float(self.ui.Ystep.text())
            if self.ui.Base.isChecked():
                self.rob.move2y(val)
            else:
                self.rob.move2yTCP(val)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
    
    def gozm(self):
        try:
            val = 0.001*float(self.ui.Zstep.text())
            if self.ui.Base.isChecked():
                self.rob.move2z(-val)
            else:
                self.rob.move2zTCP(-val)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")
        
    def gozp(self):
        try:
            val = 0.001*float(self.ui.Zstep.text())
            if self.ui.Base.isChecked():
                self.rob.move2z(val)
            else:
                self.rob.move2zTCP(val)
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot move further", "The robot cannot be moved in this direction any further.")

    def goRxm(self):
        try:
            val = float(self.ui.RxStep.text())
            if self.ui.TCP.isChecked():
                self.rob.rotx(-val)
            else:
                self.rob.rotate('x', -val, 'base')
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
        
    def goRxp(self):
        try:
            val = float(self.ui.RxStep.text())
            if self.ui.TCP.isChecked():
                self.rob.rotx(val)
            #self.rob.robot.rx = val/360*3.141592
            else:
                self.rob.rotate('x', val, 'base')
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
    
    def goRym(self):
        try:
            val = float(self.ui.RyStep.text())
            if self.ui.TCP.isChecked():
                self.rob.roty(-val)
            #self.rob.robot.ry = -val/360*3.141592
            else:
                self.rob.rotate('y', -val, 'base')
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
        
    def goRyp(self):
        try:
            val = float(self.ui.RyStep.text())
            if self.ui.TCP.isChecked():
                self.rob.roty(val)
            #self.rob.robot.ry = val/360*3.141592
            else:
                self.rob.rotate('y', val, 'base')
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
    
    def goRzm(self):
        try:
            val = float(self.ui.RzStep.text())
            if self.ui.TCP.isChecked():
                self.rob.rotz(-val)
            #self.rob.robot.rz = -val/360*3.141592
            else:
                self.rob.rotate('z', -val, 'base')
            self.updatepos()
        except TargetReachError:
            QMessageBox.warning(self, "Cannot rotate further", "The robot cannot be rotated in this direction any further.")
        
    def goRzp(self):
        try:
            val = float(self.ui.RzStep.text())
            if self.ui.TCP.isChecked():
                self.rob.rotz(val)
            #self.rob.robot.rz = val/360*3.141592
            else:
                self.rob.rotate('z', val, 'base')
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
                    if self.ui.TCP.isChecked():
                        r_pose = rkmath.KUKA_2_Pose(coords) # [mm, deg] for Euler to pose
                        pose = pose*r_pose
                    else:
                        #KUKA uses euler representation
                        pose = rkmath.KUKA_2_Pose(coords)
                else:
                    if self.ui.TCP.isChecked():
                        r_pose = rkmath.UR_2_Pose(coords)
                        pose = pose*r_pose
                    else:
                        pose = rkmath.UR_2_Pose(coords)
            else:
                for i in range(3, 7):
                    coords.append(float(coords_string_array[i]))
                if self.ui.TCP.isChecked():
                    r_pose = rkmath.quaternion_2_pose(coords[3:7])
                    r_pose.setPos(coords[0:3])
                    pose = pose*r_pose
                else:
                    pose = rkmath.quaternion_2_pose(coords[3:7])
                    pose.setPos(coords[0:3])
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
        rob = roboUR3()
    else:
        rob = UR3(sys.argv[1])
    a = tweakRobot(rob)
    app.exec_()
    rob.terminate()
    sys.exit(0)