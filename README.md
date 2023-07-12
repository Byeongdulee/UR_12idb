# Description
This package contains general python codes for UR robots (urxe) and some specific examples for their operation at the APS 12ID beamlines. The robots available at the synchrotron beamline are defined in list_of_robots.json. You should edit the json file for your robots. Also, make your own beamline operation code (robot12idb.py), camera_tools.py, and others.

# Brief history
This repository contains a python package for controlling UR robots at synchrotron beamlines. Initially, the APS 12ID-B beamline, which this code was dveloped for, intended to use a set of common commands for a number of different robots for the beamline operation. Later, some robots are retired, leaving only UR robot and a virtual simulator, roboDK in this package.

# Credits
This python package urxe is based on urx, https://github.com/Byeongdulee/python-urx or its original, https://github.com/SintefManufacturing/python-urx.
urxe is an extension of urx for Robotiq wrist camera and gripper and rtmon for a newer OS (5.8 and higher).

The secondary monitor of python-urx was rewritten for better use at the APS.
Since the rtde code of python-urx does not support many features required for 12ID operation, it is replaced with ur-rtde distributed by Universal Robot.

# Installation
You can download from github or clone, for example https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository.
If you would like to use only urxe, you may install using downloaded setup.py
```sh
cd UR_12idb
python setup.py install
```
Or, you can use without installation as
```sh
cd UR_12idb
python
```
```python
> import urxe
```
or
```python
> from urxe import robUR
```
If you intend to use robot12idb.py, it will be convenient to use without installation.

# Prerequsites
* python-urx, which you can install from pypi.org using pip install.

This requires python-urx at
https://github.com/Byeongdulee/python-urx

* urrtde.py requires https://pypi.org/project/ur-rtde/. Have a look at also https://github.com/UniversalRobots/RTDE_Python_Client_Library at a folder 'ur-rtde' or change the folder name in urrtde.py.
* math3d, https://gitlab.com/morlin/pymath3d
* pyzbar, pip install pyzbar, note Window error message on https://pypi.org/project/pyzbar/
* pupil-apriltags, pip install pupil-apriltags, https://pypi.org/project/pupil-apriltags/
* roboDK, pip install robodk, https://pypi.org/project/robodk/



# Examples
```python
> from urxe import robUR
> rob = robUR.UR("164.54.xxx.xxx") 
```
Translations
```python
> rob.mvr2z(0.1) # this will move rob along z by 0.1 meter
> rob.move2z(0.1) # this will move rob to z=0.1 position
> rob.mvr2zTCP(0.1) # this will move rob along the TCP z coordinate by 0.1 meter
> rob.moveto([0.1, 0.2, 0]) # move the tcp to the position in the base coordinate
```
Rotations
```python
> rob.rotz(30, coordinate='tcp') # rotate by 30 degree around the TCP z coordinate
> rob.rotz(30, coordinate='base') # rotate by 30 degree around the base z coordinate
> rob.rotz(30, coordinate='camera') # rotate by 30 degree around the camera z coordinate (view direction)
> rob.rotate([1, 1, 1], 30) # roate around the <1,1,1> direction in the TCP coordinate by 30 degree 
> rob.rotate_ref([1, 0, 1], [0.1, 0.1, 0.1], 30) # roate around the position [0.1, 0.1, 0.1] in the TCP coordinate by 30 degree around the <1,0,1> vector in TCP coordinate. 
```
Orientations
```python
> rob.get_euler() # get an euler angle
> rob.set_orientation() # Z aligning the robot.
```
UR script programs
```python
> rob.measureheight() # let the arm move along z direction until it bumps to a surface.
> rob.robot.bump(x=0.1, backoff=0.01) # let the arm move along +x until it bumps to a surface. Once it bumps, it will back off opposite direction by backoff value
```
Gripper
```python
> rob.activate_gripper() 
> rob.grab()
> rob.loosen() # or use rob.release(). To use this function, you will need to define a variable named 'rq_pos' in the teach pendent.
> rob.finger.get_position() # gripper position 0 close, 1 for open.
# For this, urx needs be installed from github.
```
Camera tools
```python
> rob.camera_face_down() # make the camera face down (-Z).
> rob.camera.capture() # capture an image
> rob.capture_camera() # capture an image and save it with position as filename.
> rob.move_toward_camera(value, north=0, east=0.01) # move along the camera axis, north, and east direction.
> rob.roll_around_camera(value, distance, dir='y') # rotate around the object at the distance around the tcp's y axis by value. The direction can be only x or y.
> rob.tweak_around_camera_axis(30)
> rob.fingertip2camera() # bring the finger tip to camera position and having Z aligned.
```

# Simulator: a roboDK model
Download and install RoboDK from https://robodk.com/.
Run RoboDK and load your rdk file. An example is found under the robodk_model folder. Double click on "12IDBfinal.rdk", which is a virtual model of APS beamline 12IDB, on windows machine.
```python
> from urxe import robDK
> rob = robDK.roboUR3()
> rob.mvr2x(0.1)
```
This code is not complete, meaning that it does not have all the functions in robUR.py. 

# Tweak
## with ROBODK
```sh
> python TweakRobot.py roboDK
```
## with UR robot 
You will need to enter its IP. If you would like to tweak your robot, try
Example:
```sh
> python TweakRobot.py 164.54.xxx.xxx
```

# At the beamline 12IDB
The UR3 robot at APS 12IDB equips with a Robotiq gripper and a Robotiq wrist camera. 
The robotiq camera comes with a long USB cable that goes to the UR control computer. This USB cable is for both gripper and camera. This is a default configuration of urxe. You can move the UR robot with watching the camera feed.

```python
> import robot12idb as rb
> rob = rb.UR3()
> rb.cameratools.showcamera(rob)
```
It will pop up a camera feed window. Press "h" key for help.

## Direct control of the wrist camera
The USB cable can be connected to a PC that runs this python code, which will allow you to access the camera as a webcam. Then, you can use all camera functions such as focusing, higher resolution images, and so on. To make this work, a separate 24V power is needed for the camera. Also, for the gripper to work, its M8 connector needs to be connected to the tool communication M8 connector, which may require a non-electric mount between camera and UR tool flange.
See camera_tools.py for examples.

You can configure it without a camera.
```python
> from urxe import robUR
> rob = robUR.UR("164.54.xxx.xxx", cameratype=0) 
> rob.mvr2z(0.1)
```

## Remote heater operation
APS 12IDB uses UR3 for loading/unloading samples to the remote heater.
When staff mounts the remote heater on the sample table, its new position needs to be configured for the robot. This can be done automatically as long as the heater position is not too far from the previous location. Note that the sample reference with an april tag on the side of its handle should be inserted to the heater prior to running the code below: 
```python
> import robot12idb as rb
> rob = rb.UR3()
> rb.auto_align_12idb_remote_heater(rob)
```
Once the heater is configured, test whether robot can insert a sample without an issue.
```python
> rob.dropofftest() 
```