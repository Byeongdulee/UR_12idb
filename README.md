# Description
This python package urxe is based on urx, https://github.com/Byeongdulee/python-urx or its original, https://github.com/SintefManufacturing/python-urx.
urxe is an extension of urx for Robotiq wrist camera and gripper and rtmon for a newer OS.

This package contains a specific example for UR robots at the APS 12ID beamlines. Robots are defined in list_of_robots.json. You should edit the json file, robot12idb.py, and camera_tools.py for your robots.

# Brief history
The secondary monitor of python-urx was rewritten for better use at the APS.
Since the rtde code of python-urx does not support many feature required for 12ID operation, it is replaced with ur-rtde distributed by Universal Robot.

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
```
```python
> import urxe
```
or
```python
> from urxe import robUR
```
If you intend to use robot12idb.py, it will be convenient to use without installation.

# Prerequsite
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
> rob.move2z(0.1) # this will move rob along z by 0.1 meter
> rob.mvz(0.1) # this will move rob to z=0.1 position
> rob.move2zTCP(0.1) # this will move rob along the TCP z coordinate by 0.1 meter
> rob.rotz(30, coordinate='tcp') # rotate by 30 degree around the TCP z coordinate
> rob.rotz(30, coordinate='base') # rotate by 30 degree around the base z coordinate
> rob.rotz(30, coordinate='camera') # rotate by 30 degree around the camera z coordinate (view direction)
> rob.rotate([1, 1, 1], 30) # roate around the <1,1,1> direction in the TCP coordinate by 30 degree 
> rob.rotate_ref([1, 0, 1], [0.1, 0.1, 0.1], 30) # roate around the position [0.1, 0.1, 0.1] in the TCP coordinate by 30 degree around the <1,0,1> vector in TCP coordinate. 
> rob.get_euler() # get an euler angle
> rob.measureheight() # let the arm move along z direction until it bumps to a surface.
> rob.activate_gripper() 
> rob.grab()
> rob.loosen() # or use rob.release(). To use this function, you will need to define a variable named 'rq_pos' in the teach pendent.
> rob.tweak_around_camera_axis(30)
```

# At the beamline 12IDB
The UR3 robot at APS 12IDB equips with a Robotiq gripper and a Robotiq wrist camera. 
The robotiq camera comes with a long USB cable that goes to the UR control computer. In order to use this camera using python, you must plug the USB cable into the computer that runs this python code and then a separate 24V power is needed.
See camera_tools.py for examples.
```python
> import robot12idb as rb
> rob = rb.UR3()
> rb.cameratools.showcamera(rob)
```
It will pop up a camera feed window. Press "h" key for help.


When you don't use the 12idb robot, define yours without a camera.
```python
> from urxe import robUR
> rob = robUR.UR("164.54.xxx.xxx", cameratype=0) 
> rob.move2z(0.1)
```

# 12ID roboDK model
Download and install RoboDK from https://robodk.com/.
Run RoboDK and load your rdk file. An example is found under the robodk_model folder.
```python
> from urxe import robDK
> rob = robDK.roboUR3()
> rob.move2x(0.1)
```

# How to use TweakRobot.py
ROBODK example.
robDK.py is an example code to control a UR3 simulation robot defined in RoboDK. You will need RoboDK to use this.

Example:
Under the robodk_model folder, double click on "12IDBfinal.rdk" on windows machine.
Then, run the python code
```sh
> python TweakRobot.py roboDK
```
To run your own UR robot, you will need to enter its IP.
If you would like to tweak your robot, try
Example:
```sh
> python TweakRobot.py 164.54.xxx.xxx
```

