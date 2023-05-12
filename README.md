aaa
# Prerequsite
python-urx, which you can install from pypi.org using pip install.

This requires python-urx at
https://github.com/Byeongdulee/python-urx

urrtde.py requires https://github.com/UniversalRobots/RTDE_Python_Client_Library at a folder 'ur-rtde' or change the folder name in urrtde.py.

# History
The secondary monitor of python-urx was rewritten for better use at the APS.
Since the rtde code of python-urx does not support many feature required for 12ID operation, it is replaced with ur-rtde distributed by Universal Robot.

# Robotiq Camera
The robotiq camera comes with a long USB cable that goes to the UR control computer. In order to use this camera using python, you must plug the USB cable into the computer that runs this python code.
See camera_tools.py for examples.
> import robUR3
> rob = robUR3.UR3("164.54.xxx.xxx") 
> import camera_tools as ct
> ct.showcamera(rob)
It will pop up a camera feed window. Press "h" key for help.

# Direction of using TweakRobot.py
ROBODK example.
robDK.py is an example code to control a UR3 simulation robot defined in RoboDK. You will need RoboDK to use this.

Example:
Under the robodk_model folder, double click on "12IDBfinal.rdk" on windows machine.
Then, run the python code
> python TweakRobot.py roboDK

To run your own UR robot, you will need to enter its IP.
If you would like to tweak your robot, try
Example:
> python TweakRobot.py 164.54.xxx.xxx

# robDK.py
A beamline 3D model for roboDK is included.

