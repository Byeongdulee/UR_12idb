aaa
# Prerequsite and History
python-urx, which you can install from pypi.org using pip install.

Part of codes here are duplicates of my python-urx fork.
https://github.com/Byeongdulee/python-urx

Some classes in this folder inherit the corresponding classes of python-urx.
Some classes are completely new and written based on the URscript of python-urx.
The secondary monitor of python-urx was rewritten for better use at the APS.
Since the rtde code of python-urx does not support many feature required for 12ID operation, it is replaced with ur-rtde that I modified from an example code distributed by Universal Robot.

ur-rtde is included.

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
