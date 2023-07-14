"""
Python library to control an UR robot through its TCP/IP interface
"""
__version__ = "0.1.0"
import sys
sys.path.append('rtde')
try:
    from . import robotiq_gripper_control
    from . import robotiq_preamble
    from . import robUR
except:
    raise ModuleNotFoundError