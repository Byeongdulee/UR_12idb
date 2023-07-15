"""
Python library to control an UR robot through its TCP/IP interface
"""
__version__ = "0.1.0"
import sys
sys.path.append('urxe')
try:
    from . import robot
    from . import ursecmon
    from . import urrtmon
    from . import urmon_parser
except:
    raise ModuleNotFoundError