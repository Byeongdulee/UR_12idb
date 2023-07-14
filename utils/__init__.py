"""
Python library to control an UR robot through its TCP/IP interface
"""
__version__ = "0.1.0"

try:
    from . import urcamera
    from . import urdashboard
    from . import urpop
    from . import utils
except:
    raise ModuleNotFoundError