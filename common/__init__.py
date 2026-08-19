"""
Python library to control an UR robot through its TCP/IP interface
"""
__version__ = "0.1.0"

try:
    from . import m3d  # first: applies math3d 4.x compat before anything uses it
    from . import urmodes  # leaf enums; before robUR so the driver cycle stays broken
    from . import urcamera
    from . import urdashboard
    from . import urpop
    from . import utils
    from . import robUR
    from . import tc_pipet
    from . import ursocket
except:
    raise ModuleNotFoundError