"""Test-suite bootstrap, loaded by pytest before it collects this directory.

These tests exercise robot communication: no robot, no camera and no 3-D maths
are involved. Several packages in the import graph are awkward to install
(math3d) or only needed for camera work (cv2, pupil_apriltags), and pytest
imports UR_12idb/__init__.py -- which pulls in camera_tools -- while collecting,
so without them collection fails even for unrelated tests.

Stub only what is genuinely missing. Anything actually installed is used as-is;
nothing here shadows a real package.
"""
import os
import sys
import types

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# python-urx is expected as a sibling checkout, matching urxe/robot.py.
PYTHON_URX = os.path.join(os.path.dirname(REPO_ROOT), "python-urx")

for _path in (REPO_ROOT, PYTHON_URX):
    if _path not in sys.path:
        sys.path.insert(0, _path)


class _StubModule(types.ModuleType):
    """Module whose every attribute is a permissive placeholder class."""

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        placeholder = type(name, (), {"__init__": lambda self, *a, **k: None})
        setattr(self, name, placeholder)
        return placeholder


def _stub_if_missing(name, factory=None):
    """Install a stand-in for `name`, but only if it cannot be imported."""
    if name in sys.modules:
        return
    try:
        __import__(name)
    except ImportError:
        sys.modules[name] = factory() if factory else _StubModule(name)


def _math3d_stub():
    """A math3d that looks like 3.x.

    common/m3d.py applies its math3d 4.x compatibility shims at import time;
    giving the stub a get_pose_vector() makes it take the 3.x path and leave
    the stub alone. The code under test uses math3d only for isinstance().
    """

    class Transform(object):
        def __init__(self, *args, **kwargs):
            pass

        pose_vector = property(lambda self: [0.0] * 6)

        def get_pose_vector(self):
            return [0.0] * 6

    class _Placeholder(object):
        def __init__(self, *args, **kwargs):
            pass

    stub = types.ModuleType("math3d")
    stub.Transform = Transform
    stub.Orientation = _Placeholder
    stub.Vector = _Placeholder
    stub.__version__ = "3.4.1-stub"

    for _sub in ("utils", "orientation", "transform", "vector"):
        mod = types.ModuleType("math3d." + _sub)
        mod._deprecation_warning = lambda *args, **kwargs: None
        sys.modules["math3d." + _sub] = mod
    return stub


_stub_if_missing("math3d", _math3d_stub)
for _name in ("cv2", "pupil_apriltags", "epics", "imutils",
              "pyzbar", "pyzbar.pyzbar"):
    _stub_if_missing(_name)
