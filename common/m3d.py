"""Single point of access to math3d for the whole UR stack.

Import this instead of ``math3d`` everywhere::

    from common import m3d
    pose = m3d.Transform([x, y, z, rx, ry, rz])
    orient = m3d.Orientation(matrix)

Importing this module applies the math3d 4.x compatibility shims below to the
*shared* ``math3d.Transform`` class, so every downstream user — robUR,
robot12idb, python-urx (urxe/rtde) — gets the 3.x-compatible API regardless of
which module is imported first. Because the compat now lives in this low-level
module (pulled in by common/__init__ before robUR), it no longer depends on
robot12idb having been imported.

How to use m3d
==============

A ``Transform`` is a full pose: a position (``.pos``) plus an orientation
(``.orient``). The UR robot represents a pose as a 6-element list
``[x, y, z, rx, ry, rz]`` — position in metres, and ``rx, ry, rz`` the
orientation as a *rotation vector* (axis-angle, in radians).

Convert a UR 6-element pose list to a Transform
-----------------------------------------------
Pass the list straight to the constructor::

    pose = [x, y, z, rx, ry, rz]
    t = m3d.Transform(pose)          # 6-list  -> Transform

And back to a UR 6-list (works on math3d 3.x and 4.x thanks to the compat
shim below)::

    pose = t.pose_vector.tolist()    # Transform -> [x, y, z, rx, ry, rz]
    # t.get_pose_vector() is equivalent.

Read/replace just the position::

    xyz = t.pos.list                 # [x, y, z]
    t.set_pos([x, y, z])             # or t.pos = m3d.Vector(x, y, z)

Set the orientation
-------------------
``t.orient`` is an ``Orientation``. Set it any of these ways::

    # (a) from a rotation vector (the rx, ry, rz the robot uses):
    t.orient.set_rotation_vector(m3d.Vector(rx, ry, rz))

    # (b) from a 3x3 rotation matrix (list of rows):
    t.orient.set_array([[r11, r12, r13],
                        [r21, r22, r23],
                        [r31, r32, r33]])
    #   equivalently: t.orient = m3d.Orientation(matrix)

    # (c) get the rotation vector back out (a RotationVector; use .array for a
    #     numpy array). You can also pass it straight to the robot's
    #     set_orientation_rad(...):
    rx, ry, rz = t.orient.get_rotation_vector().array

Note: this math3d build does not support constructing an ``Orientation``
directly from a ``Vector`` (``m3d.Orientation(m3d.Vector(...))`` raises), so use
``set_rotation_vector`` for the rotation-vector case. ``m3d.Orientation(matrix)``
with a 3x3 matrix does work.

Euler angles
------------
There is no in-place Euler setter; build a fresh ``Orientation`` from the angles
(in radians) and assign it to ``t.orient``::

    import math
    t.orient = m3d.Orientation.new_from_euler([a, b, c], encoding='XYZ')

    # from degrees:
    t.orient = m3d.Orientation.new_from_euler(
        [math.radians(ax), math.radians(ay), math.radians(az)], encoding='XYZ')

    # read them back (returns radians):
    a, b, c = t.orient.to_euler('XYZ')

``encoding`` is three characters from ``xyzXYZ`` naming the axis sequence.
Lower case (``'xyz'``) means *extrinsic* rotations (about the original fixed
axes); UPPER case (``'XYZ'``) means *intrinsic* rotations (about the moving
axes). Common Tait-Bryan choices: ``'ZYX'`` = roll-pitch-yaw, ``'XYZ'`` =
yaw-pitch-roll; proper Euler like ``'ZXZ'`` reuses the first axis for the third.
(UR's ``set_euler`` builds an extrinsic-'xyz' orientation, i.e. the same as
``new_from_euler([a, b, c], encoding='xyz')``.)

Rotate a pose
-------------
To rotate in the TCP frame, start from the current pose::

    t = self.get_pose()      # Transform of the current TCP

To rotate in the robot base frame, start from a fresh Transform::

    t = m3d.Transform()      # identity, expressed in the base frame

Then apply a rotation about an axis (angle in radians)::

    t.orient.rotate_xt(ang)  # about X
    t.orient.rotate_yt(ang)  # about Y
    t.orient.rotate_zt(ang)  # about Z
    t.orient.rotate_t(axis, ang)   # about an arbitrary axis vector

and send it back with ``self.set_pose(t)``.
"""
import math3d as _math3d
import math3d.transform  # ensure the submodule is loaded (m3d.transform.Transform)

# Re-export the names the UR stack actually uses so existing call sites keep
# working unchanged (``m3d.Transform(...)``, ``m3d.Orientation(...)``,
# ``m3d.transform.Transform``).
from math3d import Transform, Orientation, transform  # noqa: F401

# ── math3d 4.x compatibility ───────────────────────────────────────────────
# math3d 4.0.0 changed Transform.pose_vector to return a PoseVector object
# (with only a `.array`), whereas 3.x returned a plain ndarray, and it removed
# Transform.get_pose_vector() entirely. The UR robot stack (UR_12idb AND
# python-urx) was written against the 3.x ndarray API — it calls
# pose_vector.tolist(), indexes/iterates pose_vector, and passes it straight to
# movel/movec, and calls get_pose_vector(). Restore the ndarray-returning
# behaviour so all of that works on both 3.x and 4.x. Patching the shared
# math3d.Transform class fixes every downstream user, including python-urx.
# No-op on 3.x (get_pose_vector exists).
if not hasattr(_math3d.Transform, "get_pose_vector"):
    _m3d_pv_prop = _math3d.Transform.pose_vector

    def _m3d_pose_vector_ndarray(self, _p=_m3d_pv_prop):
        pv = _p.fget(self)
        return pv.array if hasattr(pv, "array") else pv

    _math3d.Transform.pose_vector = property(_m3d_pose_vector_ndarray,
                                             getattr(_m3d_pv_prop, "fset", None),
                                             getattr(_m3d_pv_prop, "fdel", None))
    _math3d.Transform.get_pose_vector = lambda self: self.pose_vector

# math3d 4.x prints a stdout deprecation banner (via utils._deprecation_warning,
# which also runs inspect.stack() on every call) for methods the robot stack
# still uses, e.g. Vector.dist_squared. During motion this floods the log and
# adds per-call overhead. Silence it — these are informational only.
try:
    import math3d.utils as _m3d_utils
    _m3d_utils._deprecation_warning = lambda *a, **k: None
except Exception:
    pass


def __getattr__(name):
    """Delegate any other attribute (Vector, PositionVector, ...) to the patched
    math3d module, so this stays a drop-in replacement for ``import math3d``."""
    return getattr(_math3d, name)
