''' Alignment / procedure scripts for the UR3 robot at 12IDB of APS.

These are high-level, sequenced procedures that drive a UR3 ``rob`` instance
(see robot12idb.UR3). They were moved out of robot12idb.py to keep the class
definitions separate from the beamline-specific run scripts.

Usage::

    from robot12idb import UR3
    from scripts.script_ur3_12idb import auto_align_12idb_remote_heater
    rob = UR3('UR3')
    auto_align_12idb_remote_heater(rob)
'''
import sys
import os
# Make the repo root importable so ``robot12idb`` resolves when this script is
# run/imported from the scripts/ directory.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import time
from robot12idb import april_tag_size


def auto_align_12idb_remote_heater(rob):

    dist2ATtag = 0.3
    barlength = 0.11
    gripper_width = 0.02

    # starting the procedure from the default position...
    rob.goto_default()
    print("")
    print("**************************")
    print(f"Camera will point at the april tag on the reference frame.")
    rob.moveto(rob.Waypoint_camera4heater)
    rob.camera.capture()
    rob.camera.capture()
    ret = rob.center_camera2apriltag()
    if not ret:
        print("No april tag is found.")
        print("No further alignment.")
        rob.goto_default()
        return False
    print("")
    print("An april tag is found and centered to the camera feed.")
    d = rob.camera.getATdistance(rob.camera.decoded)
    # keep the distance from camera to the april tag.
    print(f"Camera will be relocated to {dist2ATtag}m away from the tag.")
    rob.mvr2x(dist2ATtag - d)
    # read camera position and Z align the robot arm.
    cp = rob.get_camera_position()
    rob.move2z(cp.pos[2])
    rob.set_orientation()
    rob.grab()
    print("")
    print("")
    print("Confirming the distance from the tag by touching.")
    rob.bump(x=-0.2, backoff=0.05)
    rob.mvr2z(0.05)
    print("")
    print("")
    print("Move to the center of the bar.")
    rob.mvr2x(-(barlength/2+0.05+gripper_width/2))
    print("")
    print("")
    print("Rotate the gripper.")
    rob.rotz(-90)
    print("")
    print("")
    print("Aligning the position along the beam by touching.")
    rob.mvr2y(-0.04)
    rob.mvr2z(-0.04)
    rob.bump(y=0.1, backoff=0.02)
    rob.mvr2z(0.04)
    # go to the center of the heater along the beam direction.
    rob.mvr2y(0.032)
    # z position fine tuning.
    print("")
    print("")
    print("Checking the z position by touching.")
    v_standoff = 0.02
    rob.bump(z=-0.1, backoff=v_standoff)
    p0 = rob.get_xyz()
    # in-plane tilt tuning..
    print("")
    print("In the following, tilt around z will be checked.")
    z_tempdown = v_standoff+0.005
    rob.mvr2y(0.015)
    rob.mvr2z(-z_tempdown)
    rob.mvr2x(barlength/2)
    rob.bump(y=-0.01)
    p1 = rob.get_xyz()
    rob.mvr2y(0.005)
    rob.mvr2x(-barlength)
    rob.bump(y=-0.01)
    p2 = rob.get_xyz()
    rob.mvr2y(0.005)
    rob.mvr2x(barlength/2)
    ang = math.atan((p2[1]-p1[1])/barlength)*180/math.pi
    rob.rotz(ang)
    rob.mvr2z(z_tempdown)
    print("")
    print(f"The heater is tilted by {ang} degree. Taken into account.")
    rob.moveto(p0.tolist()[0:3])
    # move down
    rob.release()
    rob.mvr2z(-v_standoff-0.015)
    rob.set_current_as_sampledown()
    rob.movegripperup_totransport()
    print("")
    print("A test run will start in a second.")
    time.sleep(3)
    rob.dropofftest()
    print("Done.")
    print("")
    print("Ready for returing sample.")
    print("use rob.moveMagazine2FrameN(N) to return the reference frame to the slot N.")
    print("then, rob.returnsample() to transport it.")

def auto_align_12idb_standard_holder(rob):

    dist2ATtag = 0.3
    barlength = 0.11
    gripper_width = 0.023

    # starting the procedure from the default position...
    rob.goto_default()
    print("")
    print("**************************")
    print(f"Camera will point at the april tag on the reference frame.")
    rob.release()
    rob.moveto(rob.Waypoint_camera4standard)
    # align with camera
    rob.rotx(-30, coordinate='camera')
    rob.mvr2x(0.04)
    rob.grab()
    rob.bump(x=-0.1, backoff=0.005)
    rob.mvr2z(0.03)
    rob.mvr2x(-1*(gripper_width/2+barlength/2)-0.005)
    rob.bump(z=-0.1, backoff = 0.005)
    rob.release()
    rob.mvr2z(-0.02-0.005)
    rob.set_current_as_sampledown()
    rob.movegripperup_totransport()
    print("")
    print("A test run will start in a second.")
    time.sleep(3)
    rob.dropofftest()
    print("Done.")
    print("")
    print("Ready for returing sample.")
    print("use rob.moveMagazine2FrameN(N) to return the reference frame to the slot N.")
    print("then, rob.returnsample() to transport it.")

def auto_align_12idb_standard_holder2(rob):
    rob.camera.AT_physical_size = april_tag_size['standard']
    rob.goto_default()
    rob.transport_from_default_to_samplestage_up()
    rob.mvr2z(-0.1)
    rob.camera_face_down()
    rob.bring_QR_to_camera_center()
    rob.grippertip2camera()
    h = rob.measureheight()
    print(f"The TCP is from {h}m above a surface.")
