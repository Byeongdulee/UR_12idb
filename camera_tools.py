#from cProfile import run
import math
#from xmlrpc.server import _DispatchArity0
from common import m3d  # centralized math3d (4.x compat applied in common.m3d)
import time
import numpy as np
#from imutils import paths
import cv2
from common.urcamera import decodeQR, showQRcode, default_imgH, default_imgV, focus_threshold, camera_f
from common.urcamera import decodeAT, cal_AT2pose
from common.urcamera import detect_AT
from common.urcamera import camera
from threading import Thread
import json
import os

# -- AprilTag teach accuracy ---------------------------------------------------
# How closely the tag has to sit on the optical axis before centering is called
# done, as a distance at the tag rather than a share of the frame. Passed to
# center_camera2apriltag(tol_m=...), which converts it to pixels using the
# standoff measured on each iteration.
AT_CENTER_TOL_M = 0.0002        # 0.2 mm
# Iterations allowed to get there. The old fraction-of-frame default converged
# in one or two moves because it was ~1.6 mm wide; 0.2 mm needs more room, and
# each move is only as repeatable as the arm.
AT_CENTER_MAX_ITER = 8
# Standoff to square the camera to a tilted tag from, in meters. The
# perspective a tilt produces goes as (tag size / distance)^2, and it is tiny:
# a 12 mm tag at 0.26 m foreshortens its far edge by 0.46 px for a 7 deg tilt,
# which is below the noise the corner detector works with -- so the pose
# solver picks between its two near-identical solutions on sub-pixel noise and
# flips branches as soon as the arm moves. That is the "more than one new
# minima found" the solver reports. At 0.15 m the same tilt is worth ~1.3 px,
# which is workable; 0.10 m would give ~3 px and resolve better, but brought
# the camera down too close to the hardware to be usable.
AT_SQUARE_UP_DISTANCE = 0.15
# How square the camera has to be to the tag, in radians. Set by what the
# image can actually resolve at AT_SQUARE_UP_DISTANCE, not by what would be
# nice: with a 12 mm tag at 0.15 m, ~0.1 px of corner localization buys about
# 0.55 deg. A tolerance below that floor cannot be met, and asking for one
# only spends every iteration chasing noise before failing. Raise the standoff
# and this has to come up with it -- the signal falls off as 1/distance^2.
AT_ALIGN_TOL = 1e-2             # 0.57 deg
# Per-move clamp while squaring up, in degrees. Big enough to close a several
# degree tilt within max_steps, small enough to keep the tag in frame.
AT_ALIGN_STEP = 2.0

# Why the last AprilTag routine gave up. The reasons used to go to the server
# console and nowhere else, so a failed teach reached the operator as the
# generic "failed to find a tag" -- true, but not an answer to why it failed.
# PAL12idb.locate_apriltag() reads this back into the error it raises, which
# is what the GUI ends up showing.
last_search_failure = None


def _fail(message, value=False):
    """Record why a routine gave up, print it, and return `value`."""
    global last_search_failure
    last_search_failure = message
    print(message)
    return value

pos_sam = [-4.60838969e-01, -5.05650395e-01,  2.31693123e-01,  2.28368253e+00,
       -2.15707100e+00,  1.01565770e-03]
pos_mag = [-3.89803673e-01, -1.50716439e-01, -3.46071435e-02,  2.26448275e+00,
       -2.17747082e+00,  1.91570580e-03]

def test(rob):
    rob.release()
    rob.mvr2z(0.1)
    rob.moveto(pos_sam)
    rob.mvr2z(-0.1)
    rob.grab()
    fp = rob.gripper.get_position()
    rob.mvr2z(0.1)
    rob.moveto(pos_mag, vel=0.1)
    if fp>1: # there is a sample 
        rob.dropsample()
    else:
        rob.release()
        rob.mvr2z(-0.1)
        rob.grab()
        rob.mvr2z(0.1)
        rob.moveto(pos_sam)
        rob.dropsample()
    rob.mvr2z(0.1)

def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def run_measuredistance(rob):
    rob.grab()
    rob.put_tcp2camera()
    if (rob.camera.QRtiltangle>0):
        rob.rotz(90)
    else:
        rob.rotz(-90)
    print(rob.bump(z=-1, backoff=0.1))

def run_centering(rob):
    rob.camera._running = True
    rob.bring_QR_to_camera_center(referenceName = "AT")
    rob.camera._running = False

def run_centering_1QR(rob):
    rob.camera._running = True
    rob.bring_QR_to_camera_center(referenceName = "1QR")
    rob.camera._running = False

class QRref:
    def __init__(self):
        self.data = b'Follow me'
        self.height = [0.05, 0.1, 0.15, 0.2, 0.21, 0.25,0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65]
        self.edgelength = [229.0, 176.0, 142.0, 119.0, 116.0, 103.0,90.0,80.0, 72.8, 66.0, 62.0,57.0,52.0, 48.0]
        self.size = 55.4 # mm
        self._distances = np.array([0,0,0,0])
        self._coordinates = np.array([[0,0],[0,0],[0,0],[0,0]])

    def mean(self):
        return np.mean(self._distances)

    def diffmean(self):
        a = np.diff(self._distances)
        return a.mean()

    @property
    def distances(self):
        return self._distances
    # a setter function
    @distances.setter
    def distances(self, dist):
        self._distances = np.array(dist)
    @property
    def coordinates(self):
        return self._coordinates
    # a setter function
    @coordinates.setter
    def coordinates(self, coord):
        self._coordinates = np.array(coord)

    def get_height(self):
        # return actual height in 'm' unit.
        m = self.mean()
        y = np.interp(m, self.edgelength[::-1], self.height[::-1])
        return y

    def get_pixeldistance(self):
        # return actual distance of a pixel in 'm' unit
        m = self.mean()
        y = self.size/m
        return y 

def findAT2go(rob):
    rob.camera.capture()
    rob.center_aprilTag()
    rob.camera.capture()
    rob.orient2aprilTag()
    rob.camera.capture()
    rob.center_aprilTag()

def pickupshift(rob):
    #rob.mvr2x(0.0005)
    rob.mvr2y(-0.001)

#z direction : -0.011
#x direction : 0.0025
#y direction : -0.001    

QRfollowme = QRref()

def decodefollowme(rob):
    rob.camera.capture()
    rob.camera.decode()
    if rob.camera.QRdata != QRfollowme.data:
        return 0, 0, 0, [0, 0]
    QRfollowme.distances = rob.camera.QRedgelength
    h = QRfollowme.get_height()
    pd = QRfollowme.get_pixeldistance()
    print(f"Height is {h} and pixel distance is {pd}mm/pixel.")
    if abs(QRfollowme.diffmean()/QRfollowme.mean()) > 0.0001:
        print("Camera is not perpendicular to the QR code")
    else:
        print("Camera is perpendicular to the QR code within +-3 degree.")
    ang = rob.camera.analyzeroll_QR()
    print(f"Roll angle is {ang}")
    tilt = rob.camera.analyzetilt_QR()
    print(f"Tilt angle is {tilt}")
    return h, pd, ang, tilt
    
def search_position(rob):
    v = [-3.60658169e-01, -5.26188384e-01, -1.91809449e-01, -5.72779495e-01,
        3.08887702e+00,  6.93712436e-05]
    rob.moveto(v)
    rob.camera_y()

def grid_pictures(rob, dX, dY):
    v0 = rob.get_xyz().tolist()
    nX = 6
    nY = 3
    for x in range(-nX,nX+1, 1):
        for y in range(-nY, nY+1, 1):
            vn = [v0[0]+x*dX/nX,v0[1]+y*dY/nY,v0[2],v0[3],v0[4],v0[5]]
            rob.moveto(vn)
            time.sleep(6)
            rob.capture_camera()
    rob.moveto(v0)

def motion_pictures(rob, Radius, height):
    v0 = rob.get_xyz().tolist()
    for i in range(0,18, 1):
        goto_phi(rob, v0, i*10, Radius, height)
        time.sleep(6)
        rob.capture_camera()
    rob.moveto(v0)

# rob.roll_around_camera(10, obj_distance+0.18)    
def motion_pictures2(rob, radius=0.12):
    v0 = rob.get_xyz().tolist()
    rob.roll_around_camera((0-9)*3, radius)
    for i in range(0, 18, 1):
        rob.roll_around_camera(3, radius)    
        time.sleep(6)
        rob.capture_camera()
    rob.moveto(v0)

def goto_phi(rob, v0, ang, Radius, height):
    # cylindrical 
    ang = ang/180*math.pi
    x = Radius*math.sin(ang+math.pi/2)
    y = Radius*math.cos(ang+math.pi/2)
    v = [v0[0]+x, v0[1]+y, v0[2]+height, v0[3], v0[4], v0[5]]
    np = m3d.Transform(v)
    np.orient.rotate_zb(math.pi/2-ang)
    rob.robot.set_pose(np, wait=True, acc=0.5, vel=0.5)

def rotmat_z(ang):
    ang = ang*math.pi/180
    mat = [[math.cos(ang), math.sin(ang), 0], [-math.sin(ang), math.cos(ang), 0], [0, 0, 1]]
    beta = math.atan2(-mat[2][0], math.sqrt(mat[0][0]**2+mat[1][0]**2))
    alpha = math.atan2(mat[1][0]/math.cos(beta), mat[0][0]/math.cos(beta))
    gamma = math.atan2(mat[2][1]/math.cos(beta), mat[2][2]/math.cos(beta))
    return [alpha, beta, gamma]


def followhands(rob):
    #from cvzone.FaceDetectionModule import FaceDetector
    from cvzone.HandTrackingModule import HandDetector
    detector = HandDetector(detectionCon = 0.8, maxHands=2)
    #detface = FaceDetector()
    flipflop = True
    TrackingMode = 0 # 0 for no tracking, 1 for translation, 2 for rotation.

    while 1:    
    #print(vidcap.get(cv2.CAP_PROP_FOCUS)) #Always throws back 68.0, not sure why. More info at https://docs.opencv.org/3.4/d8/dfe/classcv_1_1VideoCapture.html
    
        # Capture
        ret, frame = rob.camera.capture()
        # Ensure frame is a writable NumPy array for OpenCV drawing functions.
        if not isinstance(frame, np.ndarray):
            frame = np.array(frame)
        if not getattr(frame, "flags", None) or not frame.flags.writeable:
            frame = frame.copy()
        
        gray = cv2.cvtColor(cv2.flip(frame,0), cv2.COLOR_BGR2GRAY)
        #fm = variance_of_laplacian(gray)
        #text = "Not Blurry"
        #if fm<focus_threshold:
        #    text = "Blurry"
        #cv2.putText(frame, "{}: {:.2f}".format(text, fm), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        #cv2.putText(frame, "a: auto, s: scan, m: manual", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)
        #check if empty
        if not ret:
            print("Retrieve frame failed...")
            break

        #Get size
        if flipflop:
            h, w, _ = frame.shape
            print("Camera Frame Size {}".format([w,h]))
            flipflop = False

        # Finding hands.
        hands, frame = detector.findHands(frame) # draw
        #frame, bboxs = detface.findFaces(frame) # find face.
        #if bboxs:
        #    # bboxInfo - "id", "bbox", "score", "center"
        #    center = bboxs[0]["center"]
        #    cv2.circle(frame, center, 5, (255, 0, 255), cv2.FILLED)

        if hands:
            # Hand 1
            hand1 = hands[0]
            lmList1 = hand1["lmList"] # List of 21 Landmarks points
            bbox1 = hand1["bbox"] # Bounding box info x, y, w, h
            centerPoint1 = hand1["center"] # center of the hand cx, cy
            handType1 = hand1["type"] # hand Type: left or right
            fingers1 = detector.fingersUp(hand1)
            TipofindexFinger1 = lmList1[8]
            center = [centerPoint1[0]-w/2, centerPoint1[1]-h/2]
            #print(sum(fingers1))
            if sum(fingers1)==5:
                bring_hand_to_camera_center(rob, bbox1, center)
            #print(bbox1)
            #print(centerPoint1)
            #print(centerPoint1[0]-w/2, centerPoint1[1]-h/2)
            #length, info, frame = detector.findDistance(lmList1[8], lmList1[12], frame)
            #length, info = detector.findDistance(lmList1[8], lmList1[12]) # not draw.
            
            #print(lmList1)
            # if len(hands)==2:
            #     hand2 = hands[1]
            #     lmList2 = hand2["lmList"] # List of 21 Landmarks points
            #     bbox2 = hand2["bbox"] # Bounding box info x, y, w, h
            #     centerPoint2 = hand2["center"] # center of the hand cx, cy
            #     handType2 = hand2["type"] # hand Type: left or right
            #     fingers2 = detector.fingersUp(hand2)
            #     TipofindexFinger2 = lmList2[8]

        #hands = detector.findHands(frame, draw=False) # no draw
        # Display 
        cv2.imshow('frame', frame)
        key = cv2.waitKey(20) & 0xFF
        if key == 48:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,400) #This sets the focus to a value of i
        if key == 49:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,450) #This sets the focus to a value of i
        if key == 50:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,500) #This sets the focus to a value of i
        if key == 51:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,550) #This sets the focus to a value of i
        if key == 97: #a
            rob.camera.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,1)
        if key == 120: #x
            rob.camera.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,0)
        if key == 115: #s
            rob.camera.scanfocus()
        if key == 27:
            break
#        foc = rob.camera.get_foc()
    #    print("The current focus value is: {}".format(foc))
        time.sleep(0.1)
    cv2.destroyAllWindows()


def run_pick_sequence(rob, QRdist, grabdepth = 0.01, dist_from_base = 0.02):
    # Pick sequence: move the TCP to the camera position, plunge down by
    # the measured tag distance, grip, hold 5 s, release, and retract.
    if not isinstance(QRdist, (int, float)) or QRdist <= 0:
        print("No valid tag distance (QRdist). Point the camera at a tag first.")
        return
    try:
        rob.release()
        QRdist = QRdist + grabdepth
        print(f"Pick sequence: descending {QRdist:.3f} m to grip...")
        #rob.move_toward_camera(0, north=-0.01, east=0.0)
        #if not rob.is_Z_aligned():
        #    rob.put_tcp2camera()
        rob.mvr2z(-QRdist, vel=0.1)   # down (base -Z)
        rob.grab()
        time.sleep(1)
        #rob.release()
        rob.mvr2z(QRdist, vel=0.1)    # back up
        time.sleep(1)
        rob.mvr2z(-QRdist+dist_from_base, vel=0.05)   # down (base -Z + 0.02)
        rob.release()
        rob.mvr2z(QRdist, vel=0.1)
        print("Pick sequence done.")
        print("Distance down to the object is {:.3f} m.".format(QRdist))
    except Exception as ex:
        print(f"Pick sequence failed: {ex}")


def average_apriltag_pose(cam, camera_params, min_margin, N=10):
    """Capture N frames and collect the AprilTag pose from each frame that has
    exactly one valid detection. Returns (eulers, dists) lists (length <= N),
    used for time-averaging the tag pose to reduce per-frame noise."""
    eulers = []
    dists = []
    for _ in range(N):
        try:
            ret2, f2 = cam.capture()
        except Exception:
            continue
        if not ret2 or f2 is None:
            continue
        if not isinstance(f2, np.ndarray):
            f2 = np.array(f2)
        g2 = cv2.cvtColor(f2, cv2.COLOR_RGB2GRAY)
        dets = detect_AT(g2, camera_params, tag_size=cam.AT_physical_size)
        dets = [d for d in dets
                if d.hamming == 0 and d.decision_margin >= min_margin]
        if len(dets) != 1 or dets[0].pose_R is None:
            continue
        try:
            e, tvec, o = cal_AT2pose(dets[0])
        except ValueError:
            continue
        if e is None:
            continue
        eulers.append(e)
        dists.append(cam.getATdistance(dets[0]))
    return eulers, dists


def open_second_webcam(skip_index=None, max_index=5):
    """Probe device indices 0..max_index and return the first webcam that opens
    and delivers a frame, skipping skip_index (the robot's USB camera).

    Returns (VideoCapture, index) or (None, None) if nothing usable is found.
    """
    for idx in range(max_index + 1):
        if skip_index is not None and idx == skip_index:
            continue
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            continue
        ret, _ = cap.read()
        if not ret:
            cap.release()
            continue
        print("Second webcam found at device index {}.".format(idx))
        return cap, idx
    print("No second webcam detected.")
    return None, None


def showcamera(rob, codetype = 0, obj_distance=0.15):
    # codetype ==1 for QR code.
    # obj_distance: distance between the gripper tip to the object. measure using rob.measureheight() function.
    rob.camera.QRdistance = ""
    flipflop = True
    QRpos = []
    QRdist = None
    rob.camera.AT_physical_size = 0.010
    worker = None  # background thread for blocking robot actions
    def dispatch(fn, *args, **kwargs):
        # Run a blocking robot action off the display loop so the camera feed
        # keeps updating. Ignore new actions while one is still running.
        nonlocal worker
        if worker is not None and worker.is_alive():
            print("Busy: a robot action is still running.")
            return
        worker = Thread(target=fn, args=args, kwargs=kwargs, daemon=True)
        worker.start()
    # Auto-detect a second webcam and show it in its own window beside the
    # robot camera. Skip the robot's own USB device so we don't grab it twice.
    skip_idx = rob.camera.device if rob.camera.connectiontype == 'usb' else None
    webcam, _webcam_idx = open_second_webcam(skip_index=skip_idx)
    webcam_win = 'webcam'
    webcam_placed = False
    webcam_zoom = 1.0  # digital zoom factor for the webcam window ('+'/'-')
    # This loop captures continuously; tell threaded robot actions to reuse the
    # frames we grab here rather than capturing again (avoids a two-reader race).
    rob.camera._running = True
    while 1:
        # Capture
        ret, frame = rob.camera.capture()
        # Ensure frame is a writable NumPy array for OpenCV drawing functions.
        if not isinstance(frame, np.ndarray):
            frame = np.array(frame)
        if not getattr(frame, "flags", None) or not frame.flags.writeable:
            frame = frame.copy()
#        frame = cv2.normalize(
#        frame, None, alpha=0, beta=0.9*255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1
#    )
        imgdata = frame[:,:,::-1].copy()
        rob.camera.image = imgdata
        #frame = rob.camera.image
        #check if empty
        if not ret:
            print("Retrieve frame failed...")
            break

        #Get size
        if flipflop:
            h, w, _ = frame.shape
            print("Camera Frame Size {}".format([w,h]))
            flipflop = False

        # Display 
#        isambient = False
        if codetype==1:
            QRcode = decodeQR(frame)
            rob.camera.image = frame
            data, rectcoord, qrsize, dist = rob.camera.decode()
            if len(data) ==1:
                QRpos = rob.camera.QRposition
                QRdist = rob.camera.QRdistance
        #        if len(rob.camera.QRposition)>0:
        #            QRpos = rob.camera.QRposition
        #            QRdist = rob.camera.QRdistance
                if len(QRpos)>0:
                    cv2.putText(frame, "{}: [{:.2f}, {:.2f}]".format("position", QRpos[0],QRpos[1]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    try:
                        cv2.putText(frame, "{}: {:.2f}mm".format("distance", QRdist*1000), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    except:
                        pass
                if hasattr(rob.camera, 'QRdata'):
                    showQRcode(QRcode, frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # Camera intrinsics for pose estimation: [fx, fy, cx, cy].
        # Principal point is taken as the image center of the current frame.
        fx = fy = rob.camera.camera_f
        cx, cy = w / 2.0, h / 2.0
        # Shared detector + lock: a robot action dispatched to a worker thread
        # detects too, and libapriltag cannot be driven from two threads at once.
        r = detect_AT(gray, [fx, fy, cx, cy],
                      tag_size=rob.camera.AT_physical_size)
        # tag16h5 is very prone to false positives (spurious detections in
        # noise/texture when no real tag is present). Reject them by requiring
        # a clean decode (hamming==0) and a strong decision_margin. Real tags
        # score ~50+; false positives are typically well below ~30.
        AT_MIN_MARGIN = 30.0
        r = [d for d in r if d.hamming == 0 and d.decision_margin >= AT_MIN_MARGIN]
        # Reset the latest pose each frame so key handlers act on a current
        # detection (None when no valid tag is currently visible).
        euler = None
        if len(r)==1:
            #print(rob.camera.AT_physical_size)
            r = r[0]
            (ptA, ptB, ptC, ptD) = r.corners
            #rob.camera.decoded = r
            QRpos = r.center
            QRdist = rob.camera.getATdistance(r)
            #ret = decodeAT(img=frame, F=[], cam_f=camera_f, imgH=default_imgH, imgV=default_imgV)
            # Pose is now populated (pose_R / pose_t). Convert to Euler angles.
            euler = None
            if r.pose_R is not None:
                # AprilTag pose estimation on a planar tag is occasionally
                # ambiguous and returns a degenerate/left-handed rotation
                # matrix, which scipy rejects. Skip that frame instead of
                # crashing the live view.
                try:
                    euler, rpost, o = cal_AT2pose(r)
                except ValueError as ex:
                    euler = None
                    print(f"Skipping bad AprilTag pose this frame: {ex}")
                if euler is not None:
                    cv2.putText(frame,
                        "rpy: [{:.1f}, {:.1f}, {:.1f}] deg and {:.3f} m".format(euler[0], euler[1], euler[2], QRdist),
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    #print(f"Euler angles: {euler}")
                    #print(f"Translation vector: {rpost}")
                    #print(f"Orientation: {o}")
            R, T = rob.camera.H2RT(r.homography)
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            #print("[INFO] tag family: {}".format(tagFamily))   
            # 
            #                 imageData = np.asarray(bytearray(resp), dtype="uint8")
            #pilImage=Image.open(io.BytesIO(imageData))
            #pilImage = np.asarray(pilImage)
            #pilImage = frame
            #rob.camera.image = pilImage
            #rob.camera.imgH = pilImage.shape[1]
            #rob.camera.imgV = pilImage.shape[0]
            #rob.camera.camera_f = camera_f/default_imgH*rob.camera.imgH         
        cv2.imshow('frame', frame)
        # Show the second webcam side-by-side in its own window.
        if webcam is not None:
            wret, wframe = webcam.read()
            if wret:
                if webcam_zoom > 1.0:
                    # Digital zoom: crop the center 1/zoom of the frame and
                    # scale it back up to the original size.
                    wh, ww = wframe.shape[:2]
                    cw, ch = int(ww / webcam_zoom), int(wh / webcam_zoom)
                    x0, y0 = (ww - cw) // 2, (wh - ch) // 2
                    wframe = cv2.resize(wframe[y0:y0+ch, x0:x0+cw], (ww, wh),
                                        interpolation=cv2.INTER_LINEAR)
                cv2.imshow(webcam_win, wframe)
                if not webcam_placed:
                    # Place the webcam window just to the right of the robot cam.
                    cv2.moveWindow(webcam_win, w + 60, 0)
                    webcam_placed = True
        #rob.camera.decode2QR()
        key = cv2.waitKey(20) & 0xFF
        if key == 48:  #0
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,400) #This sets the focus to a value of i
        if key == 49: #1
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,450) #This sets the focus to a value of i
        if key == 50: #2
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,475) #This sets the focus to a value of i
        if key == 51: #3
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,500) #This sets the focus to a value of i
        if key == 52:
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,525) #This sets the focus to a value of i
        if key == 53:
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,550) #This sets the focus to a value of i
        if key == 54:
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,600) #This sets the focus to a value of i
        if key == 55:
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,650) #This sets the focus to a value of i
        if key == 105: #i
            dispatch(rob.move_toward_camera, 0, north=0.025)
        if key == 106: #j
            dispatch(rob.move_toward_camera, 0, north=0, east=-0.025)
        if key == 107: #k
            dispatch(rob.move_toward_camera, 0.02, north=0, east=0.0)
        if key == 108: #l
            dispatch(rob.move_toward_camera, 0, north=0, east=0.025)
        if key == 109: #m
            dispatch(rob.move_toward_camera, 0, north=-0.025, east=0.0)
        if key == 111: #o
            dispatch(rob.move_toward_camera, -0.02, north=0, east=0.0)
        if key == 103: #g
            dispatch(run_pick_sequence, rob, QRdist, dist_from_base=0.005)
        #if key == 102: #f
        #    rob.roll_around_camera(-10, obj_distance+0.18)
        if key == 114: #r
            dispatch(rob.rotate_around_Zaxis_camera, 5)
        if key == 82: #R
            dispatch(rob.rotate_around_Zaxis_camera, -5)
        if key == 101: #e
            dispatch(rob.roll_around_camera, -10, QRdist)
        if key == 69: #E
            dispatch(rob.roll_around_camera, 10, QRdist)
        if key == 119: #w
            dispatch(rob.roll_around_camera, -10, QRdist, dir='x')
        if key == 87: #W
            dispatch(rob.roll_around_camera, 10, QRdist, dir='x')
        if key == 97: #a
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,1)
        if key == 120: #x
            if rob.camera.connectiontype == 'usb':
                rob.camera.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,0)
        if key == 115: #s
            if rob.camera.connectiontype == 'usb':
                rob.camera.scanfocus()
            else:
                dispatch(search_apriltag_by_tilt, rob)
        if key == 104: #h
            print("Help:")
            print("  focal point change: 0, 1, 2, .. 7")
            print("  focus mode: a(auto), x(manual), s(scanfocus USB / search AprilTag by tilt for IP)")
            print("  move robot: i(north),j(west),k(toward),l(east),m(south),o(away)")
            print("  rotate around camera Z: r(+5), R(-5)")
            print("  roll around camera (Y axis): e(-10), E(+10)")
            print("  roll around camera (X axis): w(-10), W(+10)")
            print("  put camera to TCP: d")
            print("  put TCP to camera: u")
            print("  center camera on AprilTag / QR: c")
            print("  time-average AprilTag pose: t")
            print("  pick sequence (put tcp to camera, grip, retract): g")
            print("  print QR/AprilTag info: p")
            print("  measure distance: M")
            print("  webcam zoom: + (in), - (out)")
            print("  exit: q or ESC")
        if key == 113: #q
            #print(QRcode)
            break
        if key == 112: #p
            #print(QRcode)
            # r may be a list of detections or a single detection object.
            if isinstance(r, list):
                if len(r) > 0 and hasattr(r[0], 'homography'):
                    print(r[0].homography)
                else:
                    print(r)
            else:
                if hasattr(r, 'homography'):
                    print(r.homography)
                else:
                    print(r)
            print(f"Center position is at [{QRpos}].")
            print(f"Distance from camera is {QRdist} m.")
        if key == 116: #t
            ''' time average the AprilTag pose over 10 frames and print the average translation and rotation. '''
            N = 20
            eulers, dists = average_apriltag_pose(
                rob.camera, [fx, fy, cx, cy], AT_MIN_MARGIN, N=N)
            if len(eulers) == 0:
                print("Time average: no valid AprilTag detections. Point the camera at a tag.")
            else:
                avg = np.mean(np.array(eulers), axis=0)
                avgdist = float(np.mean(dists))
                print(f"Time-averaged pose over {len(eulers)}/{N} frames:")
                print(f"  rpy      = [{avg[0]:.2f}, {avg[1]:.2f}, {avg[2]:.2f}] deg")
                print(f"  distance = {avgdist:.4f} m")
        if key == 100: #d
            dispatch(rob.put_camera2tcp)
        if key == 117: #u
            dispatch(rob.put_tcp2camera)
        if key == 121: #y
            pass
        if key == 43 or key == 61: #'+' / '=' : zoom the webcam window in
            if webcam is not None:
                webcam_zoom = min(webcam_zoom + 0.5, 8.0)
                print(f"Webcam zoom: {webcam_zoom:.1f}x")
        if key == 45: #'-' : zoom the webcam window out
            if webcam is not None:
                webcam_zoom = max(webcam_zoom - 0.5, 1.0)
                print(f"Webcam zoom: {webcam_zoom:.1f}x")
        if key == 77: #M
            dispatch(run_measuredistance, rob)
        if key == 99: #c
            if euler is not None:
                # An AprilTag is visible: center the camera on the tag.
                def _center_apriltag(rob):
                    try:
                        print("Centering to AprilTag...")
                        rob.center_camera2apriltag()
                    except Exception as ex:
                        print(f"center_camera2apriltag failed: {ex}")
                dispatch(_center_apriltag, rob)
            elif len(QRpos) > 0:
                # A QR code is visible: nudge it to the center, then run centering.
                dx = w/2-QRpos[0]
                dy = h/2-QRpos[1]
                dX = -dx/rob.camera.camera_f*QRdist
                dY = dy/rob.camera.camera_f*QRdist
                def _center_qr(rob, dX, dY):
                    rob.move_toward_camera(distance=0, north=dY, east=dX, acc=0.5, vel=0.5)
                    run_centering(rob)
                dispatch(_center_qr, rob, dX, dY)
            else:
                print("Nothing to center on. Point the camera at an AprilTag or QR code.")
        if key == 27:
            break
#        foc = rob.camera.get_foc()
    #    print("The current focus value is: {}".format(foc))
        time.sleep(0.1)
    rob.camera._running = False
    if webcam is not None:
        webcam.release()
    cv2.destroyAllWindows()
#    rob.camera.stop()
    #t.join()

def resolve_robot_ip(name='UR5', robots_file=None):
    """Look up a robot's control-box IP from list_of_robots.json by name.

    `list_of_robots.json` next to this module is only ever a *default* --
    this does not scan the current working directory or a sibling
    `RobotList/` folder for something to silently override it with (a robot
    control box is too easy to point at the wrong address that way). Pass
    `robots_file`, or set the `UR12IDB_ROBOTS_FILE` environment variable, to
    use a different file instead.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    fn = robots_file or os.environ.get('UR12IDB_ROBOTS_FILE') or os.path.join(here, 'list_of_robots.json')
    if not os.path.exists(fn):
        raise FileNotFoundError("list_of_robots.json not found: %s" % fn)
    with open(fn) as f:
        return json.load(f)[name]

def showcamera_ip(ip=None, name='UR5'):
    """View the IP camera and detect AprilTags without a robot object.

    Useful when the robot is protective-stopped or you just want the camera:
    the IP camera streams over HTTP (port 4242) independent of the UR
    controller, so no rob connection is needed.

        showcamera_ip()                # looks up UR5's IP from list_of_robots.json
        showcamera_ip(name='UR3')      # another robot by name
        showcamera_ip(ip='164.54.x.x') # explicit IP

    Keys: t = time-average pose over 10 frames, p = print last detection,
          q or ESC = quit. No robot-motion keys (there is no robot here).
    """
    if ip is None:
        ip = resolve_robot_ip(name)
    print(f"Opening IP camera at {ip} ...")
    cam = camera(ip)                      # connectiontype == 'ip'
    cam.AT_physical_size = 0.010
    AT_MIN_MARGIN = 30.0
    flipflop = True
    w = h = None
    while 1:
        # capture() raises if the HTTP request fails; treat that as a dropped
        # frame and keep going instead of tearing down the viewer.
        try:
            ret, frame = cam.capture()
        except Exception as ex:
            print(f"Frame grab failed: {ex}")
            if cv2.waitKey(200) & 0xFF in (27, ord('q')):
                break
            continue
        if not ret or frame is None:
            if cv2.waitKey(200) & 0xFF in (27, ord('q')):
                break
            continue
        if not isinstance(frame, np.ndarray):
            frame = np.array(frame)
        if flipflop:
            h, w, _ = frame.shape
            print("Camera Frame Size {}".format([w, h]))
            flipflop = False
        # IP frames arrive as RGB; convert for correct display colors.
        disp = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        fx = fy = cam.camera_f
        cx, cy = w / 2.0, h / 2.0
        r = detect_AT(gray, [fx, fy, cx, cy], tag_size=cam.AT_physical_size)
        r = [d for d in r if d.hamming == 0 and d.decision_margin >= AT_MIN_MARGIN]
        euler = None
        if len(r) == 1:
            r = r[0]
            (ptA, ptB, ptC, ptD) = r.corners
            QRdist = cam.getATdistance(r)
            if r.pose_R is not None:
                try:
                    euler, rpost, o = cal_AT2pose(r)
                except ValueError as ex:
                    euler = None
                    print(f"Skipping bad AprilTag pose this frame: {ex}")
                if euler is not None:
                    cv2.putText(disp,
                        "rpy: [{:.1f}, {:.1f}, {:.1f}] deg and {:.3f} m".format(
                            euler[0], euler[1], euler[2], QRdist),
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            cv2.line(disp, ptA, ptB, (0, 255, 0), 2)
            cv2.line(disp, ptB, ptC, (0, 255, 0), 2)
            cv2.line(disp, ptC, ptD, (0, 255, 0), 2)
            cv2.line(disp, ptD, ptA, (0, 255, 0), 2)
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(disp, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(disp, r.tag_family.decode("utf-8"), (ptA[0], ptA[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow('frame', disp)
        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord('q')):
            break
        if key == ord('t'):
            N = 10
            eulers, dists = average_apriltag_pose(
                cam, [fx, fy, cx, cy], AT_MIN_MARGIN, N=N)
            if len(eulers) == 0:
                print("Time average: no valid AprilTag detections.")
            else:
                avg = np.mean(np.array(eulers), axis=0)
                print(f"Time-averaged pose over {len(eulers)}/{N} frames:")
                print(f"  rpy      = [{avg[0]:.2f}, {avg[1]:.2f}, {avg[2]:.2f}] deg")
                print(f"  distance = {float(np.mean(dists)):.4f} m")
        if key == ord('p'):
            if hasattr(r, 'homography'):
                print(r.homography)
            else:
                print("No current AprilTag.")
        time.sleep(0.1)
    cv2.destroyAllWindows()

def _detect_apriltag(rob, settle=5, tag_id=None, stop_event=None):
    """Capture one frame and return the AprilTag detection (or None).

    tag_id selects a specific tag number; when omitted and several tags are
    in view, the one nearest the image center is used. stop_event, if given,
    is checked each poll so an operator abort doesn't have to wait out the
    full settle timeout."""
    t0 = time.time()
    r = None
    while True:
        if stop_event is not None and stop_event.is_set():
            return None
        # When a live display loop (showcamera) is already capturing, reuse its
        # latest frame instead of grabbing our own.
        if not rob.camera._running:
            rob.camera.capture()
        r = rob.camera.decodeAT(tag_id=tag_id)      # populates rob.camera.decoded
        if r is not None:
            break
        time.sleep(0.1)  # Wait a bit before trying again
        if time.time() - t0 > settle:
            # Deliberately not _fail(): during the tilt grid this fires for
            # every pose that has no tag in it and the search goes on to
            # succeed, so recording it would bury the real reason under a
            # routine one. Callers that care say what they lost and where.
            print("No AprilTag detected after waiting {:.1f} s.".format(settle))
            return None
    return r

def approach_tag_distance(rob, target, tol=0.003, max_steps=6, max_travel=0.30,
                          stop_event=None):
    """Move the camera until it sits `target` m from the tag it is looking at.

    Returns the last measured camera-to-tag distance, or None if no tag could
    be read at all. rob.camera.AT_physical_size has to already be set to the
    size of the tag in view, since the measurement is proportional to it.

    Measure and move is iterated because each step is only as good as the
    frame behind it. max_travel caps the total distance this can command, so
    one bad reading cannot walk the arm into the hardware.
    """
    measured = None
    travelled = 0.0
    for _ in range(max_steps):
        if stop_event is not None and stop_event.is_set():
            return measured
        if _detect_apriltag(rob, stop_event=stop_event) is None:
            return measured
        measured = rob.camera.QRdistance
        step = measured - target
        print(f"AprilTag is {measured:.4f} m from the camera (target {target:.4f} m).")
        if abs(step) <= tol:
            break
        if travelled + abs(step) > max_travel:
            step = math.copysign(max_travel - travelled, step)
            if abs(step) <= 0:
                print(f"Stopping at the {max_travel:.3f} m travel limit.")
                break
        rob.mvr2z(-step)
        travelled += abs(step)
    return measured


def measure_tag_tilt(rob, samples=10, stop_event=None):
    """Mean tilt of the tag about the camera X and Y axes, in degrees.

    Returns (ex, ey, distance), or None if too few frames decoded.

    Pose estimation on a planar tag is bimodal: for a small, near-face-on tag
    two poses project almost identically and the solver flips between them
    from frame to frame -- that is the library's "Error, more than one new
    minima found". A single frame's euler is therefore not something to servo
    on. It is what made the squaring loop read 1.2 deg, apply a 1.2 deg
    correction, and then read 10.5 deg: no rotation that small can move the
    measurement that far, so the second reading was the other solution.

    Averaging `samples` frames is what this does about it. Note what that does
    and does not buy: it cuts the variance, but on a genuinely split sample the
    mean lands between the two solutions rather than on the right one, so the
    figure it returns is a compromise, not a measurement. Convergence then
    rests on the caller iterating with a clamped step.
    """
    ex, ey, dist = [], [], []
    for _ in range(samples):
        if stop_event is not None and stop_event.is_set():
            return None
        r = _detect_apriltag(rob, stop_event=stop_event)
        if r is None:
            continue
        info = cal_AT2pose(r)
        if len(info) != 3 or info[0] is None:
            continue
        ex.append(info[0][0])
        ey.append(info[0][1])
        dist.append(rob.camera.getATdistance(r))
    if len(ex) < 3:
        return _fail(f"Only {len(ex)} usable AprilTag pose(s) in {samples} frames; "
              "cannot measure the tilt.", None)
    ex, ey = np.asarray(ex), np.asarray(ey)
    mx, my = float(np.mean(ex)), float(np.mean(ey))
    # The spread is reported, not acted on. It is worth seeing: a large one
    # means the frames are not disagreeing by a little, they are reporting the
    # solver's two different solutions, and the average then sits between two
    # answers rather than on either. What makes that usable anyway is the
    # caller -- _square_camera_to_tag clamps each correction to AT_ALIGN_STEP
    # and re-measures, so it behaves as a feedback loop with gain below one
    # and can still walk in on a noisy but roughly centred estimate.
    deviation = np.maximum(np.abs(ex - mx), np.abs(ey - my))
    print(f"Tag tilt averaged over {len(ex)}/{samples} frames: "
          f"({mx:.3f}, {my:.3f}) deg, frame-to-frame spread up to "
          f"{deviation.max():.2f} deg.")
    return mx, my, float(np.mean(dist))


def calibrate_tilt_response(rob, probe=4.0, stop_event=None):
    """Measure how the reported tag tilt responds to a known camera rotation.

    Returns a 2x2 array J where J[i][j] is d(euler_i)/d(rotation_j), the
    rotations being about the camera X (j=0) and Y (j=1) axes, or None if the
    tag could not be measured throughout.

    This exists because assuming the mapping was wrong on the bench. Taking
    euler[0] to be corrected by a camera-X rotation and euler[1] by a camera-Y
    one -- which is what rob.orient2aprilTag() does -- drove euler[0] from
    -3.6 deg to -12.7 deg over three iterations while commanding +5.7 deg of
    X rotation to correct it. The rotation was not acting on the axis the
    measurement reports, so rather than guess again among the axis and sign
    combinations, rotate a known amount about each axis and difference the
    readings.

    Each probe rotation is undone before the next, so the arm ends where it
    started. Every measurement is taken with the tag centred, since an
    off-axis tag reads a tilt that is partly just viewing angle.
    """
    def _measure():
        rob.center_camera2apriltag(tol_m=AT_CENTER_TOL_M,
                                   max_iter=AT_CENTER_MAX_ITER)
        return measure_tag_tilt(rob, stop_event=stop_event)

    base = _measure()
    if base is None:
        return None
    ex0, ey0, distance = base
    baselines = [(ex0, ey0)]
    columns = []
    for axis, angles in (("X", [probe, 0.0]), ("Y", [0.0, probe])):
        if stop_event is not None and stop_event.is_set():
            return None
        print(f"Probing the tilt response to a {probe:.1f} deg camera {axis} rotation ...")
        rob.roll_around_camera(list(angles), distance)
        probed = _measure()
        rob.roll_around_camera([-angles[0], -angles[1]], distance)   # put it back
        if probed is None:
            return _fail(f"Lost the AprilTag while probing the {axis} response.", None)
        columns.append([(probed[0] - ex0) / probe, (probed[1] - ey0) / probe])
        # Re-read the baseline now the arm is back where it started. Repeating
        # a measurement *without* moving would understate the noise: the pose
        # solver picks consistently for a fixed viewpoint and changes its mind
        # when the viewpoint does, so the scatter that matters is the scatter
        # across a move-and-return, which is what the loop actually sees
        # between iterations.
        again = _measure()
        if again is None:
            return _fail(f"Lost the AprilTag after the {axis} probe.", None)
        baselines.append((again[0], again[1]))
    # columns are d(euler)/d(rot_axis); stack them as J[:, j]
    J = np.array(columns, dtype=float).T
    print("Tilt response matrix d(euler)/d(rotation):\n"
          f"    d(ex)/dX = {J[0][0]:+.3f}   d(ex)/dY = {J[0][1]:+.3f}\n"
          f"    d(ey)/dX = {J[1][0]:+.3f}   d(ey)/dY = {J[1][1]:+.3f}")
    det = float(np.linalg.det(J))
    if abs(det) < 0.05:
        return _fail(f"Tilt response is not invertible (det {det:+.4f}): a camera "
              "rotation barely moves the reported tilt, or moves both axes the "
              "same way. Cannot square up to the tag from this measurement.", None)
    b = np.asarray(baselines, dtype=float)
    noise = float(np.hypot(b[:, 0].std(ddof=1), b[:, 1].std(ddof=1)))
    print(f"Repeatability over {len(b)} returns to the same pose: {noise:.3f} deg "
          f"(ex {b[:, 0].std(ddof=1):.3f}, ey {b[:, 1].std(ddof=1):.3f}).")
    return J, noise


def _square_camera_to_tag(rob, step_in_radians, tol, max_steps, stop_event=None):
    """Rotate the camera until it looks straight down the tag's own normal.

    The face-down loop below aims at world Z, which is only the same thing
    when the tag lies flat. A tag that sits at an angle -- the flowcell's, in
    its cleaning station -- has to be squared up against itself instead, so
    this drives the tilt the detector reports to zero rather than driving the
    pose to a world-frame target.

    cal_AT2pose()'s euler[0]/euler[1] are the tag's tilt about the camera X
    and Y axes; euler[2] is its in-plane spin, which is not touched here --
    search_apriltag_by_tilt() corrects that separately with
    rotate_around_Zaxis_camera().

    Assumes the caller has already pushed the TCP out to the tag, so a
    rotation here pivots about the tag and keeps it in frame.
    """
    # Centre first, measure second, rotate last -- in that order, every time.
    # The rotation pivots about the extended camera TCP, which sits along the
    # flange Z rather than along the optical axis (camtcp carries a 30 deg
    # tilt), so a step both squares the camera up and slides the tag off the
    # axis. Reading the tilt in that state mixes in the off-axis viewing angle:
    # measuring after the rotation and before the re-centring is what made this
    # loop oscillate (3.6 -> 6.7 -> 1.6 -> 4.7 deg) instead of converging.
    #
    # The rotation itself goes through rob.roll_around_camera(), the same
    # primitive rob.orient2aprilTag() uses, rather than a hand-rolled
    # set_pose: it takes the pivot distance explicitly, rotates about the
    # camera X then Y, and puts the gripper TCP back when it is done.
    # Which camera rotation moves which reported axis is measured, not
    # assumed -- see calibrate_tilt_response.
    calibration = calibrate_tilt_response(rob, stop_event=stop_event)
    if calibration is None:
        return False
    J, noise = calibration
    # Stop at the noise, not below it. `tol` is a floor, not a target: asking
    # for better than the measurement repeats to is asking the loop to chase
    # its own scatter, which is what it spent all 24 steps doing on the bench
    # (corrections of ~2 deg against 2.2 deg of pass-to-pass noise).
    tol = max(tol, noise / 180 * math.pi)
    print(f"Squaring to within {tol/math.pi*180:.3f} deg "
          f"(measurement repeatability {noise:.3f} deg).")
    previous = None
    worse = 0
    for _ in range(max_steps):
        if stop_event is not None and stop_event.is_set():
            return _fail("AprilTag search stopped by operator.")
        # 1. On-axis: the tilt below is only meaningful with the tag centred.
        rob.center_camera2apriltag(tol_m=AT_CENTER_TOL_M,
                                   max_iter=AT_CENTER_MAX_ITER)
        # 2. Measure, over several frames -- see measure_tag_tilt for why a
        # single frame is not a measurement here.
        measured = measure_tag_tilt(rob, stop_event=stop_event)
        if measured is None:
            return False
        ex, ey, distance = measured
        tilt = math.hypot(ex, ey) / 180 * math.pi
        print(f"Camera is {tilt/math.pi*180:.3f} deg off the AprilTag normal "
              f"at {distance:.4f} m.")
        if tilt < tol:
            print("Camera is square to the AprilTag.")
            if noise / 180 * math.pi > AT_ALIGN_TOL:
                # Say what "square" actually means here. Converging against a
                # noise-derived tolerance does not mean the camera is aligned
                # to that figure -- it means the measurement cannot tell.
                print(f"  -- to within the {noise:.3f} deg the measurement "
                      "repeats to, not better. Residual tilt up to that much "
                      "may remain; a larger tag is what would tighten it.")
            return True
        if previous is not None and tilt > previous:
            # An averaged reading still moves around, and a re-centring move
            # changes the viewing angle a little, so one step going the wrong
            # way means nothing. Only a run of them says the loop is not
            # converging -- bailing on a single rise is what stopped the last
            # two attempts one step after a perfectly good 7.4 -> 1.2 deg.
            worse += 1
            print(f"Tilt grew after that step ({worse} in a row).")
            if worse >= 3:
                return _fail("Squaring up to the AprilTag is not converging; giving up.")
        else:
            worse = 0
        previous = tilt
        # 3. Solve for the rotation that nulls the measured tilt, using the
        # response measured above rather than assuming euler[0] <-> camera X.
        # J @ rotation = -(ex, ey), in degrees.
        try:
            rotation = np.linalg.solve(J, np.array([-ex, -ey], dtype=float))
        except np.linalg.LinAlgError:
            return _fail("Tilt response matrix became singular; giving up.")
        # Clamp so no single move exceeds `step`, keeping the tag in frame.
        commanded = math.hypot(rotation[0], rotation[1]) / 180 * math.pi
        scale = min(1.0, step_in_radians / commanded) if commanded else 1.0
        rob.roll_around_camera([rotation[0] * scale, rotation[1] * scale], distance)
    return _fail("Reached max_steps before the camera squared up to the tag.")


def roll_around_tag(rob, step=0.1, tol=AT_ALIGN_TOL, max_steps=24,
                    align_to_tag=False, stop_event=None):
    """Tilt the camera toward face-down in steps of at most ``step`` degrees,
    pivoting about the tag so it stays centered, until the camera faces down
    (within ``tol`` radians) or the tag is lost / ``max_steps`` is reached.
    Checked against stop_event before each step, so an abort takes effect
    between moves rather than only after max_steps.

    align_to_tag aims at the tag's own normal instead of world Z-down, for a
    tag that is not lying flat -- see _square_camera_to_tag."""
    r = _detect_apriltag(rob, stop_event=stop_event)
    if r is None:
        return _fail("No AprilTag in view; cannot roll around the tag.")
    distance = rob.camera.getATdistance(r)
    print(f"AprilTag is at {distance} from the wrist camera.")
    newtcp = list(rob.camtcp)
    newtcp[2] = distance
    rob.set_tcp(newtcp)
    step_in_radians = step / 180 * math.pi
    if align_to_tag:
        # Close in before squaring. The tilt is only measurable when the tag is
        # large in the frame (see AT_SQUARE_UP_DISTANCE), and `step` is taken
        # from AT_ALIGN_STEP rather than this function's default: the default
        # is sized for the face-down loop below, and a small per-move clamp
        # cannot close a several degree tilt within max_steps.
        # Put the gripper TCP back first: the extended pivot TCP set above is
        # about to be stale anyway (it was built from the distance measured
        # before this approach), and _square_camera_to_tag re-establishes it
        # per iteration through roll_around_camera.
        rob.set_tcp(rob.tcp)
        print(f"Closing to {AT_SQUARE_UP_DISTANCE:.3f} m to measure the tag's tilt ...")
        if approach_tag_distance(rob, AT_SQUARE_UP_DISTANCE,
                                 stop_event=stop_event) is None:
            return _fail("Lost the AprilTag while closing in to square up to it.")
        return _square_camera_to_tag(rob, step / 180 * math.pi, tol,
                                     max_steps, stop_event=stop_event)
    # Face-down rotation vector, built the same way rob.Zalign() does it:
    # roll = 180 deg, pitch = 0, and the heading (yaw) the arm already has.
    # A hard-coded [0, -pi, 0] is Z-down too, but at a yaw of its own, so
    # tilting toward it would drag the camera around Z on the way.
    Wp = list(rob.get_pose().get_pose_vector())     # [x, y, z, rx, ry, rz]
    rpy = rob.rotvec2rpy(Wp[3], Wp[4], Wp[5])
    target_rotvec = np.asarray(rob.rpy2rotvec(math.pi, 0.0, rpy[2]), dtype=float)
    for _ in range(max_steps):
        if stop_event is not None and stop_event.is_set():
            return _fail("AprilTag search stopped by operator.")
        pose = rob.get_pose()
        v = pose.orient.get_rotation_vector().array
        # A rotation vector and its negative describe the same half-turn, so
        # put v on the same side as the target before differencing them.
        # (This replaces the old "force ry negative" trick, which only worked
        # for the fixed [0, -pi, 0] target.)
        if np.dot(v, target_rotvec) < 0:
            v = -v
        diff_rotvec = target_rotvec - v
        if np.linalg.norm(diff_rotvec) < tol:
            print("Camera is face-down.")
            return True
        # Clamp the remaining rotation so each step moves at most step_in_radians
        # per component toward the target (and never overshoots).
        dr = np.clip(diff_rotvec, -step_in_radians, step_in_radians)
        print("Camera not yet face-down; taking a step toward it ..."   )
        if abs(dr[0])<step_in_radians and abs(dr[1])<step_in_radians and abs(dr[2])<step_in_radians:
            dr = np.array([0,0,0])
        pose.orient = m3d.Orientation(target_rotvec - dr)  # new orientation after the step
        rob.set_pose(pose, acc=0.1, vel=0.1, wait=True)
        # clean up any residual offset, to the teach path's stated accuracy
        rob.center_camera2apriltag(tol_m=AT_CENTER_TOL_M,
                                   max_iter=AT_CENTER_MAX_ITER)
    return _fail("Reached max_steps before the camera faced down.")

def search_apriltag_by_tilt(rob, ref_pos=[],
                            tilt_range=10, tilt_step=5, align_to_tag=False,
                            stop_event=None, skip_roll=False):
    """Search for an AprilTag by tilting the camera at a reference position.

    Sequence:
      1. Move the TCP to ``ref_pos`` (keeping the current orientation).
      2. Switch to the camera TCP (rob.camtcp) so tilts pivot about the camera.
      3. Tilt about the camera X and Y axes over +/- ``tilt_range`` degrees
         (in ``tilt_step`` steps, trying 0,0 first) until a tag is detected.
      4. Once found, tip the camera face-down while keeping the tag in view,
         then run center_camera2apriltag().

    align_to_tag is for a tag that does not lie flat on its station (the
    flowcell's, in the flowcell cleaning station). It changes what "aligned"
    means from world Z-down to the tag's own normal, so step 1 does not
    Zalign() -- levelling the tool first would only be squaring it to a
    surface the tag is not parallel to -- and step 4 squares the camera to
    the tag instead of tipping it face-down.

    skip_roll keeps the camera face-normal-down instead of running step 4's
    roll_around_tag: once the tag is found and centered, the tool is leveled in
    place with rob.Zalign() (which preserves the XY position and heading) and
    then re-centered on the tag. Use it to record a station's position with a
    clean level orientation -- for a tilted seat whose real angle is taught by
    hand afterward -- rather than tipping/squaring the camera to the tag. It
    overrides align_to_tag's squaring for the same reason.

    stop_event, if given, is a threading.Event checked before each move in
    every loop below; a caller that also calls rob.robot.stopj() to interrupt
    whatever move is in flight can rely on this to stop the sequence at the
    next opportunity rather than pressing on to the next step. Every abort or
    failure path returns through the same except block, so the TCP is always
    restored no matter which phase (tilt search, face-down roll, or descent)
    was interrupted.

    Returns True if a tag was found and centered, False otherwise (including
    on operator-requested abort).
    """
    global last_search_failure
    last_search_failure = None          # this run's reason, not the last run's
    #ref_pos=(-0.0, -0.4, 0.5)
    if len(ref_pos) ==0:
        ref_pos = [-0.22, -0.374598093, 0.200013817, -2.18860535, 2.25379435, -5.53757805e-05]
    if stop_event is not None and stop_event.is_set():
        return False
    try:
        # 1. Move to the reference position with the current (gripper) TCP.
        print(f"Moving to reference position {list(ref_pos)} ...")
        rob.set_tcp(rob.tcp)
        if not align_to_tag:
            rob.Zalign()  # keep the current orientation
        rob.moveto(list(ref_pos))
        rob.put_camera2tcp()  # ensure the camera is in the TCP frame
        # 2. Switch to the camera TCP so rotations pivot about the camera point.
        #rob.set_tcp(rob.camtcp)
        found = None
        base = rob.get_pose()               # camera-TCP pose at the reference
        base_pos = base.get_pos()
        angles = list(range(-tilt_range, tilt_range+1, tilt_step))
        # Order the grid by increasing tilt magnitude, so 0,0 is tried first.
        grid = sorted(((ax, ay) for ax in angles for ay in angles),
                      key=lambda a: a[0] ** 2 + a[1] ** 2)
        for (ax, ay) in grid:
            if stop_event is not None and stop_event.is_set():
                return _fail("AprilTag search stopped by operator.")
            t = base.copy()                 # fresh copy; leaves base untouched
            t.orient.rotate_xt(ax / 180 * math.pi)
            t.orient.rotate_yt(ay / 180 * math.pi)
            t.set_pos(base_pos)             # pivot in place about the camera
            rob.set_pose(t, acc=0.2, vel=0.3, wait=True)
            #time.sleep(5)                  # let the image settle after a move
            if _detect_apriltag(rob, stop_event=stop_event) is not None:
                print(f"AprilTag found at tilt (x={ax}, y={ay}) deg.")
                found = (ax, ay)
                break
        if found is None:
            return _fail("No AprilTag found within the tilt search range.")

        print("Moving up 1 cm ...")
        rob.mvr2z(0.01)
        print("Centering camera ...")
        rob.center_camera2apriltag(tol_m=AT_CENTER_TOL_M,
                                   max_iter=AT_CENTER_MAX_ITER)
        # 3./4. Bring the camera to its final orientation, keeping the tag in
        # view. Normally roll_around_tag tips it face-down (or squares it to a
        # tilted tag); skip_roll instead levels the tool in place so the camera
        # stays face-normal-down, leaving any real seat tilt to a hand teach.
        if skip_roll:
            print("Skipping roll: leveling camera to face straight down ...")
            rob.set_tcp(rob.tcp)   # level the gripper TCP, not the pivot TCP
            rob.Zalign()           # face down, keep XY position and heading
            rob.put_camera2tcp()
        else:
            if align_to_tag:
                print("Squaring the camera to the AprilTag's own normal ...")
            else:
                print("Tipping camera face-down while keeping the tag in view ...")
            if not roll_around_tag(rob, align_to_tag=align_to_tag, stop_event=stop_event):
                return False
        if stop_event is not None and stop_event.is_set():
            return False
        print("Finally centering the camera on the AprilTag ...")
        rob.center_camera2apriltag(tol_m=AT_CENTER_TOL_M,
                                   max_iter=AT_CENTER_MAX_ITER)
        if rob.camera.AT_euler is None:
            return _fail("Lost the AprilTag while centering; cannot refresh the camera pose.")
        rob.rotate_around_Zaxis_camera(180+rob.camera.AT_euler[2])  # refresh the camera pose
        rob.center_camera2apriltag(tol_m=AT_CENTER_TOL_M,
                                   max_iter=AT_CENTER_MAX_ITER)

        # Descend in 5 cm steps until the tag is ~0.2 m from the camera.
        r = _detect_apriltag(rob, stop_event=stop_event)
        while r is not None and rob.camera.QRdistance > 0.2:
            if stop_event is not None and stop_event.is_set():
                return _fail("AprilTag search stopped by operator.")
            rob.mvr2z(-0.05)
            r = _detect_apriltag(rob, stop_event=stop_event)
        # Re-align the camera's Z rotation to the tag after descending.
        if r is not None:
            rob.rotate_around_Zaxis_camera(180+rob.camera.AT_euler[2])

        return True
    except Exception as ex:
        rob.set_tcp(rob.tcp)  # restore the original TCP
        if stop_event is not None and stop_event.is_set():
            print("AprilTag search stopped by operator.")
        else:
            return _fail(f"search_apriltag_by_tilt failed: {ex}")

def bring_hand_to_camera_center(rob, box, center, acc=0.1, vel=0.1):
    # distance vs pixel size
    # pixel distance = 1/d (in meter)*100
    #print(box[2], box[3])
    d = 190.0/box[3]  # when hand is open and fingers are up.
    dH = center[0]/box[2]*0.060 # size of fist is 60mm
    dV = center[1]/box[2]*0.060 # size of fist is 60mm
    #print("distance is {}m".format(d))
    #print("delta H and delta V are {}".format([dH, dV]))
    distance0 =1
    distance = d-distance0
    V = dV
    H = dH

    try:
        rob.move_toward_camera(distance=distance, north=-V, east=H, acc=0.5, vel=0.6)
    except:
        pass


def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def rotation_axis_angle_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    #v = np.linalg.norm(v)
    ang = np.arccos(c) # angle [radian] between two vectors
    return v, ang