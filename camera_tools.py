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
from common.urcamera import camera
from threading import Thread
from pupil_apriltags import Detector
import json
import os

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


def average_apriltag_pose(cam, at, camera_params, min_margin, N=10):
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
        dets = at.detect(g2, estimate_tag_pose=True,
                         camera_params=camera_params,
                         tag_size=cam.AT_physical_size)
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
    at = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
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
        isambient = False
        if codetype==1:
            QRcode = decodeQR(frame)
            rob.camera.image = frame
            data, rectcoord, qrsize, dist = rob.camera.decode()
            if len(data) ==1:
                if data == b'sav':
                    #print(qrd)
                    isambient = True
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
        r = at.detect(gray, estimate_tag_pose=True,
                      camera_params=[fx, fy, cx, cy],
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
                rob.camera, at, [fx, fy, cx, cy], AT_MIN_MARGIN, N=N)
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

def resolve_robot_ip(name='UR5'):
    """Look up a robot's control-box IP from list_of_robots.json by name."""
    here = os.path.dirname(os.path.abspath(__file__))
    for fn in (os.path.join('RobotList', 'list_of_robots.json'),
               os.path.join(here, 'RobotList', 'list_of_robots.json'),
               os.path.join(here, 'list_of_robots.json'),
               'list_of_robots.json'):
        if os.path.exists(fn):
            with open(fn) as f:
                return json.load(f)[name]
    raise FileNotFoundError("list_of_robots.json not found.")

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
    at = Detector(families='tag36h11',
                  nthreads=1,
                  quad_decimate=1.0,
                  quad_sigma=0.0,
                  refine_edges=1,
                  decode_sharpening=0.25,
                  debug=0)
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
        r = at.detect(gray, estimate_tag_pose=True,
                      camera_params=[fx, fy, cx, cy],
                      tag_size=cam.AT_physical_size)
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
                cam, at, [fx, fy, cx, cy], AT_MIN_MARGIN, N=N)
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

def _detect_apriltag(rob, settle=5, tag_id=None):
    """Capture one frame and return the AprilTag detection (or None).

    tag_id selects a specific tag number; when omitted and several tags are
    in view, the one nearest the image center is used."""
    t0 = time.time()
    r = None
    while True:
        # When a live display loop (showcamera) is already capturing, reuse its
        # latest frame instead of grabbing our own.
        if not rob.camera._running:
            rob.camera.capture()
        r = rob.camera.decodeAT(tag_id=tag_id)      # populates rob.camera.decoded
        if r is not None:
            break
        time.sleep(0.1)  # Wait a bit before trying again
        if time.time() - t0 > settle:
            print("No AprilTag detected after waiting {:.1f} s.".format(settle))
            return None
    return r

def roll_around_tag(rob, step=5, tol=1e-2, max_steps=24):
    """Tilt the camera toward face-down in steps of at most ``step`` degrees,
    pivoting about the tag so it stays centered, until the camera faces down
    (within ``tol`` radians) or the tag is lost / ``max_steps`` is reached."""
    r = _detect_apriltag(rob)
    if r is None:
        print("No AprilTag in view; cannot roll around the tag.")
        return False
    distance = rob.camera.getATdistance(r)
    newtcp = list(rob.camtcp)
    newtcp[2] = distance
    rob.set_tcp(newtcp)
    step_in_radians = step / 180 * math.pi
    target_rotvec = np.array([0, -math.pi, 0])  # face-down rotation vector
    for _ in range(max_steps):
        pose = rob.get_pose()
        v = pose.orient.get_rotation_vector().array
        v[1] = -1*abs(v[1])  # ensure the camera is facing down (negative Y)
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
        rob.center_camera2apriltag()  # clean up any residual offset
    print("Reached max_steps before the camera faced down.")
    return False

def search_apriltag_by_tilt(rob, ref_pos=(-0.35, -0.18, 0.5),
                            tilt_range=10, tilt_step=5):
    """Search for an AprilTag by tilting the camera at a reference position.

    Sequence:
      1. Move the TCP to ``ref_pos`` (keeping the current orientation).
      2. Switch to the camera TCP (rob.camtcp) so tilts pivot about the camera.
      3. Tilt about the camera X and Y axes over +/- ``tilt_range`` degrees
         (in ``tilt_step`` steps, trying 0,0 first) until a tag is detected.
      4. Once found, tip the camera face-down while keeping the tag in view,
         then run center_camera2apriltag().

    Returns True if a tag was found and centered, False otherwise.
    """
    #ref_pos=(-0.0, -0.4, 0.5)
    # 1. Move to the reference position with the current (gripper) TCP.
    print(f"Moving to reference position {list(ref_pos)} ...")
    rob.set_tcp(rob.tcp)
    rob.set_orientation()  # keep the current orientation
    rob.moveto(list(ref_pos))
    rob.put_camera2tcp()  # ensure the camera is in the TCP frame
    # 2. Switch to the camera TCP so rotations pivot about the camera point.
    #rob.set_tcp(rob.camtcp)
    found = None
    try:
        base = rob.get_pose()               # camera-TCP pose at the reference
        base_pos = base.get_pos()
        angles = list(range(-tilt_range, tilt_range+1, tilt_step))
        # Order the grid by increasing tilt magnitude, so 0,0 is tried first.
        grid = sorted(((ax, ay) for ax in angles for ay in angles),
                      key=lambda a: a[0] ** 2 + a[1] ** 2)
        for (ax, ay) in grid:
            t = base.copy()                 # fresh copy; leaves base untouched
            t.orient.rotate_xt(ax / 180 * math.pi)
            t.orient.rotate_yt(ay / 180 * math.pi)
            t.set_pos(base_pos)             # pivot in place about the camera
            rob.set_pose(t, acc=0.2, vel=0.3, wait=True)
            #time.sleep(5)                  # let the image settle after a move
            if _detect_apriltag(rob) is not None:
                print(f"AprilTag found at tilt (x={ax}, y={ay}) deg.")
                found = (ax, ay)
                break
    except Exception as ex:
        rob.set_tcp(rob.tcp)  # restore the original TCP
        print(f"search_apriltag_by_tilt failed: {ex}")
    if found is None:
        print("No AprilTag found within the tilt search range.")
        return False
    rob.center_camera2apriltag()
    # 3./4. Face the camera down keeping the tag in view, then fine-center.
    print("Tipping camera face-down while keeping the tag in view ...")
    roll_around_tag(rob)
    print("Finally centering the camera on the AprilTag ...")
    rob.center_camera2apriltag()
    if rob.camera.AT_euler is None:
        print("Lost the AprilTag while centering; cannot refresh the camera pose.")
        return False
    rob.rotate_around_Zaxis_camera(rob.camera.AT_euler[2])  # refresh the camera pose
    rob.center_camera2apriltag()

    # Descend in 5 cm steps until the tag is ~0.2 m from the camera.
    r = _detect_apriltag(rob)
    while r is not None and rob.camera.QRdistance > 0.2:
        rob.mvr2z(-0.05)
        r = _detect_apriltag(rob)
    # Re-align the camera's Z rotation to the tag after descending.
    if r is not None:
        rob.rotate_around_Zaxis_camera(rob.camera.AT_euler[2])

    return True

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