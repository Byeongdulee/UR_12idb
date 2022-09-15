#from cProfile import run
import math
#from xmlrpc.server import _DispatchArity0
import math3d as m3d
import time
import numpy as np
#from imutils import paths
import cv2
from urcamera import decodeQR, showQRcode
from threading import Thread

pos_sam = [-4.60838969e-01, -5.05650395e-01,  2.31693123e-01,  2.28368253e+00,
       -2.15707100e+00,  1.01565770e-03]
pos_mag = [-3.89803673e-01, -1.50716439e-01, -3.46071435e-02,  2.26448275e+00,
       -2.17747082e+00,  1.91570580e-03]

def test(rob):
    rob.release()
    rob.move2z(0.1)
    rob.moveto(pos_sam)
    rob.move2z(-0.1)
    rob.grab()
    fp = rob.finger.get_position()
    rob.move2z(0.1)
    rob.moveto(pos_mag, vel=0.1)
    if fp>1: # there is a sample 
        rob.dropsample()
    else:
        rob.release()
        rob.move2z(-0.1)
        rob.grab()
        rob.move2z(0.1)
        rob.moveto(pos_sam)
        rob.dropsample()
    rob.move2z(0.1)

def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def run_measuredistance(rob):
    rob.grab()
    rob.put_tcp2camera()
    if (rob.camera.QRtiltangle>0):
        rob.rotz(90)
    else:
        rob.rotz(-90)
    print(rob.robot.bump(z=-1, backoff=0.1))

def run_centering(rob):
    rob.camera._running = True
    rob.bring_QR_to_camera_center()
    rob.camera._running = False

def run_centering1(rob):
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
       
def measureheight(rob):
    v0 = rob.get_xyz()
    rob.robot.bump(z=-1, backoff=0.01)
    v1 = rob.get_xyz()
    v = v0-v1
    rob.move2z(v[2])
    #v1 = rob.get_xyz()
    #v = v0-v1
    #print(v0, v1)
    distance = v[2]+0.01
    return distance

def align_heater(rob):
    rob.finger.gripper_activate()
    rob.set_orientation()
    rob.tilt_x()
    rob.bring_QR_to_camera_center()
    if len(rob.camera.QRposition) ==0:
        print("No QR code is found. Need to adjust camera position before running this.")
        rob.tilt_xm()
        return
    rob.tilt_xm()
    rob.rotz(rob.camera.QRtiltangle)
    rob.grab()
    #pickupshift(rob)
    height = measureheight(rob)
    print(f"Height was {height}")
    dist = height+0.02
    print(f"move z down by {dist}")
    rob.release()
    rob.move2z(-dist, acc=0.05, vel = 0.05)
    rob.grab()
    rob.move2z(dist)
    print(f"Moved up by {dist}, and the program is done.")

def pickupshift(rob):
    #rob.move2x(0.0005)
    rob.move2y(-0.001)

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


def showcamera(rob):
#    t = Thread(target=rob.camera.run, daemon=True)
#    t.start()
#    time.sleep(0.1)
    rob.camera.QRdistance = ""
    flipflop = True
    while 1:    
        # Capture
        ret, frame = rob.camera.capture()
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
        QRcode = decodeQR(frame)
        if len(QRcode) ==1:
            for qrd in QRcode:
                if qrd.data == b'sav':
                    #print(qrd)
                    isambient = True
        try:
            showQRcode(QRcode, frame)
            cv2.putText(frame, "{}: {:.2f}mm".format("distance", rob.camera.QRdistance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        except:
            pass

        cv2.imshow('frame', frame)
        rob.camera.decode2QR()
        key = cv2.waitKey(20) & 0xFF
        if key == 48:  #0
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,400) #This sets the focus to a value of i
        if key == 49: #1
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,450) #This sets the focus to a value of i
        if key == 50:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,475) #This sets the focus to a value of i
        if key == 51:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,500) #This sets the focus to a value of i
        if key == 52:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,525) #This sets the focus to a value of i
        if key == 53:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,550) #This sets the focus to a value of i
        if key == 54:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,600) #This sets the focus to a value of i
        if key == 55:
            rob.camera.vidcap.set(cv2.CAP_PROP_FOCUS,650) #This sets the focus to a value of i
        if key == 105: #i
            rob.move_toward_camera(0, north=0.025)
        if key == 106: #j
            rob.move_toward_camera(0, north=0, east=-0.025)
        if key == 107: #k
            rob.move_toward_camera(0.025, north=0, east=0.0)
        if key == 108: #l
            rob.move_toward_camera(0, north=0, east=0.025)
        if key == 109: #m
            rob.move_toward_camera(0, north=-0.025, east=0.0)
        if key == 111: #o
            rob.move_toward_camera(-0.025, north=0, east=0.0)
        if key == 97: #a
            rob.camera.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,1)
        if key == 120: #x
            rob.camera.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,0)
        if key == 115: #s
            rob.camera.scanfocus()
        if key == 104: #h
            print("Help:")
            print("  focal point change: 1, 2, 3, and 4")
            print("  focal point change: a (auto), x(manual), s(scan)")
            print("  move robot: i(north),j(west),k(toward),l(east),m(south),o (away)")
            print("  camera tilt down: t")
            print("  center QR: c")
            print("  Measure distance : M")
            print("  exit: ESC")
        if key == 113: #q
            print(QRcode)
        if key == 116: #t
            rob.tilt_camera_down()
        if key == 77: #M
            t = Thread(target=run_measuredistance, args=(rob,))
            t.start()
        if key == 99: #c
            #rob.bring_QR_to_camera_center()
            if not isambient:
                t = Thread(target=run_centering, args=(rob,))
            else:
                t = Thread(target=run_centering1, args=(rob,))
            t.start()
            #t.join()
        if key == 27:
            break
        foc = rob.camera.get_foc()
    #    print("The current focus value is: {}".format(foc))
        time.sleep(0.1)
    cv2.destroyAllWindows()
#    rob.camera.stop()
    #t.join()

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