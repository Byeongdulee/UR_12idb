import numpy as np
import requests
from PIL import Image
from collections import namedtuple
import atexit
import io
import math
import time
import cv2
from common import m3d  # centralized math3d (4.x compat applied in common.m3d)
import threading
from scipy.spatial.transform import Rotation

ISQR = True
ISAPRILTAGS = True

class NoUSBCameraException(Exception):
    pass

# zxing-cpp replaces pyzbar, which was last released in 2022 and needs the
# native zbar DLL. zxing-cpp ships binary wheels, decodes rotated, tilted,
# blurred, low-contrast and inverted codes that OpenCV's own detector misses,
# and reads Micro QR -- useful at the 23 mm holder size (see QRsavSize).
try:
    import zxingcpp
except ImportError:
    ISQR = False

try:
    from pupil_apriltags import Detector
    from pupil_apriltags.bindings import Detection
except ImportError:
    ISAPRILTAGS = False
import os
img_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'images')

default_imgH = 1280
default_imgV = 720
focus_threshold = 510
QRsavSize = 0.023 # 1QR 'sav' size = ~23mm (about 1mm error)
#AT_size = 0.021
AT_size = 0.010
# dx = dX/Z*f
# dx: change in pixels on camera.
# dX: change of the position of an object (or the robot arm carrying the camera) (m)
# Z : the distance from camera to the object (m)
# f : pixel/m ratio
# f ~ 1620 # This is calculated from for imgH = 1280 and imgV = 720
camera_f = 1620

def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

QRpos = [[[0, 0, 0], 
    [0, 0.015/2, 0],
    [0, 0.015, 0], 
    [0.015/2, 0.015/2, 0],
    [0.015, 0.015, 0],
    [0.015/2, 0, 0],
    [0.015, 0, 0]]] #position of QR codes

def isblurry(image):
    size = image.shape
    # 9:16 size
    # 72:128
    ysize = 72
    xsize = 128
    y0 = int(size[0]/2-ysize)
    y1 = int(size[0]/2+ysize-1)
    x0 = int(size[1]/2-xsize)
    x1 = int(size[1]/2+xsize-1)
    gray = cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    val = False
    if fm<focus_threshold:
        val = True
    return val

def cal_AT2pose(r):
    if not isinstance(r, Detection):
        print("The input is not aprilTag object.")
        return []
    # aprilTag's rotation matrix is to convert the world coordinate to camera coordinate.
    #r.pose_R[:,[0,2]]=r.pose_R[:,[2,0]]
    p = m3d.Transform()
    r2 = Rotation.from_matrix(r.pose_R)
    euler = r2.as_euler('xyz', degrees = True)
    R = Rotation.from_euler('zyx', [euler[2], -euler[1], -euler[0]], degrees=True)
    #print(f"Euler angles : {euler} degrees")
    p.orient = m3d.Orientation(R.as_matrix())
    #p.pos =np.array([r.pose_t[0][0],r.pose_t[1][0],0])
    return euler, r.pose_t, p
    #return euler, r.pose_t

__author__ = "Byeongdu Lee, <blee@anl.gov>, Argonne National Laboratory"
__license__ = "LGPLv3"

# camera foc vs real distance from the camera
# 455, 0.69
# 488, 0.59
# 477, 0.49
# 488, 0.39
# 510, 0.29
# 543, 0.19
# 587, 0.14

# One AprilTag detector for the whole process, and one lock around every
# detection. pupil_apriltags' Detector is a thin ctypes wrapper on libapriltag,
# and two things made the old "build a Detector per call" code crash the
# interpreter with a segfault:
#   * Detector.__del__ destroys the tag family first and then calls
#     apriltag_detector_destroy(), which walks the families still registered
#     with the detector -- it reads the block it just freed. One decode per
#     frame meant one such destructor per frame, corrupting the heap until an
#     unrelated allocation fell over.
#   * showcamera() detects in its display loop while a robot action dispatched
#     to a worker thread detects at the same time. libapriltag is not safe to
#     drive concurrently, and ctypes drops the GIL for the duration of the call.
# Keeping a single detector alive for the life of the process removes the
# per-frame destructor, and _AT_LOCK serializes the two threads.
_AT_DETECTOR = None
_AT_LOCK = threading.RLock()

def _disarm_AT_detector():
    # The one remaining destructor is the shared detector's own, at interpreter
    # shutdown -- which turns a clean exit into a core dump for the same reason.
    # Detector.__del__ does nothing when tag_detector_ptr is None, so clear it
    # and let the OS reclaim the C detector as the process dies.
    global _AT_DETECTOR
    at = _AT_DETECTOR
    if at is not None and hasattr(at, 'tag_detector_ptr'):
        at.tag_detector_ptr = None
    _AT_DETECTOR = None

def get_AT_detector():
    """Return the process-wide AprilTag detector, building it on first use.

    Returns None when pupil_apriltags is not installed."""
    global _AT_DETECTOR
    if not ISAPRILTAGS:
        return None
    with _AT_LOCK:
        if _AT_DETECTOR is None:
            at = Detector(families='tag36h11',
                          nthreads=1,
                          quad_decimate=1.0,
                          quad_sigma=0.0,
                          refine_edges=1,
                          decode_sharpening=0.25,
                          debug=0)
            atexit.register(_disarm_AT_detector)
            _AT_DETECTOR = at
        return _AT_DETECTOR

def detect_AT(gray, camera_params, tag_size=AT_size):
    """Detect AprilTags in a grayscale frame using the shared detector.

    Every AprilTag detection in this package must go through here (or through
    decodeAT below), so that detections from the display loop and from threaded
    robot actions never run at the same time."""
    with _AT_LOCK:
        at = get_AT_detector()
        if at is None:
            return []
        return at.detect(gray, estimate_tag_pose=True,
                         camera_params=camera_params, tag_size=tag_size)

def decodeAT(img=[], F=[], cam_f=camera_f, imgH=default_imgH, imgV=default_imgV):
    if isinstance(img, type(None)):
        return []
    if len(img)==0:
        print("empty image")
        return

    if len(F)==0:
        F = [[cam_f, 0, imgH/2], [0, cam_f, imgV/2], [0, 0, 1]]
    fx = F[0][0]
    fy = F[1][1]
    cx = F[0][2]
    cy = F[1][2]
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    r = detect_AT(gray, [fx, fy, cx, cy], tag_size=AT_size)
    return r

def selectAT(detections, tag_id=None, center=None):
    # Pick a single tag out of a list of detections. All tags of the tag36h11
    # family look alike to the detector; they are told apart by .tag_id.
    #   tag_id : return only this tag number, or None when it is not in view.
    #   center : (x, y) of the image center. When no tag_id is asked for and
    #            several tags are visible, the one nearest this point wins.
    # Returns the chosen Detection, or None.
    if detections is None or len(detections) == 0:
        return None
    if tag_id is not None:
        for d in detections:
            if d.tag_id == tag_id:
                return d
        return None
    if len(detections) == 1:
        return detections[0]
    if center is None:
        return detections[0]
    cx, cy = center
    return min(detections,
               key=lambda d: (d.center[0]-cx)**2 + (d.center[1]-cy)**2)

# Shapes matching what pyzbar returned, so every caller below -- and in
# camera_tools and robot12idb -- is unaffected by the switch. .data stays
# bytes because callers compare against literals such as b'stv1' and b'sav'.
QRPoint = namedtuple("QRPoint", "x y")
QRRect = namedtuple("QRRect", "left top width height")
QRDecoded = namedtuple("QRDecoded", "data type rect polygon")

# Micro and rMQR are included because they fit more data in the same physical
# mark; OpenCV's detector cannot read either.
_QR_FORMATS = (
    zxingcpp.BarcodeFormat.QRCode,
    zxingcpp.BarcodeFormat.MicroQRCode,
    zxingcpp.BarcodeFormat.RMQRCode,
) if ISQR else ()


def decodeQR(img):
    """Decode every QR code in `img`, a BGR or grayscale OpenCV array.

    Returns a list of QRDecoded, shaped like pyzbar's decode() output:

      .data     payload as bytes
      .type     human-readable symbology, e.g. "QR Code"
      .rect     axis-aligned bounding box (left, top, width, height)
      .polygon  the four corners, clockwise from top-left

    The polygon order matters: analyzeroll_QR and analyzetilt_QR treat
    consecutive points as adjacent corners, and decodeQR below averages the
    four edge lengths to estimate distance.
    """
    if not ISQR:
        return []
    results = []
    for barcode in zxingcpp.read_barcodes(img, formats=_QR_FORMATS):
        pos = barcode.position
        polygon = [QRPoint(int(corner.x), int(corner.y)) for corner in
                   (pos.top_left, pos.top_right, pos.bottom_right, pos.bottom_left)]
        xs = [point.x for point in polygon]
        ys = [point.y for point in polygon]
        rect = QRRect(min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
        results.append(QRDecoded(data=barcode.bytes, type=str(barcode.format),
                                 rect=rect, polygon=polygon))
    return results

def showQRcode(QRdata, image):
    for barcode in QRdata:
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 5)

class camera(object):
    def __init__(self,
                 IP="", device=0, apriltagsize = AT_size):
        self.IP = IP
        self.device = device
        self.camera_f = camera_f
        self.imgH = default_imgH
        self.imgV = default_imgV
        self.QR_physical_size = QRsavSize
        self.AT_physical_size = apriltagsize
        self.intrinsic_mtx = []
        self.image = None
        # --- AprilTag state -------------------------------------------------
        # AT_detections/AT_ids hold every tag seen in the last decoded frame;
        # decoded/AT_id/AT_euler/... describe the single selected tag.
        self.AT_detections = []
        self.AT_ids = []
        self.decoded = None
        self.AT_id = None       # tag number within the family, e.g. 1, 2, ...
        self.AT_euler = None
        self.AT_translation = None
        self.AT_pose = None
        self._running = False
        if len(self.IP) == 0:
            vidcap = cv2.VideoCapture(self.device)
            if not vidcap.isOpened():
                print("Cannot open camera {}".format(self.device))
                exit()
            vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, default_imgH)
            vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, default_imgV)
            self.vidcap = vidcap
            self.focus(520)
            self.connectiontype = 'usb'
        else:
            self.connectiontype = 'ip'
    
    def isblurry(self):
        if self.image is not None:
            return isblurry(self.image)
        else:
            print("No captured image.")
    
    def scanfocus(self):
        if len(self.IP)>0:
            raise NoUSBCameraException
        
        for i in range(350,600,5):
            ret, img = self.capture()
            if not isblurry(img):
                break
            self.focus(i)
            time.sleep(0.1)
        foc = self.vidcap.get(cv2.CAP_PROP_FOCUS)
        return foc

    def focus(self, val):
        if len(self.IP)>0:
            raise NoUSBCameraException
        #  #Not sure what max and min range are for the camera focus,
        # I found that a value of 450 works well at a 1 ft range under current conditions
        if len(self.IP) == 0:
            self.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,0)
            self.vidcap.set(cv2.CAP_PROP_FOCUS, val)

    def autofocus(self):
        if len(self.IP)>0:
            raise NoUSBCameraException
        if len(self.IP) == 0:
            self.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,1)
            foc = self.vidcap.get(cv2.CAP_PROP_FOCUS)
            return foc

    def get_foc(self):
        if len(self.IP)>0:
            raise NoUSBCameraException
        foc = self.vidcap.get(cv2.CAP_PROP_FOCUS)
        return foc

    def capture(self):
        resp=None
        if len(self.IP)>0:
            #Try to get camera image with provided robot IP
            try:
                resp = requests.get("http://"+self.IP+":4242/current.jpg?type=color").content
            except:
                pass
            #Check the response
            if resp == None:
                #If the response is empty display an error image
                pilImage=None
                ret = False
            else:
                #If the response is not empty format the data to get a
                #tkinter format image
                imageData = np.asarray(bytearray(resp), dtype="uint8")
                pilImage=Image.open(io.BytesIO(imageData))
                pilImage = np.asarray(pilImage)
                ret = True
        else:
            ret, pilImage = self.vidcap.read()
            if not ret:
                print("Fail to capture camera.")
        # Only publish a valid frame. Never blank self.image mid-capture, so a
        # concurrent reader (e.g. a threaded robot move during showcamera) never
        # catches self.image as None. On failure, keep the previous frame.
        if ret and pilImage is not None:
            self.image = pilImage
            self.imgH = pilImage.shape[1]
            self.imgV = pilImage.shape[0]
            self.camera_f = camera_f/default_imgH*self.imgH
        return ret, pilImage

    def _clearAT(self):
        # Forget the previously selected tag. Without this the AT_* attributes
        # keep the pose of an older detection, which a caller that does not
        # check the return value would happily use as if it were fresh.
        self.decoded = None
        self.AT_id = None
        self.AT_euler = None
        self.AT_translation = None
        self.AT_pose = None

    def decodeAT(self, tag_id=None):
        # Decode the AprilTags in the current image and keep one of them.
        #   tag_id : select this tag number; returns None when it is not in
        #            view. When omitted and several tags are visible, the tag
        #            nearest the image center is selected.
        # Every tag seen is kept in self.AT_detections / self.AT_ids; the
        # selected one is returned and described by self.decoded, self.AT_id,
        # self.AT_euler, self.AT_translation and self.AT_pose.
        r = decodeAT(self.image, self.intrinsic_mtx, self.camera_f, self.imgH, self.imgV)
        if isinstance(r, type(None)):
            self.AT_detections = []
            self.AT_ids = []
            self._clearAT()
            return None
        self.referenceName = "AT"
        self.AT_detections = list(r)
        self.AT_ids = [d.tag_id for d in r]
        # Measure "nearest the center" against the frame actually captured.
        if self.image is not None:
            h, w = self.image.shape[:2]
        else:
            w, h = self.imgH, self.imgV
        r = selectAT(self.AT_detections, tag_id=tag_id, center=(w/2, h/2))
        if r is None:
            self._clearAT()
            return None
        self.getATdistance(r)
        self.decoded = r
        self.AT_id = r.tag_id
        euler, t, pose = cal_AT2pose(r)
        self.AT_euler = euler
        self.AT_translation = t
        self.AT_pose = pose
        return r

    def getATdistance(self, r):
        pgpnts = r.corners
        dist = []
        for k in range(4):
            ind1 = k%4
            ind2 = (k+1)%4
            x0 = pgpnts[ind1][0]
            y0 = pgpnts[ind1][1]
            x1 = pgpnts[ind2][0]
            y1 = pgpnts[ind2][1]
            d = math.sqrt((x0-x1)**2+(y0-y1)**2)
            dist.append(d)
        self.QRdistance = self.AT_physical_size/np.mean(dist)*self.camera_f
        self.QRposition = r.center
        self.QRsize = [math.dist(pgpnts[0], pgpnts[1]), 
                       math.dist(pgpnts[1], pgpnts[2])]
        return self.QRdistance

    def H2RT(self, H):
        # https://medium.com/analytics-vidhya/using-homography-for-pose-estimation-in-opencv-a7215f260fdd
        if len(self.intrinsic_mtx)==0:
            self.intrinsic_mtx = [[self.camera_f, 0, self.imgH/2], [0, self.camera_f, self.imgV/2], [0, 0, 1]]
        K = self.intrinsic_mtx
        H = H.T
        h1 = H[0]
        h2 = H[1]
        h3 = H[2]
        K_inv = np.linalg.inv(K)
        L = 1 / np.linalg.norm(np.dot(K_inv, h1))
        r1 = L * np.dot(K_inv, h1)
        r2 = L * np.dot(K_inv, h2)
        r3 = np.cross(r1, r2)
        T = L * (K_inv @ h3.reshape(3, 1))
        R = np.array([[r1], [r2], [r3]])
        R = np.reshape(R, (3, 3))
        return R, T

    def decode(self, p0in=(0,0), p1in=(0,0), imgwidth=default_imgH, imgheight=default_imgV, color = (0, 0, 255), thickness = 1):
        rectcoord = []
        data = []
        dist = []
        pgpnts = []
        qrsize = [0, 0]
        if isinstance(self.image, type(None)):
            return data, rectcoord, qrsize, dist
        opencvimage = np.array(self.image)
        imgdata = opencvimage[:,:,::-1].copy()
        QRdata = decodeQR(imgdata)
        if len(QRdata)>0:
            data, rectcoord, qrsize, dist = self.decodeQR()
            return data, rectcoord, qrsize, dist
        r = self.decodeAT()
        if r is None:
            self.QRdata = data
            self.QRposition = rectcoord
            self.QRsize = qrsize            
            self.QRedgelength = dist
            self.QRcoordinates = pgpnts
            return data, rectcoord, qrsize, dist

    def decodeQR(self, p0in=(0,0), p1in=(0,0), imgwidth=default_imgH, imgheight=default_imgV, color = (0, 0, 255), thickness = 1):
        #imgdata = self.image
        opencvimage = np.array(self.image)
        imgdata = opencvimage[:,:,::-1].copy()
        QRdata = decodeQR(imgdata)
        centx = []
        centy = []
        centindx = []

        rectcoord = []
        data = []
        dist = []
        pgpnts = []

        qrsize = [0, 0]
        # no decoded results
        if len(QRdata)==0:
            self.QRtype = "2QR"
            self.QRdata = data
            self.QRposition = rectcoord
            self.QRsize = qrsize            
            self.QRedgelength = dist
            self.QRcoordinates = pgpnts
            return data, rectcoord, qrsize, dist

        showQRcode(QRdata, imgdata)
        # single QR
        if len(QRdata)==1:
            height, width, channels = imgdata.shape
            if height < imgheight/2:
                return (False, -1)

            p0 = (int(p0in[0]/imgwidth*width), int(p0in[1]/imgheight*height))
            p1 = (int(p1in[0]/imgwidth*width), int(p1in[1]/imgheight*height))
            cv2.line(imgdata, p0, p1, color, thickness) 
            for qrd in QRdata:
                rectcoord = [qrd.rect.left+qrd.rect.width/2,  qrd.rect.top+qrd.rect.height/2]
                data = qrd.data
                qrsize[0] = qrd.rect.width
                qrsize[1] = qrd.rect.height

                pg = qrd.polygon
                pgpnts = []
                for pnts in pg:
                    pgpnts.append([pnts.x, pnts.y])
                for k in range(4):
                    ind1 = k%4
                    ind2 = (k+1)%4
                    x0 = pgpnts[ind1][0]
                    y0 = pgpnts[ind1][1]
                    x1 = pgpnts[ind2][0]
                    y1 = pgpnts[ind2][1]
                    d = math.sqrt((x0-x1)**2+(y0-y1)**2)
                    dist.append(d)
                break
                #print(f"Edge lengthes of QR are {dist}")
            self.QRtype = "1QR"
            self.QRdistance = self.QR_physical_size/np.mean(dist)*self.camera_f
            self.image = Image.fromarray(imgdata)
            self.QRdata = data
            self.QRposition = rectcoord
            self.QRsize = qrsize
            self.QRedgelength = dist
            self.QRcoordinates = pgpnts
            return data, rectcoord, qrsize, dist

        # double QR.

        height, width, channels = imgdata.shape
        if height < imgheight/2:
            return (False, -1)

        p0 = (int(p0in[0]/imgwidth*width), int(p0in[1]/imgheight*height))
        p1 = (int(p1in[0]/imgwidth*width), int(p1in[1]/imgheight*height))
        cv2.line(imgdata, p0, p1, color, thickness) 
#        print(QRdata)
        for qrd in QRdata:
            rectcoord = [qrd.rect.left+qrd.rect.width/2,  qrd.rect.top+qrd.rect.height/2]
            data = qrd.data
            qrsize[0] = qrd.rect.width
            qrsize[1] = qrd.rect.height
#            print(data)
            pg = qrd.polygon
            pgpnts = []
            for pnts in pg:
                pgpnts.append([pnts.x, pnts.y])
            for k in range(4):
                ind1 = k%4
                ind2 = (k+1)%4
                x0 = pgpnts[ind1][0]
                y0 = pgpnts[ind1][1]
                x1 = pgpnts[ind2][0]
                y1 = pgpnts[ind2][1]
                d = math.sqrt((x0-x1)**2+(y0-y1)**2)
                dist.append(d)
            centx.append(rectcoord[0])
            centy.append(rectcoord[1])
            centindx.append(data)
        if len(centx) == 2:
            centx0 = (centx[0]+centx[1])/2
            centy0 = (centy[0]+centy[1])/2
            qrpos = [centx0, centy0]
#            print(f"two QRs center: X = {centx0}, center Y = {centy0}")
#            print(f"Image size = {imgdata.shape}")
            imgsize = imgdata.shape
#            print(f"two QRs imgcenter - center: dX = {imgsize[1]/2-centx0}, dY = {imgsize[0]/2-centy0}")
            try:
                ix1 = centindx.index(b'stv1')
                ix0 = centindx.index(b'stv0')
            except:
                ix1 = 1
                ix0 = 0
            ang = math.atan2(-(centy[ix1]-centy[ix0]), centx[ix1]-centx[ix0]) # In an image, y axis value decreases with up.
            if (centx[ix1]-centx[ix0]) > 0:
                ang = math.pi+ang
#            print(f"two QRs: Tilt angle: {ang/math.pi*180} degree")
        else:
            qrsize = 0
            qrpos = []
        self.QRtype = "2QR"
        self.image = Image.fromarray(imgdata)
        self.QRdata = centindx
        self.QRposition = qrpos
        self.QRsize = qrsize
        self.QRedgelength = dist
        self.QRtiltangle = ang/math.pi*180
        return data, [centx0, centy0], qrsize, dist

    def analyzeroll_QR(self):
        # return roll angle (in-plane orientation angle) in degree.
        if len(self.QRcoordinates) < 4:
            return 0
        x0 = self.QRcoordinates[0][0]
        x1 = self.QRcoordinates[0][1]
        y0 = self.QRcoordinates[1][0]
        y1 = self.QRcoordinates[1][1]
        ang = math.atan2(-(y1-y0), x1-x0) # In an image, y axis value decreases with up.
        print(f"Tilt angle: {ang/math.pi*180} degree")
        return ang/math.pi*180

    def analyzetilt_QR(self):
        # analysize tilt angle from a QR code.
        # it returns direction to rotate with .tilt_over function.
        if len(self.QRcoordinates) < 4:
            return [0, 0]
        cntpos = []
        for k in range(4):
            ind1 = k%4
            ind2 = (k+1)%4
            x0 = self.QRcoordinates[ind1][0]
            y0 = self.QRcoordinates[ind1][1]
            x1 = self.QRcoordinates[ind2][0]
            y1 = self.QRcoordinates[ind2][1]
            pos = [(x0+x1)/2, (y0+y1)/2]
            cntpos.append(pos)
        xmaxind = 0
        xminind = 0
        ymaxind = 0
        yminind = 0
        for k in range(4):
            x = cntpos[k][0]
            y = cntpos[k][1]
            if x>cntpos[xmaxind][0]:
                xmaxind = k
            if x<cntpos[xminind][0]:
                xminind = k
            if y>cntpos[ymaxind][1]:
                ymaxind = k
            if y<cntpos[yminind][1]:
                yminind = k
#        print(self.QRedgelength[xmaxind], 'xmax')
#        print(self.QRedgelength[xminind], 'xmin')
#        print(self.QRedgelength[ymaxind], 'ymax')
#        print(self.QRedgelength[yminind], 'ymin')
        dir = [0, 0]
        if (self.QRedgelength[xmaxind] - self.QRedgelength[xminind])/self.QRedgelength[xmaxind]>0.015:
            print("Direction [0, 1] and negative angle")
            dirv = [0, -1]
            for i in range(2):
                dir[i] = dir[i]+dirv[i]
        if (self.QRedgelength[xminind]-self.QRedgelength[xmaxind])/self.QRedgelength[xminind]>0.015:
            print("Direction [0, 1] and positive angle")
            dirv = [0, 1]
            for i in range(2):
                dir[i] = dir[i]+dirv[i]
        if (self.QRedgelength[ymaxind] - self.QRedgelength[yminind])/self.QRedgelength[ymaxind]>0.015:
            print("Direction [1, 0] and positive angle")
            dirv = [1, 0]
            for i in range(2):
                dir[i] = dir[i]+dirv[i]
        if (self.QRedgelength[yminind]-self.QRedgelength[ymaxind])/self.QRedgelength[ymaxind]>0.015:
            print("Direction [1, 0] and negative angle")
            dirv = [-1, 0]
            for i in range(2):
                dir[i] = dir[i]+dirv[i]
        return dir
#        print(cntpos)
#        print(self.QRedgelength)
        
    def addtext(self):
        opencvimage = np.array(self.image)
        imageData = opencvimage[:,:,::-1].copy()
        if hasattr(self, 'QRposition'):
            rectcoord = self.QRposition
        else:
            return
        font = cv2.FONT_HERSHEY_SIMPLEX
        # org
        org = (50, 50)
        # fontScale
        fontScale = 1
        # Blue color in BGR
        color = (255, 0, 0)
        # Line thickness of 2 px
        thickness = 2
        # Using cv2.putText() method
        try:
            mytext = f"[{rectcoord[0]}, {rectcoord[1]}]"
            imageData = cv2.putText(imageData, mytext, org, font, fontScale, color, thickness, cv2.LINE_AA)
        except:
            pass
        self.image = Image.fromarray(imageData)

    def save(self, filename = "capture"):
        if self.image is not None:
            try:
                self.image.save(filename+".png")
            except AttributeError: # if self.image is a numpy array.
                img = Image.fromarray(self.image)
                img.save(filename+".png")
            print("saved")

    def loadimages(self, dir=""):
        import glob
        if len(dir)>0:
            fnames = glob.glob(f"{dir}/*.png")
        else:
            fnames = glob.glob(f"*.png")
        images = []
        for fn in fnames:
            print(f"{fn} is loaded.")
            images.append(cv2.imread(fn))
        return images

    def calibrate(self, img = []): 
        #calibrate RoboticQ small checkerboard
        # most of the code here is from https://learnopencv.com/camera-calibration-using-opencv/

        # Defining the dimensions of checkerboard
        CHECKERBOARD = (6,7)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # Creating vector to store vectors of 3D points for each checkerboard image
        objpoints = []
    	# Creating vector to store vectors of 2D points for each checkerboard image
        imgpoints = []
    	# Defining the world coordinates for 3D points
        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        # 6 points in 35mm (0.035m)
        objp = objp*0.035/6
        #prev_img_shape = None
        # Extracting path of individual image stored in a given directory
        images = []
        if len(img)==0:
            print("Image will be captured")
            v, img = self.capture()
            images.append(img)
        else:
            if hasattr(img, 'shape'):
                images.append(img)
            else:
                images = img
        for img in images:
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                # Find the chess board corners
                # If desired number of corners are found in the image then ret = true
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            """
            If desired number of corner are detected,
            we refine the pixel coordinates and display
            them on the images of checker board
            """
            if ret == True:
                objpoints.append(objp)
                # refining pixel coordinates for given 2d points.
                corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
                imgpoints.append(corners2)
                ## Draw and display the corners
                #img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            #cv2.imshow('img',img)
            #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        """
            Performing camera calibration by
            passing the value of known 3D points (objpoints)
            and corresponding pixel coordinates of the
            detected corners (imgpoints)
        """
#        print(imgpoints)
        if len(imgpoints) > 0:
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
#            print("Camera matrix : \n")
#            print(mtx)
            # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
#            print("distortion coefficients : \n")
#            print(dist)
            # Rotation specified as a 3×1 vector. 
            # The direction of the vector specifies the axis of rotation and 
            # the magnitude of the vector specifies the angle of rotation. 
#            print("rvecs : \n")
#            print(rvecs)
            # for rv in rvecs:
            #     ang = np.linalg.norm(rv)
            #     rv = rv/ang
            #     print("rotation axis : \n")
            #     print(rv)
            #     print(f"rotation angle (radian) : {ang}\n")
#            print("tvecs : \n")
#            print(tvecs)
            self.intrinsic_mtx = mtx
            self.distCoeffs = dist
            self.objpoints = objpoints
            self.imgpoints = imgpoints
            return rvecs, tvecs
        else:
            print("Cannot find corners. Calibration failed.")

    def show(self):
        # press ESC to stop.
        ret, img = self.capture()
        while 1:    
        # Capture
            ret, img = self.capture()
            if not ret:
                print("Retrieve frame failed...")
                break
            cv2.imshow('frame', img)
            #rob.camera.decode2QR()
            key = cv2.waitKey(20) & 0xFF
            if key == 27:
                break
            time.sleep(0.02)
        cv2.destroyAllWindows()