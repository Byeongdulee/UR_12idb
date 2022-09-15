import numpy as np
import requests
from PIL import Image
import io
import math
import time
from pyzbar import pyzbar
import cv2
import threading

focus_threshold = 510
def variance_of_laplacian(image):
    return cv2.Laplacian(image, cv2.CV_64F).var()

def isblurry(image):
    size = image.shape
    # 9:16 size
    # 72:128
    ysize = 72
    xsize = 128
    gray = cv2.cvtColor(image[(size[0]/2-ysize):(size[0]/2+ysize-1), (size[1]/2-xsize):(size[1]/2+xsize-1)], cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    val = False
    if fm<focus_threshold:
        val = True
    return val

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
def decodeQR(img):
    img2 = img
    QRdata = pyzbar.decode(img2)
    n = len(QRdata)
    if n < 1:
        return []
    return QRdata

def showQRcode(QRdata, image):
    for barcode in QRdata:
        (x, y, w, h) = barcode.rect
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 5)

class camera(object):
    def __init__(self,
                 IP="", device=0):
        self.IP = IP
        self.device = device
        self._running = False
        if len(self.IP) == 0:
            vidcap = cv2.VideoCapture(self.device)
            if not vidcap.isOpened():
                print("Cannot open camera {}".format(self.device))
                exit()
            vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.vidcap = vidcap
            self.focus(520)
    
    def scanfocus(self):
        for i in range(350,600,5):
            if not isblurry(self.capture()):
                break
            self.focus(i)
            time.sleep(0.1)
        foc = self.vidcap.get(cv2.CAP_PROP_FOCUS)
        return foc

    def focus(self, val):
        #  #Not sure what max and min range are for the camera focus, 
        # I found that a value of 450 works well at a 1 ft range under current conditions
        if len(self.IP) == 0:
            self.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,0)
            self.vidcap.set(cv2.CAP_PROP_FOCUS, val)

    def autofocus(self):
        if len(self.IP) == 0:
            self.vidcap.set(cv2.CAP_PROP_AUTOFOCUS,1)
            foc = self.vidcap.get(cv2.CAP_PROP_FOCUS)
            return foc

    def get_foc(self):
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
                ret = True
#                print("Successfully captured.")
        else:
            ret, pilImage = self.vidcap.read()
            if not ret:
                print("Fail to capture camera.")
            # else:
            #     print("read successfuuly.")
        self.image = pilImage
        return ret, pilImage

    def decode(self, p0in=(0,0), p1in=(0,0), imgwidth=866, imgheight=650, color = (0, 0, 255), thickness = 1):
        #imgdata = self.image
        opencvimage = np.array(self.image)
        imgdata = opencvimage[:,:,::-1].copy()
        QRdata = decodeQR(imgdata)
        rectcoord = []
        data = []
        dist = []
        pgpnts = []

        qrsize = [0, 0]
        if len(QRdata)>0:
            showQRcode(QRdata, imgdata)
        else:
            self.QRdata = data
            self.QRposition = rectcoord
            self.QRsize = qrsize            
            self.QRedgelength = dist
            self.QRcoordinates = pgpnts
            return data, rectcoord, qrsize, dist

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
        self.image = Image.fromarray(imgdata)
        self.QRdata = data
        self.QRposition = rectcoord
        self.QRsize = qrsize
        self.QRedgelength = dist
        self.QRcoordinates = pgpnts
        return data, rectcoord, qrsize, dist

    def decode2QR(self, p0in=(0,0), p1in=(0,0), imgwidth=866, imgheight=650, color = (0, 0, 255), thickness = 1):
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
        if len(QRdata)>1:
            showQRcode(QRdata, imgdata)
        else:
            self.QRtype = "2QR"
            self.QRdata = data
            self.QRposition = rectcoord
            self.QRsize = qrsize            
            self.QRedgelength = dist
            self.QRcoordinates = pgpnts
            return data, rectcoord, qrsize, dist

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
            self.image.save(filename+".png")
            print("saved")
