import robot12idb as rb
import cv2
rob = rb.UR5()
img = rob.camera.capture()
print('img type/shape:', type(img), getattr(img,'shape',None))
r = None
try:
    r = rob.camera.decodeAT()
    print('decodeAT succeeded')
except Exception as ex:
    print('decodeAT error:', ex)
print('type(r)=', type(r))
print('repr(r)[:500]=', repr(r)[:500])
if isinstance(r, list):
    print('len(r)=', len(r))
    for i, x in enumerate(r[:3]):
        print(i, type(x), dir(x)[:200])
        print('has pose_R? ', hasattr(x,'pose_R'))
        pr = getattr(x,'pose_R',None)
        try:
            print('pose_R shape:', None if pr is None else pr.shape)
        except Exception as e:
            print('pose_R repr error:', e)
        print('pose_R repr:', pr)
        print('has pose_t? ', hasattr(x,'pose_t'))
        print('pose_t:', getattr(x,'pose_t',None))
else:
    print('has pose_R? ', hasattr(r,'pose_R'))
    print('pose_R:', getattr(r,'pose_R',None))
    print('has pose_t? ', hasattr(r,'pose_t'))
    print('pose_t:', getattr(r,'pose_t',None))

# #If decodeAT() errors or is not available, run:
# from pupil_apriltags import Detector
# from common import urcamera
# at = Detector(families='tag36h11')
# gray = img if len(img.shape)==2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# fx = rob.camera.camera_f
# fy = rob.camera.camera_f
# cx = rob.camera.imgH/2
# cy = rob.camera.imgV/2
# r_pose = at.detect(gray, estimate_tag_pose=True, camera_params=[fx,fy,cx,cy], tag_size=rob.camera.AT_physical_size)
# print('type(r_pose)=', type(r_pose))
# print('len(r_pose)=', len(r_pose) if hasattr(r_pose,'__len__') else 'no len')
# for i,x in enumerate(r_pose[:3]):
#     print(i, type(x), dir(x)[:200])
#     print('pose_R:', getattr(x,'pose_R',None))
#     print('pose_t:', getattr(x,'pose_t',None))
