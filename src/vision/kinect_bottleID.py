# coding: utf-8

from mpl_toolkits.mplot3d import Axes3D


import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel
from matplotlib import pyplot as plt

from cv2 import rgbd as rgbd 
from numpy import linalg as LA



try:
    from pylibfreenect2 import OpenCLPacketPipeline
    pipeline = OpenCLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenGLPacketPipeline
        pipeline = OpenGLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)



def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )


def find_squares(img_depth):

    gray = cv2.GaussianBlur(np.uint8(img_depth), (5, 5), 0)
    squares = []
    
    for thrs in xrange(0, 255, 26):
        if thrs == 0:
            bin = cv2.Canny(gray, 0, 50, apertureSize=5)
            bin = cv2.dilate(bin, None)
        else:
            _retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
        bin, contours, _hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            cnt_len = cv2.arcLength(cnt, True)
            cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
            if len(cnt) == 4 and cv2.contourArea(cnt) > 500 and cv2.isContourConvex(cnt):
                cnt = cnt.reshape(-1, 2)
                max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                if max_cos < 0.1:
                    squares.append(cnt)
    return squares


# Create and set logger
logger = createConsoleLogger(LoggerLevel.Error)
setGlobalLogger(logger)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()

# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())


#print device.getIRCameraParams()

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

params = device.getColorCameraParams()

K_IR = np.array([[params.fx, 0, params.cx], [0, params.fy, params.cy], [0,0, 1]])

# Optinal parameters for registration
# set True if you need
need_bigdepth = True
need_color_depth_map = True

# fig = plt.figure(1)
# ax1 = fig.add_subplot(1,1,1, projection='3d')

# plt.show()

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if need_color_depth_map else None

counter = 0


THRESHOLD = 1.5

minLineLength = 10
maxLineGap = 10

while True:
   
    frames = listener.waitForNewFrame()

    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    registration.apply(color, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)

    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.
    #cv2.imshow("ir", ir.asarray() / 65535.)
    #cv2.imshow("depth", depth.asarray() / 4500.)
    cv2.imshow("color", cv2.resize(color.asarray(),
                                   (int(1920 / 3), int(1080 / 3))))
    
    reg = registered.asarray(np.uint8)

    reg = cv2.medianBlur(reg, 5)


    #print rgbd.shape
    
    # print rgbd

    cv2.imshow("registered", reg)

  



    color_array = reg[:,:,0:3]

    #if need_bigdepth:
    #   cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
     #                                     (int(1920 / 3), int(1082 / 3))))
    #if need_color_depth_map:
      #  cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

    # color_array = cv2.resize(color.asarray(),(int(512), int(424)))
    
    img_hsv = cv2.cvtColor(color_array, cv2.COLOR_BGR2HSV)


    # imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), "HSV")
    # plt.clf()
    # plt.imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), cmap="nipy_spectral")
    # plt.colorbar()
    # plt.pause(0.01)

    mask_fg = np.maximum(cv2.inRange(img_hsv, np.array([170,200,0]), np.array([180,255,255])),
                      cv2.inRange(img_hsv, np.array([0,200,0]), np.array([5 ,255,255])))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_OPEN, np.ones((8,8)))
    mask_bg = cv2.bitwise_not(mask_fg)
    mask_bg = cv2.morphologyEx(mask_bg, cv2.MORPH_ERODE, np.ones((100,100)))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_ERODE, np.ones((10,10)))

    cv2.imshow("Mask", mask_fg)
    cv2.imshow("Background Mask", mask_bg)
    depth_img = depth.asarray()/ 4500.

    cv2.imshow("reigstered color", color_array)
    cv2.imshow(" depth", depth_img)

    



    markers = np.zeros(mask_fg.shape, dtype=np.int32)
    markers[mask_fg > 0] = 1
    markers[mask_bg > 0] = 2
    markers = cv2.watershed(img_hsv, markers)
    mask_red = (255 * (markers==1)).astype(np.uint8)
    mask_red = cv2.medianBlur(mask_red, 3)


    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_DILATE, np.ones((40,40)))


    masked = cv2.bitwise_and(color_array, color_array, mask=mask_red)



    # squares = find_squares(masked)
    # cv2.drawContours( masked, squares, -1, (0, 255, 0), 3 )
   
    gray = cv2.cvtColor(masked,cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(np.uint8(gray),50,150,apertureSize = 3)

    lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)

    if (lines != None): 
        for x1,y1,x2,y2 in lines[0]:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)

        cv2.imwrite('masked with lines',gray)

    cv2.imshow('squares', gray)
        

    
    img_hsv[markers == 2] = [0, 0, 0]
    depth_img2 = depth_img.copy()
    depth_img2[markers ==2] = [0]

    # color_array[markers == -1] = [0,0,255]

    unique, counts = np.unique(markers, return_counts = True)

    depth_img2 = cv2.medianBlur(depth_img2, 3)




   
    # moments = cv2.moments(mask_red, True)

    # print(moments["m01"])
    # print(moments["m00"])

    # seed = np.array([moments["m10"]/moments["m00"], moments["m01"]/moments["m00"]]).round().astype(np.uint8)
    # # cv2.floodFill(depth_img, np.zeros((mask_red.shape[0]+2, mask_red.shape[1]+2), dtype=np.uint8), (seed[0], seed[1]), 0)

    # cv2.imshow("flood fill", depth_img)



    #img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_red)
    # img_rgb2 = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)


    # cv2.imshow("marked img", img_rgb2)
    # cv2.imshow("marked depth", depth_img2)
    # cv2.imshow("color id", masked)
    
    # points3d = np.zeros(img_hsv.shape, dtype=np.float32)

    # rgbd.depthTo3d(depth_img2, K_IR, points3d)


    # red_loc = points3d[markers==1]



    #print red_loc[:,2]

    # valueError = False 
    # try:
    #     far_red = np.amax(red_loc[:,2])
    # except ValueError:  #raised if `y` is empty.
    #     valueError = True
    #     print "value error"
    #     pass
    # try:
    #     close_red = np.amin(red_loc[:,2])
    # except ValueError:
    #     valueError = True
    #     print "value error"
    #     pass


    # if (not valueError): 

    #     length_thres= (far_red-close_red)/THRESHOLD

    #     red_loc_close = red_loc[red_loc[:,2]<(close_red+length_thres)]


    #     covariance = red_loc_close.T.dot(red_loc_close)
    #     w, v = LA.eig(covariance)

    #     index = np.argmax(w)

    #     if (counter%5==0):
    #         print red_loc_close.shape 
    #         print v[index]


    # print points3d[:,:,0]



    # # ax1.clear()
    # ax1.scatter(points3d[:,:,0], points3d[:,:,1], points3d[:,:,2])

    
    # plt.draw()
    # #cv2.imshow("3d depth", points3d)


    # print "depth img shape"
    # print depth_img.shape
    # print "depth mask shape"
    # print  mask_red.shape

    #depth_img_filter = cv2.bitwise_and(depth_img, depth_img, mask=mask_red)



    listener.release(frames)

    counter+=1

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

device.stop()
device.close()

sys.exit(0)