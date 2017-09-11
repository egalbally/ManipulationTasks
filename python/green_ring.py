#!/usr/bin/env python3
import numpy as np
import cv2
import threading
import time
import signal
import matplotlib.pyplot as plt
from data_logger import KinectStreamer
from skimage.transform import hough_ellipse

def filter_red(app, img_rgb):
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
    # imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), "HSV")
    # plt.clf()
    # plt.imshow(np.hstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), cmap="nipy_spectral")
    # plt.colorbar()
    # plt.pause(0.01)

    mask_fg = np.maximum(cv2.inRange(img_hsv, np.array([130,50,0]), np.array([180,255,255])),
                      cv2.inRange(img_hsv, np.array([0  ,50,0]), np.array([30 ,255,255])))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_OPEN, np.ones((10,10)))
    mask_bg = cv2.bitwise_not(mask_fg)
    mask_bg = cv2.morphologyEx(mask_bg, cv2.MORPH_ERODE, np.ones((100,100)))
    mask_fg = cv2.morphologyEx(mask_fg, cv2.MORPH_ERODE, np.ones((30,30)))
    app.imshow(mask_fg, "Mask")
    app.imshow(mask_bg, "Background Mask")

    markers = np.zeros(mask_fg.shape, dtype=np.int32)
    markers[mask_fg > 0] = 1
    markers[mask_bg > 0] = 2
    markers = cv2.watershed(img_hsv, markers)
    mask_red = (255 * (markers==1)).astype(np.uint8)

    # img_red = img_rgb.copy()
    # moments = cv2.moments(mask, True)
    # seed = np.array([moments["m10"]/moments["m00"], moments["m01"]/moments["m00"]]).round().astype(np.uint8)
    # cv2.floodFill(img_red, np.zeros((mask.shape[0]+2, mask.shape[1]+2), dtype=np.uint8), (seed[0], seed[1]), 0)
    # imshow(img_red)

    img_hsv = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_red)
    img_rgb2 = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
    return img_rgb2

def imshow(name, img, scale=1):
    cv2.imshow(name, cv2.resize(img, (int(round(scale * img.shape[1])), int(round(scale * img.shape[0])))))

if __name__ == "__main__":
    # from skimage.transform import hough_ellipse
    # from skimage.draw import ellipse_perimeter
    # img = np.zeros((25, 25), dtype=np.uint8)
    # rr, cc = ellipse_perimeter(10, 10, 6, 8)
    # img[cc, rr] = 1
    # result = hough_ellipse(img, threshold=8)
    # print(result.tolist())
    # exit()

    app = KinectStreamer()

    # Start Redis thread
    t1 = threading.Thread(target=app.stream_thread)
    t1.daemon = True
    t1.start()

    while app.run_loop:
        app.lock.acquire()
        try:
            img_rgb = np.copy(app.img_color)
            # img_rgb = np.round(np.mean(app.img_buffer, axis=3)).astype(np.uint8)
        except:
            app.lock.release()
            time.sleep(1)
            continue
        app.lock.release()

        img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)
        img_hsv_stack = cv2.resize(np.vstack((img_hsv[:,:,0], img_hsv[:,:,1], img_hsv[:,:,2])), (640, 1080))
        # img_hsv_stack = cv2.resize(np.vstack((img_hsv[:,:,0],)), (640, 1080))
        plt.imshow(img_hsv_stack, colormap="jet")
        plt.colorbar()
        plt.show()
        app.run_loop = False
        continue

        mask_green = cv2.inRange(img_hsv, np.array([30, 80,  180]),
                                          np.array([60, 255, 255]))
        # mask_orange = cv2.inRange(img_hsv, np.array([30, 80,  180]),
        #                                    np.array([60, 255, 255]))
        imshow("Green Mask", mask_green, 0.3)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_ERODE, np.ones((5,5), np.uint8))
        # mask_green = cv2.medianBlur(mask_green, 9)
        # imshow("Green Mask 2", mask_green, 0.3)
        idx_rows, idx_cols = np.where(mask_green>0)
        if len(idx_rows) < 1000:
            continue
        points = np.column_stack((idx_cols, idx_rows))
        ellipse = cv2.fitEllipse(points)
        img_ellipse = cv2.ellipse(img_rgb, ellipse, (255,0,255), 10)
        imshow("Ellipse", img_ellipse, 0.3)
        print(ellipse)
        # pos_crop = (idx_rows.min(), idx_cols.min())
        # mask_green_crop = mask_green[pos_crop[0]:idx_rows.max()+1, pos_crop[1]:idx_cols.max()+1]
        # # mask_green_crop = cv2.Canny(mask_green_crop, 100, 200)
        # # mask_green_crop[mask_green_crop>0] = 1

        # imshow("Green Mask", mask_green_crop, 0.3)

        # print("hough ellipse start")
        # result = hough_ellipse(mask_green, threshold=200, accuracy=250, min_size=100)
        # result.sort(order="accumulator")
        # print("hough ellipse finish")
        # print(result.tolist())

