#!/usr/bin/env python
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rospy
import numpy as np

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def img_callback(data):

    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node('cam_tune', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
# rospy.spin()

def get_crosspt(x11,y11, x12,y12, x21,y21, x22,y22):

    if x12==x11 or x22==x21:
        # print('delta x=0')
        return None
    
    m1 = (y12 - y11) / (x12 - x11)
    m2 = (y22 - y21) / (x22 - x21)
    
    if m1==m2:
        # print('parallel')
        return None
    
    cx = (x11 * m1 - y11 - x21 * m2 + y21) / (m1 - m2)
    cy = m1 * (cx - x11) + y11

    return cx, cy

def lane_detect():
    while not rospy.is_shutdown():
        if cv_image.size != (640*480*3):
            continue
        # img = cv_image.copy()
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # print(gray_img)
        blur_img = cv2.GaussianBlur(gray_img,(5,5) ,0)
        canny_img = cv2.Canny(blur_img, 150, 250)
        mask = np.zeros_like(canny_img)
        vertices = np.array([[0,400], [280,200], [350,200],[640,400]], dtype=np.int32)
        cv2.fillPoly(mask, [vertices], 255)
        roi_img = cv2.bitwise_and(canny_img, mask)
        
        lines = cv2.HoughLinesP(roi_img, rho=1, theta=np.pi/180, threshold=30,
                                minLineLength=100, maxLineGap=30)
        lines = np.squeeze(lines)
        # print("lines  ", lines)
        try:
            slope_deg =  np.rad2deg(np.arctan2(lines[:, 1] - lines[:, 3], lines[:, 0]- lines[:, 2]))
            # lines = lines[np.abs(slope_deg) < 155]
            # slope_deg = slope_deg[np.abs(slope_deg) < 155]

            left_lines = lines[slope_deg>0]
            right_lines = lines[slope_deg<0]

            sum_x1_l, sum_x2_l, sum_y1_l, sum_y2_l, = 0, 0 ,0, 0
            sum_x1_r, sum_x2_r, sum_y1_r, sum_y2_r = 0, 0 ,0, 0

            for x1, y1, x2, y2 in left_lines:
                sum_x1_l += x1
                sum_x2_l += x2
                sum_y1_l += y1
                sum_y2_l += y2

            for x1, y1, x2, y2 in right_lines:
                sum_x1_r += x1
                sum_x2_r += x2
                sum_y1_r += y1
                sum_y2_r += y2
            
            x11 = sum_x1_l//len(left_lines)
            y11 = sum_y1_l//len(left_lines)
            x12 = sum_x2_l//len(left_lines)
            y12 = sum_y2_l//len(left_lines)
            x21 = sum_x1_r//len(right_lines)
            y21 = sum_y1_r//len(right_lines)
            x22 = sum_x2_r//len(right_lines)
            y22 = sum_y2_r//len(right_lines)

            cx, cy = get_crosspt(x11, y11, x12, y12, x21, y21, x22, y22)
            
        except:
            pass

        cv2.line(cv_image, (x11, y11), (x12, y12), color=[255,0,0], thickness=2)
        cv2.line(cv_image, (x21, y21), (x22, y22), color=[255,0,0], thickness=2)
        cv2.circle(cv_image, (int(cx), int(cy)), 5, color= [0,255,0], thickness = -1)
        cv2.line(cv_image, (320,480), (320,0), color= [0,0,255], thickness = 2)

        cv2.imshow("original", cv_image)
        key = cv2.waitKey(1)

        if key == 27:
            cv2.destroyAllWindows()
    # cv2.imwrite('lane.jpg', cv_image)



if __name__ == '__main__':
    lane_detect()
        
