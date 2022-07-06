#!/usr/bin/env python3

# yellow line detection node

import cv2
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from shifted_line_pkg.cfg import DetectYellowConfig

# global variables
yellow_detected = Bool()

################### callback ###################

def dynamic_reconfigure_callback(config, level):
    global RC
    RC = config
    return config

def image_callback(camera_image):

    try:
        cv_image = CvBridge().imgmsg_to_cv2(camera_image, "bgr8")
    except CvBridgeError:
        print(CvBridgeError)

    # get the dimensions of the image
    #height, width = cv_image.shape[0], cv_image.shape[1]

    # convert image to HLS
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)

    # crop the image
    cropped = cv_image[435:786, 340:500]

    #whitemask
    # w_lower = np.array([np.round(0/2), np.round(0.75 * 255), np.round(0.00 * 255)])
    # w_upper = np.array([np.round(360/2), np.round(1.00 * 255), np.round(0.30 * 255)])
    # white_mask = cv2.inRange(cv_image, w_lower, w_upper)

    #yellowmask
    # y_lower = np.array([np.round(40/2), np.round(0.00 * 255), np.round(0.35 * 255)])
    # y_upper = np.array([np.round(60/2), np.round(1.00 * 255), np.round(1.00 * 255)])
    y_lower = np.uint8([RC.hue_l, RC.light_l, RC.sat_l])
    y_upper = np.uint8([RC.hue_h, RC.light_h, RC.sat_h])
    yellow_mask_uncropped = cv2.inRange(cv_image, y_lower, y_upper)
    yellow_mask_cropped = cv2.inRange(cropped, y_lower, y_upper)

    #combine masks
    # mask = cv2.bitwise_or(yellow_mask, white_mask)
    # masked = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    contours, _ = cv2.findContours (yellow_mask_cropped, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # initialize the variables for computing the centroid and finding the largest contour
    max_area = 0
    max_contour = []

    if len(contours) != 0:
        # find the largest contour by its area
        max_contour = max(contours, key = cv2.contourArea)
        max_area = cv2.contourArea(max_contour)

    #draw the obtained contour lines(or the set of coordinates forming a line) on the original image
    cv2.drawContours(cv_image, max_contour, -1, (0, 0, 255), 5)

    # cv2.imshow("YELLOW_MASK_uncropped Window", yellow_mask_uncropped)

    # #If yellow blob big ---> say true
    if max_area > 100:
        cv2.drawContours(yellow_mask_cropped, max_contour, -1, (0, 255, 255), 5) # BGR
        # cv2.imshow("YELLOW_MASK DETECTED!!!", yellow_mask_cropped)
        yellow_detected.data = True
        yellow_detected_pub.publish(yellow_detected)
    else:
        yellow_detected.data = False

    yellow_detected_pub.publish(yellow_detected)

    cv2.waitKey(3)

################### helper functions ###################
def color_filter(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_bounds = (RC.hue_l, RC.sat_l, RC.val_l) # lower bounds of h, s, v for the target color
    upper_bounds = (RC.hue_h, RC.sat_h, RC.val_h) # upper bounds of h, s, v for the target color

    # return the yellow mask
    return cv2.inRange(hsv_image, lower_bounds, upper_bounds)


################### main ###################

if __name__ == "__main__":
    rospy.init_node("detect_yellow", anonymous=True)

    rospy.Subscriber("/camera/image_raw", Image, image_callback)

    yellow_detected_pub = rospy.Publisher("yellow_detected", Bool, queue_size=1)

    dynamic_reconfigure_server = Server(DetectYellowConfig, dynamic_reconfigure_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
