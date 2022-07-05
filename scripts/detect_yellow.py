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
    height, width = cv_image.shape[0], cv_image.shape[1]

    # crop the image
    cv_image = cv_image[int(height / 2):, int(width / 3):]

    # filtered_image = color_filter(cv_image)
    # cv2.imshow("FRAME", filtered_image)


    filtered_image = cv2.cvtColor(cv_image ,cv2.COLOR_BGR2GRAY)

    filtered_image = cv2.GaussianBlur(filtered_image, (3, 3), 0)

    canny_image = cv2.Canny(filtered_image, RC.canny_low, RC.canny_high)

    line_image = np.zeros_like(cv_image) # blank frame to draw the Hough lines
    theta = RC.theta * (np.pi / 180)   # convert degrees to radians

    # Hough line detection on the image image
    hough_lines = cv2.HoughLinesP(canny_image, RC.rho, theta, RC.thresh, np.array([]), RC.min_length, RC.max_gap)
    if hough_lines is not None:
        for line in hough_lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), (255, 255, 0), 5)

    # draw the lines on the  image
    hough_image = cv2.addWeighted(cv_image, 0.8, line_image, 1, 0)

    #
    # # find contours in the binary (black and white) image
    # contours, _ = cv2.findContours (image_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #
    # # initialize the variables for computing the centroid and finding the largest contour
    # max_area = 0
    # max_contour = []
    #
    # if len(contours) != 0:
    #     # find the largest contour by its area
    #     max_contour = max(contours, key = cv2.contourArea)
    #     max_area = cv2.contourArea(max_contour)
    # else:
    #     yellow_detected.data = False
    #     yellow_detected_pub.publish(yellow_detected) return
    #
    # try:
    #     # draw the obtained contour lines (or the set of coordinates forming a line) on the original image
    #     # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
    #     cv2.drawContours(cv_image, max_contour, -1, (0, 0, 255), 5)
    # except UnboundLocalError:
    #     print("max contour not found")
    #
    # if max_area > 200:
    #     yellow_detected.data = True
    # else:
    #     yellow_detected.data = False
    #
    # cv2.imshow("Yellow", cv_image)

    # always publish false (WIP)
    yellow_detected.data = False
    yellow_detected_pub.publish(yellow_detected)

    # cv2.imshow("HOUGH", hough_image)
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
