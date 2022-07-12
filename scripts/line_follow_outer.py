#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from shifted_line_pkg.cfg import LineFollowConfig

# global variables
yaw_rate = Float32()

################### callback ###################

def dynamic_reconfigure_callback(config, level):
    global RC
    RC = config
    return config

def image_callback(camera_image):
    global cx, cy
    try:
        # convert camera_image into an opencv-compatible image
        cv_image = CvBridge().imgmsg_to_cv2(camera_image, "bgr8")
    except CvBridgeError:
        print(CvBridgeError)

    # resize the image
    cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)

    roi_image = get_region_of_interest(cv_image)
    filtered_roi_image = apply_filters(roi_image)

    # find the contours in the binary image
    contours, _ = cv2.findContours(filtered_roi_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # initialize the variables for computing the centroid and finding the largest contour
    cx = 0
    cy = 0
    max_contour = []

    if len(contours) != 0:
        # find the largest contour by its area
        max_contour = max(contours, key = cv2.contourArea)

        # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        M = cv2.moments(max_contour)

        if M["m00"] != 0:
            # compute the x and y coordinates of the centroid
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

    try:
        # draw the obtained contour lines (or the set of coordinates forming a line) on the original image
        cv2.drawContours(roi_image, max_contour, -1, (0, 0, 255), 10)
    except UnboundLocalError:
        rospy.loginfo("max contour not found")

    # draw a circle at centroid (https://www.geeksforgeeks.org/python-opencv-cv2-circle-method)
    cv2.circle(roi_image, (cx, cy), 8, (180, 0, 0), -1)  # -1 fill the circle

    # offset the x position of the vehicle to follow the lane
    cx -= 170

    pub_yaw_rate(roi_image, cx, cy)

    # concatenate the roi images to show in a single window
    # the shape of the images must have the same length: len(image.shape)
    filtered_roi_image_with_channel = cv2.cvtColor(filtered_roi_image, cv2.COLOR_GRAY2BGR)
    concatenated_roi_image = cv2.vconcat([roi_image, filtered_roi_image_with_channel])

    cv2.imshow("ROI and Filtered ROI", concatenated_roi_image)
    cv2.waitKey(3)

################### filters and perspective ###################

def apply_white_balance(cv_image):

    # convert image to the LAB color space
    lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

    average_a = np.average(lab_image[:, :, 1])
    average_b = np.average(lab_image[:, :, 2])

    lab_image[:, :, 1] = lab_image[:, :, 1] - ((average_a - 128) * (lab_image[:, :, 0] / 255.0) * 1.1)
    lab_image[:, :, 2] = lab_image[:, :, 2] - ((average_b - 128) * (lab_image[:, :, 0] / 255.0) * 1.1)

    return cv2.cvtColor(lab_image, cv2.COLOR_LAB2BGR)

def apply_filters(cv_image):

    # helps remove some of the yellow from the sunlight
    balanced_image = apply_white_balance(cv_image)

    # one more time
    balanced_image = apply_white_balance(balanced_image)

    # convert image to the HLS color space
    hls_image = cv2.cvtColor(balanced_image, cv2.COLOR_BGR2HLS)

    # lower and upper bounds for the color white
    lower_bounds = np.uint8([0, RC.light_low, 0])
    upper_bounds = np.uint8([255, 255, 255])
    white_detection_mask = cv2.inRange(hls_image, lower_bounds, upper_bounds)

    # lower and upper bounds for the color yellow
    # lower_bounds = np.uint8([10, 0, 100])
    # upper_bounds = np.uint8([40, 255, 255])
    # yellow_detection_mask = cv2.inRange(hls_image, lower_bounds, upper_bounds)

    # combine the masks
    # white_or_yellow_mask = cv2.bitwise_or(white_detection_mask, yellow_mask)
    balanced_image_with_mask =  cv2.bitwise_and(balanced_image, balanced_image, mask = white_detection_mask)

    # convert image to grayscale
    gray_balanced_image_with_mask = cv2.cvtColor(balanced_image_with_mask, cv2.COLOR_BGR2GRAY)

    # smooth out the image
    kernel = np.ones((5, 5), np.float32) / 25
    smoothed_gray_image = cv2.filter2D(gray_balanced_image_with_mask, -1, kernel)

    # find and return the edges in in smoothed image
    return cv2.Canny(smoothed_gray_image, 200, 255)

def get_region_of_interest(image):

    width = image.shape[1]
    height = image.shape[0]

    width = width / 8
    height = height / 8

    # get the region of interest
    droi = np.array([[

                   [width * 4, height * 8],
                   [width * 4, height * 4],
                   [width * 5, height * 4],
                   [(width * 7), height * 6],
                   [(width * 7) + 50, height * 8]

               ]], dtype = np.int32)

    blank_frame = np.zeros_like(image)
    roi_mask = cv2.fillPoly(blank_frame, droi, (255, 255, 255))

    # combine the region of interest with the mask
    roi_image = cv2.bitwise_and(image, roi_mask)

    # crop the black edges and return cropped image
    y_nonzero, x_nonzero, _ = np.nonzero(roi_image)
    return roi_image[np.min(y_nonzero):np.max(y_nonzero), np.min(x_nonzero):np.max(x_nonzero)]

################### algorithms ###################

def pub_yaw_rate(image, cx, cy):

    # get the dimension of the image
    height, width = image.shape[0], image.shape[1]

    # compute the coordinates for the center the vehicle's camera view
    camera_center_y = (height / 2)
    # camera_center_x = (width / 2)
    # set camera center x to zero in consideration of the new ROI
    camera_center_x = 0

    # compute the difference between the x and y coordinates of the centroid and the vehicle's camera center
    center_error = cx - camera_center_x

    # In simulation:
    #       less than 3.0 - deviates a little inward when turning
    #                 3.0 - follows the line exactly
    #       more than 3.0 - deviates a little outward when turning
    correction = 3.0 * camera_center_y

    # compute the yaw rate proportion to the difference between centroid and camera center
    angular_z = float(center_error / correction)

    if cx > camera_center_x:
        # angular.z is negative; left turn
        yaw_rate.data = -abs(angular_z)
    elif cx < camera_center_x:
        # angular.z is positive; right turn
        yaw_rate.data = abs(angular_z)
    else:
        # keep going straight
        yaw_rate.data = 0.0

    yaw_rate_pub.publish(yaw_rate)

    return

################### main ###################
if __name__ == "__main__":

    rospy.init_node("line_follow_outer", anonymous=True)

    rospy.Subscriber("/camera/image_raw", Image, image_callback)

    yaw_rate_pub = rospy.Publisher("/yaw_rate", Float32, queue_size=1)

    dynamic_reconfigure_server = Server(LineFollowConfig, dynamic_reconfigure_callback)

    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass
