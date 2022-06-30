#!/usr/bin/env python3

# this file contains the algorithms to

import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# global variables
yaw_rate = Float32()

################### callback ###################

def image_callback(camera_image):

    try:
        # convert camera_image into an opencv-compatible image
        cv_image = CvBridge().imgmsg_to_cv2(camera_image, "bgr8")
    except CvBridgeError:
        print(CvBridgeError)

    # resize the image
    cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)

    # apply filters to the image
    balanced_image = find_white_balance(cv_image)
    filtered_image = apply_filters(balanced_image)

    # find the contours in the binary image
    contours, _ = cv2.findContours(filtered_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

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
    else:
        # rospy.loginfo(f"empty contours: {contours}")
        pass

    try:
        # draw the obtained contour lines (or the set of coordinates forming a line) on the original image
        cv2.drawContours(cv_image, max_contour, -1, (0, 0, 255), 10)
    except UnboundLocalError:
        rospy.loginfo("max contour not found")

    # draw a circle at centroid (https://www.geeksforgeeks.org/python-opencv-cv2-circle-method)
    cv2.circle(cv_image, (cx, cy), 8, (180, 0, 0), -1)  # -1 fill the circle

    # get the properties of the image
    (width, height, _) = cv_image.shape

    # offset the x position of the vehicle to follow the lane
    cx -= 170

    pub_yaw_rate(cv_image, cx, cy, width, height)

    concatenated_image = np.concatenate((cv_image, filtered_image), axis=1)
    cv2.imshow("Image", concatenated_image)
    cv2.waitKey(3)

################### filters ###################

def find_white_balance(cv_image):

    # convert image to the LAB color space
    lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

    average_a = np.average(lab_image[:,:,1])
    average_b = np.average(lab_image[:,:,2])

    lab_image[:,:,1] = lab_image[:,:,1] - ((average_a - 128) * (lab_image[:,:,0] / 255.0) * 1.1)
    lab_image[:,:,2] = lab_image[:,:,2] - ((average_b - 128) * (lab_image[:,:,0] / 255.0) * 1.1)

    return cv2.cvtColor(lab_image, cv2.COLOR_LAB2BGR)

def perspective_warp(image,
                     destination_size=(1280, 720),
                     source=np.float32([(0.43, 0.65), (0.58, 0.65), (0.1, 1), (1, 1)]),
                     destination=np.float32([(0, 0), (1, 0), (0, 1), (1, 1)])):

    image_size = np.float32([(image.shape[1], image.shape[0])])
    source = source * image_size

    # For destination points, I'm arbitrarily choosing some points to be a nice fit for displaying
    # our warped result again, not exact, but close enough for our purposes
    destination = destination * np.float32(destination_size)

    # given source and destination points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(source, destination)

    # return the warped image
    return cv2.warpPerspective(image, M, destination_size)

def apply_filters(cv_image):

    # apply white balance filter to even out the image
    cv_image = find_white_balance(cv_image)

    # convert image to the HLS color space
    hls_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HLS)

    # define the upper and lower bounds for white
    lower_bounds = np.uint8([0, 200, 0])
    upper_bounds = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(hls_image, lower_bounds, upper_bounds)

    # define the upper and lower bounds for yellow
    lower_bounds = np.uint8([ 10,   0, 100])
    upper_bounds = np.uint8([ 40, 255, 255])
    yellow_mask = cv2.inRange(hls_image, lower_bounds, upper_bounds)

    # combine the masks
    white_or_yellow_mask = cv2.bitwise_or(white_mask, yellow_mask)
    cv_image_using_mask =  cv2.bitwise_and(cv_image, cv_image, mask = white_or_yellow_mask)

    # convert image to grayscale
    gray_image = cv2.cvtColor(cv_image_using_mask, cv2.COLOR_BGR2GRAY)

    # smooth out the image
    kernel = np.ones((5, 5), np.float32) / 25
    smooth_image = cv2.filter2D(gray_image, -1, kernel)

    # find and return the edges in in smoothed image
    return cv2.Canny(smooth_image, 200, 255)

################### algorithms ###################

def pub_yaw_rate(cv_image, cx, cy, width, height):

    # compute the coordinates for the center the vehicle's camera view
    camera_center_x = (width / 2)
    camera_center_y = (height / 2)


    # compute the difference between the x and y coordinates of the centroid and the vehicle's camera center
    center_error = cx - camera_center_y

    # In simulation:
    #       less than 3.0 - deviates a little inward when turning
    #                 3.0 - follows the line exactly
    #       more than 3.0 - deviates a little outward when turning
    correction = 3.0 * camera_center_x

    # compute the yaw rate proportional to the difference between centroid and camera center
    angular_z = float(center_error / correction)

    if cx > camera_center_y:
        # angular.z is negative; left turn
        yaw_rate.data = -abs(angular_z)
    elif cx < camera_center_y:
        # angular.z is positive; right turn
        yaw_rate.data = abs(angular_z)
    else:
        # keep going straight
        yaw_rate.data = 0.0

    yaw_rate_pub.publish(yaw_rate)

    return

################### main ###################

if __name__ == "__main__":

    rospy.init_node("follow_line", anonymous=True)

    camera_topic = rospy.get_param("~camera_topic_name")  # as defined in the launch file
    rospy.Subscriber(camera_topic, Image, image_callback)

    yaw_rate_pub = rospy.Publisher("yaw_rate", Float32, queue_size=1)

    try:
      rospy.spin()
    except rospy.ROSInterruptException:
      pass
