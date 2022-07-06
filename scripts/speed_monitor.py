#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from dbw_polaris_msgs.msg import UlcReport

# global variables
speed_ms = 0.0
speed_mph = 0.0
speed_limit_mph = 7.0
average_speed_mph = 0.0
distance_traveled = 0.0


n_laps = 0
time_of_lap = 0
time_initial = 0
time_elapsed = 0
previous_time = 0
yellow_frames = 0

start_time = True

################### callback ###################

def ulc_report_callback(report):
    global speed_ms, speed_mph, time_initial, time_elapsed, start_time
    global previous_time, distance_traveled

    speed_ms = report.speed_meas
    speed_mph = speed_ms * 2.237

    # keep track of the total time the vehicle is in motion
    if (speed_ms > 0.0) and start_time:
        # get the initial time in seconds
        time_initial = report.header.stamp.secs
        start_time = False
    elif (speed_ms > 0.0) and not start_time:
        # the vehicle is in motion, keep adding the time that has elapsed
        time_final = report.header.stamp.secs
        time_elapsed += time_final - time_initial
        time_initial = time_final
    else:
        # the vehicle stopped, pause the time
        start_time = True

    # compute distance traveled using Riemann sum
    if (time_elapsed > 0) and (previous_time == 0):
        previous_time = time_elapsed
    elif (time_elapsed > 0) and ((time_elapsed - previous_time) == 1):
        # distance formula: speed * change_in_time
        distance_traveled += speed_ms * (time_elapsed - previous_time)

        # display the instantaneous speed and the time every second
        rospy.loginfo(f"Speed | Distance | Time : {speed_ms:0.1f} m/s -> {speed_mph:0.1f} mph | {distance_traveled:0.1f} m | {time_elapsed} s")
        previous_time = time_elapsed

def detect_yellow_callback(yellow_detected):
    global n_laps, speed_ms, yellow_frames, time_elapsed, time_of_lap

    # average speed to do one full lap around the road test course
    if yellow_detected.data and (yellow_frames < 55) and (speed_ms > 0.0): # NOTE: add a condition to make it continous frames using time
        yellow_frames += 1
    elif yellow_detected.data and (yellow_frames == 55) and (speed_ms > 0.0):
        n_laps += 1
        time_of_lap += time_elapsed - time_of_lap
        rospy.loginfo(f"Lap: {n_laps} | Average Speed: {average_speed_mph} mph | Time Taken: {time_of_lap} s")
        yellow_frames = 0

    return

################### main ###################

if __name__ == "__main__":
    rospy.init_node("speed_monitor", anonymous=True)

    rospy.Subscriber("yellow_detected", Bool, detect_yellow_callback)
    rospy.Subscriber("/vehicle/ulc_report", UlcReport, ulc_report_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
