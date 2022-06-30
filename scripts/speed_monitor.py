#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

# global variables
speed = 0.0
time_elapsed = 0.0
average_speed = 0.0
meter_per_second = 0.0
length_of_road_course_in_meters = 86.32

n_laps = 0
time_start = 0

first_time = True
first_yellow_frame = True

################### callback ###################

def cmd_vel_callback(vel_msg):
    global speed, time_start, first_time, time_elapsed

    speed = vel_msg.linear.x

    # keep track of the total time the vehicle is in motion
    if (speed > 0.0) and first_time:
        time_start = rospy.Time.now()
        first_time = False
    elif (speed > 0.0) and not first_time:
        # the vehicle is in motion
        time_end = rospy.Time.now()
        time_elapsed += (time_end - time_start).to_sec()
        time_start = time_end
    else:
        # the vehicle stopped, pause the timer
        first_time = True

    return

def detect_yellow_callback(yellow_detected):
    global n_laps, time_elapsed, first_yellow_frame, meter_per_second

    # average speed to do one full lap around the road test course
    if yellow_detected.data and first_yellow_frame and (time_elapsed > 0.0) and (speed > 0.0):
        meter_per_second = length_of_road_course_in_meters / time_elapsed
        # kilometers_per_hour = meter_per_second * 3.6
        miles_per_hour = meter_per_second * 2.237
        n_laps += 1
        rospy.loginfo(f"Lap: {n_laps} | Average Speed: {meter_per_second:0.2f} m/s, {miles_per_hour:0.2f} mi/h | Time Elapsed: {time_elapsed:0.0f} s")
        time_elapsed = 0.0
        first_yellow_frame = False
    elif not yellow_detected.data and not first_yellow_frame and (speed > 0.0):
        first_yellow_frame = True

    return

################### main ###################

if __name__ == "__main__":
    rospy.init_node("speed_monitor", anonymous=True)

    rospy.Subscriber("yellow_detected", Bool, detect_yellow_callback)
    rospy.Subscriber("/vehicle/cmd_vel", Twist, cmd_vel_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
