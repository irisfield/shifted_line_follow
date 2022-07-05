#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from dbw_polaris_msgs.msg import SteeringReport

# global variables
speed_ms = 0.0
speed_mph = 0.0
speed_limit_mph = 7.0
average_speed_mph = 0.0
distance_traveled = 0.0

time_of_lap = 0.0
time_elapsed = 0.0
length_of_road_course_in_meters = 86.32

n_laps = 0
time_start = 0

first_time = True
first_yellow_frame = True

################### callback ###################

def steering_report_callback(report):
    global speed_ms, speed_mph, first_time, time_start, time_elapsed

    speed_ms = report.speed
    speed_mph = speed_ms * 2.237

    # keep track of the total time the vehicle is in motion
    if (speed_ms > 0.0) and first_time:
        time_start = rospy.Time.now()
        first_time = False
    elif (speed_ms > 0.0) and not first_time:
        # the vehicle is in motion
        time_end = rospy.Time.now()
        time_elapsed += (time_end - time_start).to_sec()
        time_start = time_end
    else:
        # the vehicle stopped, pause the timer
        first_time = True

    if (int(time_elapsed) % 2 == 1):
        # display the instantaneous speed every other second
        rospy.loginfo(f"Speed: {speed_ms} m/s | {speed_mph} mph")

    return


def detect_yellow_callback(yellow_detected):
    global n_laps, speed_ms, first_yellow_frame, time_elapsed, time_of_lap

    # average speed to do one full lap around the road test course
    if yellow_detected.data and first_yellow_frame and (time_elapsed > 0.0) and (speed_ms > 0.0):
        n_laps += 1

        time_of_lap += time_elapsed - time_of_lap
        rospy.loginfo(f"Lap: {n_laps} | Average Speed: {average_speed_mph} mph | Time: {time_of_lap} ")

        first_yellow_frame = False
    elif not yellow_detected.data and not first_yellow_frame and (speed_ms > 0.0):
        first_yellow_frame = True

    return

################### main ###################

if __name__ == "__main__":
    rospy.init_node("speed_monitor", anonymous=True)

    rospy.Subscriber("yellow_detected", Bool, detect_yellow_callback)
    # rospy.Subscriber("/vehicle/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/vehicle/steering_report", SteeringReport, steering_report_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
