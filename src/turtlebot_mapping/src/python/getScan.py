import math

import rospy
from sensor_msgs.msg import LaserScan


def get_scan(data):
    global SAMPLES
    global SAMPLES_VIEW
    scan = data
    scan_filter = []

    SAMPLES = len(scan.ranges)
    SAMPLES_VIEW = 360
    if SAMPLES_VIEW > SAMPLES:
        SAMPLES_VIEW = SAMPLES
    if SAMPLES_VIEW == 1 :
        scan_filter.append(scan.ranges[0])

    else: 
        scan_filter.extend(scan.ranges)
    for i in range(SAMPLES_VIEW):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i] = 0

    return scan_filter
