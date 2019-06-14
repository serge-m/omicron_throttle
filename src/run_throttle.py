#!/usr/bin/python3

import os
import json

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32

# GLOBAL VARIABLES
DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.join(DIR, "..", "../throttle_config.json")

CONFIG = dict()


def set_variables():
    if not os.path.exists(CONFIG_DIR):
        raise ("CONFIG does not exist! Please calibrate your car first!")

    with open(CONFIG_DIR) as f:
        d = json.load(f)

        CONFIG["THROTTLE_FORWARD_PWM"] = d["THROTTLE_FORWARD_PWM"]
        CONFIG["THROTTLE_STOPPED_PWM"] = d["THROTTLE_STOPPED_PWM"]
        CONFIG["THROTTLE_REVERSE_PWM"] = d["THROTTLE_REVERSE_PWM"]


def map_range(x, x_min, x_max, y_min, y_max):
    """
    Linear mapping between two ranges of values
    """
    x = float(x)
    x_range = x_max - x_min
    y_range = y_max - y_min
    xy_ratio = x_range / y_range

    y = ((x - x_min) / xy_ratio + y_min)

    return int(y)


class ROSPackageThrottle:
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self):
        self.throttling_publisher = rospy.Publisher('channel_15', Int32, queue_size=10)

    def callback(self, data):
        throttle = data.drive.speed
        if throttle > 0:
            pulse = map_range(throttle, 0, self.MAX_THROTTLE, CONFIG["THROTTLE_STOPPED_PWM"],
                              CONFIG["THROTTLE_FORWARD_PWM"])
        else:
            pulse = map_range(throttle, self.MIN_THROTTLE, 0, CONFIG["THROTTLE_REVERSE_PWM"],
                              CONFIG["THROTTLE_STOPPED_PWM"])

        self.throttling_publisher.publish(pulse)

    def throttle_subscriber(self):
        rospy.init_node('throttle_config', anonymous=False)
        rospy.Subscriber('ackermann_cmd_mux/output', AckermannDriveStamped, self.callback)
        rospy.spin()


if __name__ == '__main__':
    set_variables()

    package = ROSPackageThrottle()
    package.throttle_subscriber()
