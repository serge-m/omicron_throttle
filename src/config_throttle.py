#!/usr/bin/env python

import os
import json
import rospy

from std_msgs.msg import Int32
from dynamic_reconfigure.server import Server

from throttle.cfg import ThrottleConfig

DIR = os.path.dirname(os.path.realpath(__file__))
CONFIG_DIR = os.path.join(DIR, "..", "../throttle_config.json")

CONFIG = dict()
PUBLISHER = rospy.Publisher('channel_15', Int32, queue_size=10)
FIRST = True


def set_variables():
    global CONFIG
    if not os.path.exists(CONFIG_DIR):
        CONFIG["THROTTLE_FORWARD_PWM"] = 0
        CONFIG["THROTTLE_STOPPED_PWM"] = 0
        CONFIG["THROTTLE_REVERSE_PWM"] = 0

    else:
        with open(CONFIG_DIR) as f:
            CONFIG = json.load(f)


def dump_data():
    with open(CONFIG_DIR, 'w') as outfile:
        json.dump(CONFIG, outfile)


def callback(received_config, level):
    global FIRST
    if FIRST:
        received_config["THROTTLE_FORWARD_PWM"] = CONFIG["THROTTLE_FORWARD_PWM"]
        received_config["THROTTLE_REVERSE_PWM"] = CONFIG["THROTTLE_REVERSE_PWM"]
        received_config["THROTTLE_STOPPED_PWM"] = CONFIG["THROTTLE_STOPPED_PWM"]
        FIRST = False

    rospy.loginfo("""Current CONFIG: {THROTTLE_FORWARD_PWM} {THROTTLE_STOPPED_PWM} {THROTTLE_REVERSE_PWM}""".format(**CONFIG))
    rospy.loginfo("""Reconfigure Request: {THROTTLE_FORWARD_PWM} {THROTTLE_STOPPED_PWM} {THROTTLE_REVERSE_PWM}""".format(**received_config))

    # only one at the time will change and we need to publish it
    # TODO: For calibration of backwords, we need to publish 3 times in specific order
    # TODO: We still need to clarify the order.
    if CONFIG["THROTTLE_FORWARD_PWM"] != received_config["THROTTLE_FORWARD_PWM"]:
        CONFIG["THROTTLE_FORWARD_PWM"] = received_config["THROTTLE_FORWARD_PWM"]
        PUBLISHER.publish(received_config["THROTTLE_FORWARD_PWM"])

    elif CONFIG["THROTTLE_REVERSE_PWM"] != received_config["THROTTLE_REVERSE_PWM"]:
        CONFIG["THROTTLE_REVERSE_PWM"] = received_config["THROTTLE_REVERSE_PWM"]
        PUBLISHER.publish(received_config["THROTTLE_REVERSE_PWM"])

    elif CONFIG["THROTTLE_STOPPED_PWM"] != received_config["THROTTLE_STOPPED_PWM"]:
        CONFIG["THROTTLE_STOPPED_PWM"] = received_config["THROTTLE_STOPPED_PWM"]
        PUBLISHER.publish(received_config["THROTTLE_STOPPED_PWM"])

    dump_data()

    return received_config


def main():
    set_variables()
    rospy.init_node('config_throttle')
    srv = Server(ThrottleConfig, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
