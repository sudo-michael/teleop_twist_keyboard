#!/usr/bin/env python

from __future__ import print_function
import tty
import termios
import select
import sys
import rospy

import threading

import math
from jetracer.msg import jetRacerCar as JetRacerCarMsg


BREAK_KEY = ' '
msg = """
Reading from the keyboard  and Publishing to /jetRacer_Controller!
---------------------------
Increase throttle by 1: w
Decrease throttle by -1: s

Increase steering angle left by pi/96: a 
Decrease steering angle right by pi/96: d 

Break: space

Fine tune steering angle
Increase steering angle left by pi/192: A 
Decrease steering angle right by pi/192: D 

CTRL-C to quit
"""

throttleBindings = {
    'w': 1,
    's': -1,
}

steeringBindings = {
    'a': -math.pi / 96,
    'd': math.pi / 96,
    'A': math.pi / 192,
    'D': -math.pi / 192,
    'z': 0.0,
    'x': 0.0,
    'c': 0.0
}

KEYBOARD_CONTROL_TOPIC = "/jetracer/keyboard"
CAR_CONTROL_TOPIC = '/jetRacer_Controller'
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher(KEYBOARD_CONTROL_TOPIC, JetRacerCarMsg, queue_size=1)
        self.throttle = 0.0
        self.steerAngle = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(
                    self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception(
                "Got shutdown request before subscribers connected")

    def update(self, throttle, steerAngle):
        self.condition.acquire()
        self.throttle = throttle
        self.steerAngle = steerAngle
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0)
        self.join()

    def run(self):
        msg = JetRacerCarMsg()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            msg.steerAngle = self.steerAngle
            msg.throttle = self.throttle

            self.condition.release()

            # Publish.
            self.publisher.publish(msg)

        # Publish stop message when thread exits.
        msg.steerAngle = 0
        msg.throttle = 0
        self.publisher.publish(msg)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(throttle, steerAngle):
    return "currently:\tthrottle %s\tsteerAngle %s " % (throttle, steerAngle)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    throttle = rospy.get_param("~throttle", 0.0)
    steerAngle = rospy.get_param("~steerAngle", 0.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    status = 0
    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(throttle, steerAngle)

        print(msg)
        print(vels(throttle, steerAngle))
        while(1):
            key = getKey(key_timeout)
            if key == BREAK_KEY:
                print("BREAK")
                throttle = 0.0
                steerAngle = 0.0
            elif key in throttleBindings.keys():
                throttle += throttleBindings[key]
                throttle = min(throttle, 100)
                # prevent car from going backwards
                throttle = max(throttle, 0)

                print(vels(throttle, steerAngle))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

            elif key in steeringBindings.keys():
                steerAngle += steeringBindings[key]
                if key == 'z':
                    steerAngle = -math.pi/12
                elif key == 'x':
                    steerAngle = 0.0
                elif key == 'c':
                    steerAngle = math.pi/12

                # keep steerAngle in range
                steerAngle = max(steerAngle, -math.pi/12)
                steerAngle = min(steerAngle, math.pi/12)

                print(vels(throttle, steerAngle))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                '''
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                '''
                if (key == '\x03'):
                    pub_thread.stop()
                    break

            pub_thread.update(throttle, steerAngle)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
