#!/usr/bin/env python

import rospy
from pose_detection.msg import Command


class CommunicationManager:
    def __init__(self, drone_controller):
        self.drone = drone_controller

    def start_listening(self):
        rospy.Subscriber('pose_detection', Command, self.receive_message)

    def receive_message(self, message):
        if message.command == 'HOLD':
            self.drone.hold()
        elif message.command == 'FORWARD':
            self.drone.forward()
        elif message.command == 'TURN_LEFT':
            self.drone.left()
        elif message.command == 'TURN_RIGHT':
            self.drone.right()
        elif message.command == 'THROTTLE_UP':
            self.drone.throttle_up()
        elif message.command == 'THROTTLE_DOWN':
            self.drone.throttle_down()
        else:
            rospy.logwarn('Unknown command!')
