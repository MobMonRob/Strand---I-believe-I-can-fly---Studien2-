#!/usr/bin/env python

import airsim
import rospy
import time


class DroneController:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.update_rate = 1
        self.flying = False
        self.next_action = None

    def takeoff(self):
        rospy.loginfo('[DRONE] Performing takeoff...')
        self.client.enableApiControl(True)
        self.client.takeoffAsync(5).join()
        self.flying = True
        rospy.loginfo('[DRONE] Takeoff finished')

    def land(self):
        rospy.loginfo('[DRONE] Performing landing...')
        self.client.landAsync(5).join()
        self.client.enableApiControl(False)
        self.flying = False
        rospy.loginfo('[DRONE] Landing finished')

    def forward(self):
        rospy.loginfo('[DRONE] Forward')
        self.set_next_action(self.client.moveByAngleThrottleAsync, 20, 0, 10, 0, self.update_rate)
        rospy.loginfo('[DRONE] Forward finished')

    def hold(self):
        self.set_next_action(self.client.hoverAsync)
        rospy.loginfo('[DRONE] Hold')

    def left(self):
        rospy.loginfo('[DRONE] Left')

    def right(self):
        rospy.loginfo('[DRONE] Right')

    def throttle_up(self):
        rospy.loginfo('[DRONE] Throttle Up')
        self.reset_next_action()
        if not self.flying:
            self.takeoff()

    def throttle_down(self):
        rospy.loginfo('[DRONE] Throttle down')

    def set_next_action(self, function, *args):
        self.next_action = [function, args]

    def reset_next_action(self):
        self.next_action = None

    def perform_action(self):
        if self.next_action is not None:
            if self.next_action[1] is not None:
                self.next_action[0](*self.next_action[1])
            else:
                self.next_action[0]()
            time.sleep(self.update_rate)
