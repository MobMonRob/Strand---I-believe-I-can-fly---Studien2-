#!/usr/bin/env python

import airsim
import time
import rospy


class DroneController:
    def __init__(self):
        try:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.update_rate = 1
            self.flying = False
            self.next_action = None
        except:
            rospy.logerr('Could not connect to running AirSim session! Is AirSim really running?')
            rospy.signal_shutdown('AirSim not running!')

    def takeoff(self):
        self.client.enableApiControl(True)
        self.client.takeoffAsync(5).join()
        self.flying = True

    def land(self):
        self.client.landAsync(5).join()
        self.client.enableApiControl(False)
        self.flying = False

    def forward(self):
        self.set_next_action(self.client.moveByAngleThrottleAsync, -0.3, 0, 0.7, 0, self.update_rate)

    def hold(self):
        self.set_next_action(self.client.hoverAsync)

    def left(self):
        self.set_next_action(self.client.moveByAngleThrottleAsync, -0.1, -0.3, 0.75, -0.3, self.update_rate)

    def right(self):
        self.set_next_action(self.client.moveByAngleThrottleAsync, -0.1, 0.3, 0.75, 0.3, self.update_rate)

    def throttle_up(self):
        self.reset_next_action()
        if not self.flying:
            self.takeoff()
        else:
            self.set_next_action(self.client.moveToZAsync, self.client.getPosition().z_val - 100, 5, self.update_rate)

    def throttle_down(self):
        self.set_next_action(self.client.moveToZAsync, self.client.getPosition().z_val + 200, 5, self.update_rate)

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
        else:
            rospy.logwarn('No upcoming action specified! Waiting for new user input.')
        time.sleep(self.update_rate)
