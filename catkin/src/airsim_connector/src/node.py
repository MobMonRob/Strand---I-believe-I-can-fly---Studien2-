#!/usr/bin/env python

import rospy
from drone_controller import DroneController
from communication_manager import CommunicationManager


def init_node():
    def on_shutdown():
        try:
            drone_controller.land()
        except:
            rospy.loginfo('Drone was not initialized at shutdown!')

    rospy.init_node('airsim_connector')
    rospy.on_shutdown(on_shutdown)
    drone_controller = DroneController()
    communication_manager = CommunicationManager(drone_controller)
    communication_manager.start_listening()
    while not rospy.is_shutdown():
        drone_controller.perform_action()


if __name__ == '__main__':
    init_node()
