#!/usr/bin/env python

import rospy
from drone_controller import DroneController
from communication_manager import CommunicationManager
from i_believe_i_can_fly_common.msg import Reset as ResetMsg


def init_node():
    """
    Initializes ROS node 'i_believe_i_can_fly_airsim_connector'.
    """

    def on_shutdown():
        """
        Lands the drone when ROS node is shut down. Do NOT wait for it's execution to shutdown node immediately.
        """
        try:
            rospy.loginfo('Release API control of drone as node is shutdown!')
            drone_controller.release_control()
        except:
            rospy.loginfo('Drone was not initialized at shutdown!')
            rospy.signal_shutdown('Shutdown i_believe_i_can_fly_airsim_connector node')

    def on_reset(reset_msg):
        """
        Resets state of this RPS node.
        """
        drone_controller.reset_position()
        drone_controller.print_message('Reset drone', 1)

    rospy.init_node('i_believe_i_can_fly_airsim_connector',
                    log_level = (rospy.DEBUG if rospy.get_param('/i_believe_i_can_fly_airsim_connector/debug') else rospy.ERROR))
    rospy.on_shutdown(on_shutdown)
    rospy.Subscriber('i_believe_i_can_fly', ResetMsg, on_reset)
    drone_controller = DroneController()
    communication_manager = CommunicationManager(drone_controller)
    communication_manager.start_listening()
    while not rospy.is_shutdown():
        drone_controller.perform_action()


if __name__ == '__main__':
    init_node()
