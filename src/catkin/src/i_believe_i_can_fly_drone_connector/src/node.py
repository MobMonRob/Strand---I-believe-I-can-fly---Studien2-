#!/usr/bin/env python

import rospy


def init_node():
    """
    Initializes ROS node 'i_believe_i_can_fly_drone_connector'.
    """

    rospy.init_node('i_believe_i_can_fly_drone_connector', log_level = (
        rospy.DEBUG if rospy.get_param('/i_believe_i_can_fly_drone_connector/debug') else rospy.ERROR))
    print("Launched Drone Connector!")
    rospy.spin()

if __name__ == '__main__':
    init_node()
