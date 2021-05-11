#!/usr/bin/env python


import rospy



class param_reader:
    def __init__(self):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('param_reader')
        rospy.loginfo("String Param Reader Node!")
        reader = param_reader()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass