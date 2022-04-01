#!/usr/bin/env python3
import rospy


class Mapper:
    def __init__(self):
        pass

def startNode():
    c = Mapper()
    rospy.spin()

if __name__ == "__main__":
    ns = rospy.get_namespace()
    try:
        rospy.init_node('mapper')
        rate = rospy.Rate(100)
        startNode()
    except rospy.ROSInterruptException as e:
        print(e)