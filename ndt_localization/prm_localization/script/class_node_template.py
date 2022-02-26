#!/usr/bin/python2.7
# import roslib
import rospy
import sys


# Ros Messages


class lanenet_processer:
    # member

    def __init__(self):
        # self.picProcesser = rospy.Subscriber("/imagetopic", CompressedImage, callback=self.callback, queue_size=10)
        self.picProcesser = rospy.Subscriber("/camera/image_color", Image, callback=self.callback,
                                             queue_size=10)  # set topic name
        self.result_puber = rospy.Publisher("/lanenet_result", Image, queue_size=10)  # output topic
        # your class-rate-global varab
        self.bridge = 1

    # def init_lanenet(self):
    #         #do something

    def callback(self, image_msg):
        """Callback function of subscribed topic.
        Here images get converted and features detected"""
        rospy.loginfo("callback once")


def main(args):
    """Initializes and cleanup ros node"""
    ic = lanenet_processer()
    rospy.init_node('template_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print
        "Shutting down ROS node"


if __name__ == '__main__':
    main(sys.argv)
