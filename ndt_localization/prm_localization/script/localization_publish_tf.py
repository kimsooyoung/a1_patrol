#! /usr/bin/python2
import rospy 
import tf 
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry 


if __name__ == '__main__':
    rospy.init_node('localization_tf')
    tf_pub = rospy.Publisher('map_base_tf', Odometry, queue_size=10)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)
    seq = 0
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # (trans, rot) = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        odom_msg = Odometry()
        odom_msg.header.seq = seq
        odom_msg.header.stamp = rospy.Time().now()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'velodyne'
        odom_msg.pose.pose.position.x = trans[0]
        odom_msg.pose.pose.position.y = trans[1]
        odom_msg.pose.pose.position.z = trans[2]
        odom_msg.pose.pose.orientation.x = rot[0]
        odom_msg.pose.pose.orientation.y = rot[1]
        odom_msg.pose.pose.orientation.z = rot[2]
        odom_msg.pose.pose.orientation.w = rot[3]

        tf_pub.publish(odom_msg)
        rospy.loginfo('Publishing map_base_tf during localization ...')
        seq += 1
        rate.sleep()


