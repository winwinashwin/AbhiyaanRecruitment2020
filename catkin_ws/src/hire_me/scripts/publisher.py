#!/usr/bin/env python

# import necessary modules
import rospy
from std_msgs.msg import String


def talker():
    print(">>> Initiating Publisher ....")
    pub = rospy.Publisher('/welcome_message', String, queue_size=10)  # initialise publisher with topic, dtype and queue size
    rospy.init_node('talker', anonymous=True)  # initialise node and make it unique
    rate = rospy.Rate(10)  # 10Hz ie, publish message 10 times a second
    while not rospy.is_shutdown():
        message = "Welcome to Abhiyaan"
        # rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

