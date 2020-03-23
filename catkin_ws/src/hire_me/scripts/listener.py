#!/usr/bin/env python

# import necessary libraries
import rospy
from std_msgs.msg import String


# function to print received message to screen
def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + " " + data.data)
    print(data.data)


def listener():
    print(">>> Listening to topic : \"/welcome_message\"")
    rospy.init_node('listener', anonymous=True)  # initialise node 'listener as unique'
    rospy.Subscriber('/welcome_message', String, callback)  # instantise subscriber, to topic '/welcome_message', dtype: string, calls function callback whenever it receives a message
    rospy.spin()  # wait till user terminates the execution


if __name__ == "__main__":
    listener()

