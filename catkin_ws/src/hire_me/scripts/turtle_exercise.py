#!/usr/bin/env python


# import necesary modules
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from threading import Thread


# CONSTANTS
DIST_TOLERANCE = 2.0  # for stopping bot when it reaches the desired zone


# just an infinite loop to keep program running
def mainloop():
    while True:
        pass


class TurtleBot:
    def __init__(self):
        # Creates a node with name 'turtle_exercise' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('turtle_exercise', anonymous=True)
        # to publish to turtle1 to move
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # subscribe to topic for getting own position, calls function to update own position
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        # subscribe to topic to acquire target position, calls function to move towards goal
        self.target_subscriber = rospy.Subscriber('/abhiyaan/pose', Pose, self.move2goal)
        # pose object for own position
        self.pose = Pose()
        self.rate = rospy.Rate(10)
    
    # function to update own postion
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # compute euclidean distance between self and target
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    # function to move towards target
    def move2goal(self, data):
        self.target_subscriber.unregister()
        goal_pose = Pose()

        goal_pose.x = round(data.x, 4)
        goal_pose.y = round(data.y, 4)

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= DIST_TOLERANCE:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

        # stop when bot reaches desired zone
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot()
        t = Thread(target=mainloop)  # thread to keep program running
        t.daemon = True
        t.start()
        t.join()
    except Exception as e:
        print("ERROR : ", e)
        pass
