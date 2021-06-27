#!/usr/bin/env python

import rospy
import rospkg
import os
from std_msgs.msg import String
from sensor_msgs.msg import Joy


class KeyboardControlNode():
    
    def __init__(self, name):
        rospy.init_node(name)

        self.manipulator_state = 'neutral' # 'close' / 'open'
        self.a_pressed = 0
        self.b_pressed = 0

        self.manipulator_pub = rospy.Publisher("manipulator", String, queue_size=1)
        self.gamepad_sub = rospy.Subscriber("joy", Joy, self.callback)
    
    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            self.handle_buttons()
            self.publish_message()
            rate.sleep()

    def callback(self, msg):
        self.a_pressed = msg.buttons[0]
        self.b_pressed = msg.buttons[1]
        #rospy.loginfo(a_pressed)
    
    def handle_buttons(self):
        if self.a_pressed == 1:
            self.manipulator_state = "close"
        elif self.b_pressed == 1: 
            self.manipulator_state = "open"
        else:
            self.manipulator_state = "neutral"
         

    def publish_message(self):
        msg = String()
        msg.data = self.manipulator_state
        self.manipulator_pub.publish(msg)
        rospy.loginfo("Manipulator State: %s", msg.data)

def main():
    node = KeyboardControlNode("keyboard")
    node.run()

if __name__ == "__main__":
    main()
