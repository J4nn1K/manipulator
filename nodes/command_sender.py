#!/usr/bin/env python
import rospy
import pigpio
from std_msgs.msg import String

# PWM Neutral Signal: 1500
# PWM Open Signal: 1530 - 1900
# PWM Close Signal: 1470 - 1100 

PIN = 18

class CommandSender():
    def __init__(self):
        rospy.init_node("command_sender")

        self.manipulator_state = "neutral"

        self.key_sub = rospy.Subscriber("manipulator", String, self.manipulator_callback)
        self.pi = pigpio.pi()

    def manipulator_callback(self, msg):
        self.manipulator_state = msg.data


    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():
            if self.manipulator_state == "neutral":
                self.pi.set_servo_pulsewidth(PIN, 1500)
            
            elif self.manipulator_state == "close":
                self.pi.set_servo_pulsewidth(PIN, 1200)

            elif self.manipulator_state == "open":
                self.pi.set_servo_pulsewidth(PIN, 1800)

            rate.sleep()

def main():
    node = CommandSender()
    node.run()

if __name__ == "__main__":
    main()
