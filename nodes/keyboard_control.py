#!/usr/bin/env python

import rospy
import pygame
import rospkg
import os
from std_msgs.msg import String


class KeyboardControlNode():
    WINDOW_SIZE = (640, 200)
    DISPLAY_FLAGS = pygame.DOUBLEBUF

    def __init__(self, name):
        pygame.init()
        pygame.mixer.quit()
        rospy.init_node(name)

        self.manipulator_state = 'neutral' # 'close' / 'open'

        # create GUI
        self.screen = self.init_display()

        self.controls = self.init_controls()

        self.manipulator_pub = rospy.Publisher("manipulator",
                                            String,
                                            queue_size=1)
    
    def init_display(self):
        screen = pygame.display.set_mode(self.WINDOW_SIZE, self.DISPLAY_FLAGS)
        #vehicle_name = rospy.get_namespace().strip("/")
        pygame.display.set_caption("Manipulator Control")

        #icon_path = os.path.join(self.get_resource_path(), "icon.png")
        #icon = pygame.image.load(icon_path)
        #pygame.display.set_icon(icon)
        return screen
    
    def run(self):
        rate = rospy.Rate(30.0)

        while not rospy.is_shutdown():

            self.handle_events()
            self.screen.fill((0, 0, 0))
            #self.update_text_grid()
            pygame.display.flip()
            self.publish_message()
            
            rate.sleep()

    def set_manipulator_state(self, value):
        self.manipulator_state = value

    def init_controls(self):
        controls = {
            pygame.K_f:
            dict(
                pressed=False,
                changed=False,
                description="close",
                pressed_callback=(lambda: self.set_manipulator_state("close")),
                released_callback=(lambda: self.set_manipulator_state("neutral")),
            ),
            pygame.K_r:
            dict(
                pressed=False,
                changed=False,
                description="open",
                pressed_callback=(lambda: self.set_manipulator_state("open")),
                released_callback=(lambda: self.set_manipulator_state("neutral")),
            ),
        }
        return controls

    def handle_events(self):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key in self.controls:
                    control = self.controls[event.key]
                    if "pressed_callback" in control:
                        control["pressed_callback"]()
            elif event.type == pygame.KEYUP:
                if event.key in self.controls:
                    control = self.controls[event.key]
                    if "released_callback" in control:
                        control["released_callback"]()

            elif event.type == pygame.QUIT:
                pygame.quit()
                rospy.signal_shutdown("Quitting")

    def publish_message(self):
        msg = String()
        #msg.header.stamp = rospy.Time.now()
        msg.data = self.manipulator_state
        self.manipulator_pub.publish(msg)
        rospy.loginfo("Manipulator State: %s", msg.data)

def main():
    node = KeyboardControlNode("keyboard")
    node.run()

if __name__ == "__main__":
    main()
