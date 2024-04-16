#! /usr/bin/env python3
"""This script is used to control the paddle in Gazebo using arrow keys."""

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
from pynput.keyboard import Key


class KeyDrive():
    """Class to move the paddle using keyboard laterally."""
    def __init__(self):

        update_rate = 50
        freq = 1. / update_rate
        self.speed = 20  # m/s

        # Publishers
        self.cmd_pub = rospy.Publisher('/paddle/cmd_vel', Twist, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(freq), self.keyboard_update)

    def move_left(self):
        """Move the paddle to the left."""
        cmd = Twist()
        cmd.linear.x = -self.speed
        self.cmd_pub.publish(cmd)

    def move_right(self):
        """Move the paddle to the right."""
        cmd = Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

    def halt(self):
        """Stop the paddle."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def key_press(self, key):
        """Listen for key press."""
        if key == Key.left:
            self.move_left()
        elif key == Key.right:
            self.move_right()
        return False

    def key_release(self, _):
        """Listen for key release."""
        self.halt()
        return False

    def keyboard_update(self, _):
        """Keyboard Listener for a press and release."""
        with keyboard.Listener(on_press=self.key_press) as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) as listener_for_key_release:
            listener_for_key_release.join()

    def kill_node(self):
        """Function to kill the ROS node."""
        rospy.signal_shutdown("Done")


if __name__ == '__main__':

    rospy.init_node('paddle_play_node', anonymous=True)
    KeyDrive()
    rospy.spin()
