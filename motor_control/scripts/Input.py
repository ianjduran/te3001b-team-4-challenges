#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np
import math
from motor_control.msg import set_point, signal_generator_flag

# Define different signal gen methods
def square_wave_gen(t, period):
    # Changes from 0 to 1 every period
    return (math.floor(t / period) % 2 - 0.5) * 2

def sine_wave_gen(t, _):
    return np.sin(t)

# Signal generator task, takes in signal_generator_type, where the current signal gen method is determined
# Publishes set_point with new setpoints
class SignalGenerator():
    def __init__(self):
        rospy.Subscriber("signal_generator_type", signal_generator_flag, self.flag_cb)
        self.signal_pub = rospy.Publisher('set_point', set_point, queue_size=10)
        self.selected_method = sine_wave_gen # Default is sine wave gen
        self.period = rospy.get_param("period", 2)
        self.magnitude = rospy.get_param("magnitude", 15)

    def update(self):
        # Calculate with selected method
        x = rospy.get_time()
        y = self.selected_method(x, self.period) * self.magnitude

        # Create message and publish it
        msg = set_point()
        msg.value = y
        msg.time = x
        self.signal_pub.publish(msg)

    def flag_cb(self, msg):
        method = msg.type

        # Change selected method based off the constants in the flag message
        if method == signal_generator_flag.SQUARE_WAVE:
            self.selected_method = square_wave_gen
        elif method == signal_generator_flag.SINE_WAVE:
            self.selected_method = sine_wave_gen
        elif method == signal_generator_flag.STOP:
            self.selected_method = lambda _: 0
        else:
            rospy.logerr("INVALID SIGNAL GEN")

if __name__ == '__main__':
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)
    
    generator = SignalGenerator()

    while not rospy.is_shutdown():
        generator.update()
        rate.sleep()