#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

#Global variable to store the data from the message in the /signal topic
signal_data = 0
current_time = 0
phase_shift_buffer = []

# Example Callback Function (Hint)
def signal_callback(msg):
    global signal_data
    out = msg.data + 1.0
    out /= 2

    phase_shift_ammount = 20
    phase_shift_buffer.append(out)
    if len(phase_shift_buffer) == phase_shift_ammount:
        signal_data = phase_shift_buffer.pop(0)
    else:
        signal_data = 0

def time_callback(msg):
    global current_time
    current_time = msg.data

if __name__=='__main__':
    
    #Finish configuring your node here (part of the code has already been written as a hint)
    rospy.init_node("process")
    signal_publisher = rospy.Publisher("/proc_signal", Float32, queue_size=10)
    rospy.Subscriber("/signal", Float32, signal_callback)
    rospy.Subscriber("/time", Float32, time_callback)
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():

        #Write your code here
        signal_publisher.publish(signal_data)
        rate.sleep()