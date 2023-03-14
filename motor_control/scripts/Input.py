#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import Float32

def talker():
    signal_pub = rospy.Publisher('setpoint', Float32, queue_size=10)
    rospy.init_node('signal_generator', anonymous=False)
    ti = rospy.get_time()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        t = rospy.get_time() - ti
        fun = np.sin(t)
        word = "time: %s" % t + "sine value: %s" % fun
        rospy.loginfo(word)
        signal_pub.publish(fun)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
