#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np
from motor_control.msg import set_point

def talker():
    signal_pub = rospy.Publisher('set_point', set_point, queue_size=10)
    rospy.init_node('signal_generator', anonymous=False)
    ti = rospy.get_time()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        t = rospy.get_time() - ti
        y = np.sin(t)
        
        msg = set_point()
        msg.value = y
        msg.time = t
        rospy.loginfo(y)
        signal_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
