#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)



#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Set_Point_Generator")
    init_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    set_point_data = set_point()


    #Setup Publishers and subscribers here
    set_point_pub = rospy.Publisher('set_point',set_point,queue_size=10)


    print("The Set Point Genertor is Running")

    #Run the node
    while not rospy.is_shutdown():
        
        current_time = rospy.get_time()-init_time
        
        offset = rospy.get_param('setpoint/offset',default=0)
        amplitude = rospy.get_param('setpoint/amplitude',default=1)
        
        # Assign data to message and publish
        set_point_data.time = rospy.get_time()-init_time
        set_point_data.setPoint =  amplitude * np.sin(current_time) + offset
        set_point_pub.publish(set_point_data)

        rate.sleep()