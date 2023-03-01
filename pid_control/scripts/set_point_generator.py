#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point as sp
from std_msgs.msg import Float32

# Setup Variables, parameters and messages to be used (if required)
#PONER EL ARCHIVO DE PARAMETROS PARA QUE PUEDA
amplitude = rospy.get_param("/set_point_value",0.0)
#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


def signal_gen():
  
  ti = rospy.get_time()
  rate = rospy.Rate(10) # 10hz
  dato = sp()
  while not rospy.is_shutdown():
      t = rospy.get_time()-ti
      fun = np.sin(t)*amplitude
      #word = "time: %s" % t + "sine value: %s" % fun
      dato.value = fun
      dato.time = t
      #rospy.loginfo(dato)
      signal_pub.publish(dato)


if __name__=='__main__':
  #Initialise and Setup node
  rospy.init_node("SetPoint")
  rate = rospy.Rate(10)
  rospy.on_shutdown(stop)

  #Setup Publishers and subscribers here
  signal_pub = rospy.Publisher('set_point', sp, queue_size=10)
  print("The Set Point Genertor is Running")

  #Run the node
  while not rospy.is_shutdown():
    #Write your code here
    signal_gen()
    rate.sleep()