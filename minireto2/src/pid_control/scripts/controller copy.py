#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input

#set point msg
from pid_control.msg import set_point


#Setup parameters, vriables and callback functions here (if required)

kp = rospy.get_param('controller_kp', default=0)
ki = rospy.get_param('controller_ki', default=0)
kd = rospy.get_param('controller_kd', default=0)
u_min = -15
u_max = 15
error = 0.0
Error_prev = 0.0
Error_Int = 0.0
u_time = 0.0
angularVelocity = 0.0
setPoint = 0

motor_data = motor_input()

estimated_output = 0.0
output_status = "Not Turning"

def point_data_cb(msg):
  global setPoint
  global u_time
  setPoint = msg.setPoint
  u_time = msg.time
  

def motor_estimated_output_cb(msg):
  global estimated_output
  global output_status
  estimated_output = msg.output
  output_status = msg.status
  
#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

def control(dt):
   global error, Error_prev, Error_Int, estimated_output, setPoint, kp, ki, kd

   error = setPoint - estimated_output
   Error_Int+= error*dt

   u_val = kp*error + ki * Error_Int + kd * (error - Error_prev)/dt
   
   Error_prev = error

   return u_val 


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    set_point_sub = rospy.Subscriber('/set_point', set_point, point_data_cb)
    motor_estimated_output_sub = rospy.Subscriber('/motor_output', motor_output, motor_estimated_output_cb)
    motor_input_pub = rospy.Publisher('/motor_input', motor_input, queue_size=1)
    

    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():
        #Write your code here
        
        motor_data.input = control(1.0/5000.0)
          # Set Msg Data
        motor_data.time = u_time

        #Publish
        motor_input_pub.publish(motor_data)

        rate.sleep()