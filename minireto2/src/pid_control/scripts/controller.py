#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input

#set point msg
from pid_control.msg import set_point


#Setup parameters, vriables and callback functions here (if required)

class Controller:
    def __init__(self, dt) -> None:
      self.kp = rospy.get_param('controller_kp', default=0)
      self.ki = rospy.get_param('controller_ki', default=0)
      self.kd = rospy.get_param('controller_kd', default=0)
      self.dt = dt
      self.u_min = -15
      self.u_max = 15
      self.error = 0.0
      self.u_val = 0.0 # Control Force
      self.Error_prev = 0.0
      self.Error_Int = 0.0
      self.estimated_output = 0.0
      self.output_status = "Not Turning"
      self.output_timestamp = 0.0
      self.setPoint = 0.0
      self.setPoint_time = 0.0

      #Defining Publishers and Subscribers
      self.set_point_sub = rospy.Subscriber('/set_point', set_point, self.setpoint_data_cb)
      self.motor_estimated_output_sub = rospy.Subscriber('/motor_output', motor_output, self.motor_estimated_output_cb)
      self.motor_input_pub = rospy.Publisher('/motor_input', motor_input, queue_size=1)

      self.input_msg = motor_input()

    def setpoint_data_cb(self, msg):
      self.setPoint = msg.setPoint
      self.setPoint_time = msg.time
   
    
    def motor_estimated_output_cb(self, msg):
      self.estimated_output = msg.output
      self.output_status = msg.status
      self.output_timestamp = msg.time

    def control(self):

      self.error = self.setPoint - self.estimated_output
      self.Error_Int+= self.error * self.dt

      self.u_val = self.kp* self.error + self.ki * self.Error_Int + self.kd * (self.error - self.Error_prev)/self.dt

      self.Error_prev = self.error

      self.input_msg.input = self.u_val

      self.input_msg.time = self.output_timestamp

      self.motor_input_pub.publish(self.input_msg)

 
#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    controller = Controller(dt=1.0/5000.0)
    

    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():
        
        controller.control()

        rate.sleep()