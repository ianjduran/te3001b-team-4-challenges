#!/usr/bin/env python3
import rospy
import numpy as np
from motor_control.msg import set_point
from std_msgs.msg import Float32

class Controller:
  def __init__(self, dt):
    self.Kp = rospy.get_param("/control_kp", 0.0)
    self.Ki = rospy.get_param("/control_ki", 0.0)
    self.Kd = rospy.get_param("/control_kd", 0.0)
    self.K = rospy.get_param('/control_K', 1.0)

    rospy.Subscriber("/motor_output", Float32, self.motor_output_cb)
    rospy.Subscriber("/set_point", set_point, self.setpoint_cb)

    self.motor_pub = rospy.Publisher("/motor_input", Float32, queue_size=1)
    self.motor_error = rospy.Publisher("/error", Float32, queue_size=1)
    self.controller_p = rospy.Publisher("/motor_controller/p", Float32, queue_size=10)
    self.controller_i = rospy.Publisher("/motor_controller/i", Float32, queue_size=10)
    self.controller_d = rospy.Publisher("/motor_controller/d", Float32, queue_size=10)

    self.current_setpoint = 0
    self.current_motor_output = 0
    self.prev_error = 0
    self.error_integral = 0
    self.dt = dt

  def setpoint_cb(self, msg):
    self.current_setpoint = msg.value

  def motor_output_cb(self, msg):
    self.current_motor_output = msg.data

  def set_motor(self, out):
    msg = Float32()
    # msg.time = rospy.get_time()
    msg.data = out
    self.motor_pub.publish(msg)

  def update(self):
    error = self.current_setpoint - self.current_motor_output
    error_msg = Float32()
    error_msg.data = error
    self.motor_error.publish(error_msg)

    p = error * self.Kp
    self.error_integral += error * self.dt
    i = self.Ki * self.error_integral
    d = (error - self.prev_error) / self.dt
    d = self.Kd * d

    self.controller_p.publish(p)
    self.controller_i.publish(i)
    self.controller_d.publish(d)

    output = self.K * (p + i + d)
    self.set_motor(output)

    self.prev_error = error

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
    controller = Controller(1.0/100.0)

    print("The Controller is Running")
    #Run the node
    while not rospy.is_shutdown():
        #Write your code here
        controller.update()
        rate.sleep()