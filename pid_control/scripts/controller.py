#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#variables de PID
"""class Controller:
  def __init__(self):
    #PID variables?

    
    #self.kp = self.ko * 0.7
    #self.ki = (self.ko/self.t1) *0.03
    #self.kd = self.ko*self.t1 * 0.005

    self.error = 0.0
    self.Error_prev = 0.0
    self.Error_Int = 0.0
    self.u_val = 0.0
    self.u_time = 0.0
    
    #FORMULA PARA SACAR LOS ERRORES
    self.error = self.motor_input - self.motor_output #error proporcional
    self.Error_Int += self.error*dt #error integral
    self.Error_Der = (self.error-self.Error_prev)/dt
    
    
    self.angularVelocity = 0.0 #?
    #self.setPoint = 0.0 #?
    self.u_val  = self.error*self.kp + self.Error_Int*self.ki + self.kd*self.Error_Der

    def run_controller(self):
       pass
"""

'''rospy.loginfo(rospy.get_caller_id() + " I heard IN CONTROLLER %s, time: %s", valueSetpoint, timeSetpoint)
msg = motor_input()
msg.input=valueSetpoint
msg.time=timeSetpoint
if(timeSetpoint > 20):
   msg.input = 1
#rospy.loginfo(msg)
pubMotorInput.publish(msg)
'''

kp = rospy.get_param("/control_kp",0.0)
ki = rospy.get_param("/control_ki",0.0)
kd = rospy.get_param("/control_kd",0.0)
dt = rospy.get_param("/control_dt",0.001)
sin_signal = motor_input()
#variables del motor OUTPUT
output_motor = 0
output_motor_time = 0

#variables del setpoint
valueSetpoint = 0.0
timeSetpoint = 0.0


#variables de los errores
errorIntegral = 0.0
errorP = 0.0
u_val = 0.0

#Setup parameters, vriables and callback functions here (if required)

def callback_SetPoint(data):
   global valueSetpoint, timeSetpoint 
   valueSetpoint = data.value
   timeSetpoint = data.time
   #sin_signal.input = valueSetpoint
   #pubMotorInput.publish(sin_signal)
   #rospy.loginfo(rospy.get_caller_id() + " I heard IN cb CONTROLLER %s, time: %s", data.value, data.time)


def callback_motorOut(data):
   #rospy.loginfo(rospy.get_caller_id() + " I heard %s, time: %s", data.output, data.time)
   global output_motor, output_motor_time 
   #print(output_motor, output_motor_time)
   #print(valueSetpoint, timeSetpoint)
   output_motor = data.output
   output_motor_time = data.time

def controller_out():
   global errorIntegral, errorP,u_val, output_motor, valueSetpoint
   errorP = valueSetpoint-output_motor 
   errorIntegral += errorP*dt
   u_val  = errorP*kp + errorIntegral*ki
   sin_signal.input = u_val
   pubMotorInput.publish(sin_signal)
   #rospy.loginfo(rospy.get_caller_id() + " I heard %s", sin_signal.input)


def no_controller():
   #global valueSetpoint, out
   #error = valueSetpoint - 
   pass

   
if __name__=='__main__':
   #Initialise and Setup node
   
   #Setup Publishers and subscribers here
   #Initiate the node called controller, and create de pub and subs
   rospy.init_node("Controller")
   rospy.Subscriber('motor_output', motor_output, callback_motorOut) #hace falta funcion de callback
   pubMotorInput = rospy.Publisher('motor_input', motor_input, queue_size=10)
   rospy.Subscriber('set_point', set_point, callback_SetPoint)
   print("The Controller is Running")
   rate = rospy.Rate(100) 

   while not rospy.is_shutdown(): 
      
      controller_out()
      rate.sleep()
    
        