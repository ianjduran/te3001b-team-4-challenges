import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import numpy as np
setpoint = Float32()

# Receive input from controller and set 
def keyboard_input_cb(msg: Joy):
    global  setpoint
    vel = 0
    # Declare intervals [a,b] from controller and intervals [c,d] for output
    a = 1
    b = -1
    c = 1
    d = 255
    temp = c + (d - c)/(b - a) 
    if(msg.buttons[7] == 1): # Check if right trigger is pressed
        vel = temp * (msg.axes[5] - a) # Calc value with value sent from controller

    elif(msg.buttons[6] == 1): # Left Trigger is being pressed
        vel = temp * (msg.axes[2] -a) * -1 # Calc value with LT angle
    
    # rospy.loginfo("pwm: = " + str(np.math.floor(vel)))
    setpoint.data = np.math.floor(vel) # Assign msg data



def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping Node")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("keyboard_setpoint")
    init_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    
    # Declare Publisher and Subscriber
    cmd_vel_sub = rospy.Subscriber('joy',Joy,keyboard_input_cb)
    set_point_pub = rospy.Publisher('cmd_pwm',Float32,queue_size=10)

    print("The Set Point Genertor is Running")
    
    #Run the node
    while not rospy.is_shutdown():      
        

        # Assign data to message and publish
        set_point_pub.publish(setpoint)
        rate.sleep()