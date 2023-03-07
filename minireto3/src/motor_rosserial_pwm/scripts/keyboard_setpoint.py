import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float32

setpoint = Float32()

# Receive input from teleop_twist_keyboard package and assign data
def keyboard_input_cb(msg: Twist):
    global setpoint
    rospy.loginfo(msg.linear.x)
    if(msg.linear.x in range(-255,255)):
        setpoint.data = msg.linear.x
    



def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping Node")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("keyboard_setpoint")
    init_time = rospy.get_time()
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Declare publishers and subscribers
    cmd_vel_sub = rospy.Subscriber('cmd_vel',Twist,keyboard_input_cb)
    set_point_pub = rospy.Publisher('cmd_pwm',Float32,queue_size=10)

    print("The Set Point Genertor is Running")
    
    #Run the node
    while not rospy.is_shutdown():      
        

        # Assign data to message and publish
        set_point_pub.publish(setpoint)

        rate.sleep()