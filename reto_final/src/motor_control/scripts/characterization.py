#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class Characterization:
    def __init__(self):
        self.start_time = rospy.get_time()
        
        self.step_start_time = 5
        self.step_duration = 5
        self.step_mag = 0.5

        self.current_velocity = 0
        self.current_setpoint = 0

        self.motor_data = {'time': [], 'velocity': [], 'setpoint': []}
        rospy.Subscriber("motor_output", Float32, self.motor_out_cb)
        self.motor_pub = rospy.Publisher("motor_input", Float32, queue_size=1)
        
    def motor_out_cb(self, msg):
        # Write motor velocity to csv
        self.current_velocity = msg.data

    def write_motor(self, val):
        msg = Float32()
        msg.data = val
        self.motor_pub.publish(msg)
        self.current_setpoint = val

    def close(self):
        # TODO Handle writing motor_data to csv file
        rospy.loginfo("Exiting from node")

        import csv
        with open('/tmp/motor_data.csv', 'w', newline='') as file:
            csv_writer = csv.DictWriter(file, delimiter=',', fieldnames=['time', 'velocity', 'setpoint'])
            csv_writer.writeheader()
            for vel, setpoint, time in zip(self.motor_data['velocity'], self.motor_data['setpoint'], self.motor_data['time']):
                csv_writer.writerow({'velocity': vel, 'setpoint': setpoint, 'time': time})

            rospy.loginfo("Saved to motor_data.csv! :)")

        exit()

    def update(self):
        # Constant sample time ish
        self.motor_data['time'].append(rospy.get_time())
        self.motor_data['velocity'].append(self.current_velocity)
        self.motor_data['setpoint'].append(self.current_setpoint)

        if rospy.get_time() - self.start_time > self.step_start_time:
            # Begin step
            self.write_motor(self.step_mag)
        if rospy.get_time() - self.start_time > (self.step_start_time + self.step_duration):
            # End step :)
            self.write_motor(0)
            self.close()

if __name__ == '__main__':
    rospy.init_node("characterization_node")

    characterization = Characterization()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        characterization.update()
        rate.sleep()