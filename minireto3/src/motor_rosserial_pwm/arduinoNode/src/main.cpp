#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// Declare Node Handle
ros::NodeHandle nh;

// H Bridge Pins
const int ENB = 2; // Enable PWM
const int IN3 = 22; 
const int IN4 = 23;


// Set direction and duty cycle of 
void input_cb(const std_msgs::Float32 & msg){
  if(abs(msg.data)>255){ // Prevents from writing values not admitted
    nh.logwarn("EXCEEDED INPUT");
  }
  else {
    if(msg.data>0){ // Positive Direction (+)
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
    } else { // Positive Direction (-)
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
    }
    // int motorSpeed = floor(255 * abs(msg.data)); // transform input from [0,1] to [0,255] 
    int motorSpeed = floor(abs(msg.data)); // 
    // Debug Info
    String data = String(motorSpeed);
    nh.loginfo(data.c_str());
    // Set Dutycycle and apply PWM
    analogWrite(ENB, motorSpeed);
  }
}

ros::Subscriber<std_msgs::Float32> input_sub("cmd_pwm", &input_cb);


void setup() {
  // pin init
  pinMode(ENB,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  analogWrite(ENB, 0);

  // Initialize ROS Node
  nh.initNode();
  nh.subscribe(input_sub);
}
void loop() {
  // ROS Node
  nh.spinOnce();
  delay(1);
}