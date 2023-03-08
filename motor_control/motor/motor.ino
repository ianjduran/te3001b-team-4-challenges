#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

const int ENB = 2;
const int IN3 = 22;
const int IN4 = 23;

void input_cb(const std_msgs::Float32 & msg){
  if(abs(msg.data)>1){
    nh.logwarn("EXCEEDED INPUT");
  }
  else {
    if(msg.data>0){
      digitalWrite(IN3,HIGH);
      digitalWrite(IN4,LOW);
    } else {
      digitalWrite(IN3,LOW);
      digitalWrite(IN4,HIGH);
    }
    int motorSpeed = floor(255 * abs(msg.data));
    String data = String(motorSpeed);
    nh.loginfo(data.c_str());
    
    analogWrite(ENB, motorSpeed);
  }
}

ros::Subscriber<std_msgs::Float32> input_sub("setpoint", &input_cb);


void setup() {
  // put your setup code here, to run once:
  pinMode(ENB,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  analogWrite(ENB, 0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);

  // Initialize ROS Node
  nh.initNode();
  nh.subscribe(input_sub);
}
void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);
}
