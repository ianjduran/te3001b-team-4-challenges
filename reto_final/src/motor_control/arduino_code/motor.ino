#include <Arduino.h>
#include <ros.h>
#include <math.h>
#include <std_msgs/Float32.h>

//ROS node handler
ros::NodeHandle nh;

// Pin definitions
const int MOTOR_PWM = 2;
const int MOTOR_A = 22;
const int MOTOR_B = 23;
const int ENCODER_PIN_A = 18;
const int ENCODER_PIN_B = 19;

/**
 * Variables for encoder count and velocity calculations
 */
volatile int enc_count = 0;

int last_count = 0;
unsigned long last_enc_check = millis();

float update_encoder() {
  // Get last time upd encoder was ran
  unsigned long dt = millis() - last_enc_check;
  if (dt == 0) return 0;

  // Store count in a temporary (non volatile) variable
  int protCount;
  noInterrupts();
  protCount = enc_count;
  interrupts();

  // Velocity = delta count / delta time
  float velocity = (float)(protCount - last_count) / (float)dt;

  // Update last variables, to calculate deltas
  last_count = protCount;
  last_enc_check = millis();

  // Convert counts/ms to rotations/minute
  velocity *= 1000;
  velocity /= 35.0 * 13;  // Convert to rotations per sec
  velocity *= M_PI * 2;         // Convert to rad/s

  return velocity;
}

/**
 * Variables for calculating moving average
 */
constexpr static int moving_average_n = 10;
float moving_average_window[moving_average_n];
int moving_average_pos = 0;

// Computes a new, filtered velocity, uses a moving average
float filter_velocity(float new_velocity) {
  // Place new value into window
  moving_average_window[moving_average_pos++] = new_velocity;

  // Wrap window if exceeded
  if (moving_average_pos == moving_average_n) moving_average_pos = 0;

  // Calculate new average
  float sum = 0;
  for (int i = 0; i < moving_average_n; i++) {
    sum += moving_average_window[i];
  }

  return sum / (float)moving_average_n;
}

// If channel A leading B, CW, increment count, otherwise channel B leading A,
// CCW, decrement count
void isr_enc_A() {
  if (!digitalRead(ENCODER_PIN_B)) enc_count++;
}

void isr_enc_B() {
  if (!digitalRead(ENCODER_PIN_A)) enc_count--;
}

// Takes in -1 to 1 value, clamp it and write it to motor pwm, change direction
// to fwd or rvd

void write_motor(float val) {
  if (abs(val) > 1) val = copysign(1, val);  // Clamp value to -1 and 1

  digitalWrite(MOTOR_A, val > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_B, val > 0 ? LOW : HIGH);
  analogWrite(MOTOR_PWM, abs(val) * 255);
}

//subscripition CAMBIAR TOPICO Y CALLBACK
ros::Subscriber<std_msgs::Float32> input_sub("motor_input", &input_cb);
std_msgs::Float32 motor_out_msg;
ros::Publisher motor_pub("motor_output", &motor_out_msg);
unsigned long last_msg_time = 0;

void input_cb(const std_msgs::Float32 & val){
  write_motor(val.data);
  last_msg_time = millis();
}

void setup() {
  // Init motor pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);

  // Init enc pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach enc interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), isr_enc_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), isr_enc_B, RISING);

  // Initialize moving average window
  for (int i = 0; i < moving_average_n; i++) {
    moving_average_window[i] = 0;
  }

  //Initialize ROS Node
  nh.initNode();
  nh.subscribe(input_sub);
  nh.advertise(motor_pub);

  write_motor(0);
}

void loop() {
  float velocity = update_encoder();
  float filtered_velocity = filter_velocity(velocity);
  motor_out_msg.data = filtered_velocity;
  motor_pub.publish(&motor_out_msg);

    // Set motor to 0 when msg exceedes timeout
  if(millis() - last_msg_time > 5000){
    write_motor(0);
  }
  
  nh.spinOnce();
  delay(10);
}
