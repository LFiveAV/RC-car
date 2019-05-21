#include <ros.h>
#include <custom_messages/ultrasonic.h>
#include <custom_messages/dc_pwm.h>



// PWM-control
int pwm = 0; 
String mode = "standstill";
String prev_mode = "standstill";
int forward_pin = 12; // change pin number
int reverse_pin = 13; // change pin number
int pwm_pin = 11; // change pin number

void pwm_callback(const custom_messages::dc_pwm &msg){
  pwm=msg.pwm;
  mode=msg.mode;
}

// ROS
ros::NodeHandle nh;
custom_messages::ultrasonic ultra_msgs; 
ros::Publisher ultrasonic("ultrasonic", &ultra_msgs);
ros::Subscriber<custom_messages::dc_pwm> pwm_sub("pwm",&pwm_callback);

// Ultrasonic   
int distanaceArray[4];

// change pins
const int trigPin1 = 2;
const int echoPin1 = 3;

const int trigPin2 = 4;
const int echoPin2 = 5;

const int trigPin3 = 6;
const int echoPin3 = 7;

const int trigPin4 = 8;
const int echoPin4 = 9;

void setup() {
  
  nh.initNode();
  nh.advertise(ultrasonic);
  nh.subscribe(pwm_sub);

  pinMode(trigPin1, OUTPUT); 
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT); 
  pinMode(trigPin3, OUTPUT); 
  pinMode(echoPin3, INPUT); 
  pinMode(trigPin4, OUTPUT); 
  pinMode(echoPin4, INPUT); 
}

void loop() {
  get_ultrasonic();
  publish_ultrasonic();
  motor_control();
  nh.spinOnce();
  delay(10);
}

void get_ultrasonic() {
  distanaceArray[0] = get_distance(trigPin1, echoPin1);
  distanaceArray[1] = get_distance(trigPin2, echoPin2);
  distanaceArray[2] = get_distance(trigPin3, echoPin3);
  distanaceArray[3] = get_distance(trigPin4, echoPin4);
}

int get_distance(int trigPin, int echoPin) {
  long duration = get_duration(trigPin,echoPin);
  int distance = duration*0.034/2;
  return distance;
}

long get_duration(int trigPin, int echoPin) {
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration;
}

void publish_ultrasonic() {
  ultra_msgs.sensor1 = distanaceArray[0];
  ultra_msgs.sensor2 = distanaceArray[1];
  ultra_msgs.sensor3 = distanaceArray[2];
  ultra_msgs.sensor4 = distanaceArray[3];
  ultrasonic.publish( &ultra_msgs);
}

void motor_control() {
  if (mode == "standstill") {
    digitalWrite(forward_pin,LOW);
    digitalWrite(reverse_pin,LOW);

    // Adding a delay before changing direction
    if (prev_mode != mode) {
      delay(500);
    }
  }
  else if (mode == "forward") {
    digitalWrite(forward_pin,HIGH);
    digitalWrite(reverse_pin,LOW);
  }
  else if (mode == "reverse") {
    digitalWrite(forward_pin,LOW);
    digitalWrite(reverse_pin,HIGH);
  }
  else {
    digitalWrite(forward_pin,LOW);
    digitalWrite(reverse_pin,LOW);
  }

  pwm = 255; // Always max speed
  
  analogWrite(pwm_pin,pwm);
  prev_mode = mode;
  
}
