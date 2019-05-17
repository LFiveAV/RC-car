#include <SoftwareSerial.h>
#include <ros.h>
#include <custom_messages/ultrasonic.h>
#include <custom_messages/dc_pwm.h>
// ? ^

SoftwareSerial portOne(10, 11); // TODO: change pins

// PWM-control
int pwm = 0; 
String mode = "standstill";
String prev_mode = "standstill";
int forward_pin = 6; // change pin number
int reverse_pin = 7; // change pin number
int pwm_pin = 8; // change pin number

void pwm_callback(const custom_messages::dc_pwm &msg){
  pwm=msg.pwm;
  mode=msg.mode;
}

// ROS
ros::NodeHandle nh;
custom_messages::ultrasonic ultra_msgs; 
ros::Publisher ultrasonic("ultrasonic", &ultra_msgs);
ros::Subscriber<custom_messages::dc_pwm> pwm_sub("pwm",&pwm_callback);
boolean something_to_publish = false;

// Ultrasonic 
int num_sensors = 5;  // atm only 5 works :(
const byte num_chars = 32;
char recieved_data[num_chars];
long int recieved_distanace[5];

// TX/RX
boolean newData = false;
char startMarker = '<';
char endMarker = '>';
char delim[] = ",";

void setup() {
  portOne.begin(9600);
  nh.initNode();
  nh.advertise(ultrasonic);
  nh.subscribe(pwm_sub);

}

void loop() {
  motor_control();
  get_data();
  if (newData == true) {
    parseData();
    newData = false;
    something_to_publish = true;
  }
  if (something_to_publish == true) {
    publish_data();
    
    something_to_publish = false;
  }
  nh.spinOnce();
  delay(10);
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
  analogWrite(pwm_pin,pwm);
  prev_mode = mode;
  
}



void get_data() {
  static boolean receiving_in_progress = false;
  int idx = 0;
  char rc;

  while (portOne.available() > 0 && newData == false) {
    rc = portOne.read();

    if (receiving_in_progress == true) {
      if (rc != endMarker) {
        recieved_data[idx] = rc;
        idx++;
        if (idx >= num_chars) {
          idx = num_chars - 1 ;
        }
      }
      else {
        recieved_data[idx] = '\0';
        receiving_in_progress = false;
        idx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      receiving_in_progress = true;
    }
  }
}

void parseData() {
  char * token;
  static byte idx = 0;

  token = strtok(recieved_data, delim);
  while (token != NULL) {
    recieved_distanace[idx] = atoi(token);
    token = strtok(NULL, delim);
    idx++;
  }
}

void publish_data() {
  ultra_msgs.sensor1 = recieved_distanace[0];
  ultrasonic.publish( &ultra_msgs);
}
