#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

SoftwareSerial portOne(10, 11); // TODO: change pins

// PWM-control
float pwm1 = 0; 
float pwm2 = 0;

ROS_CALLBACK(pwm_callback,std_msgs::Float32MultiArray){
  pwm1=msg.data[0];
  pwm2=msg.data[1];
}

// ROS
ros::NodeHandle nh;
std_msgs::Float32MultiArray ultra_msgs; 
ros::Publisher ultrasonic("ultrasonic", &ultra_msgs);
ros::Subscriber pwm("pwm",&msg,&pwm_callback);
boolean something_to_publish = false;

// Ultrasonic 
int num_sensors = 5;  // atm only 5 works :(
const byte num_chars = 32;
char recieved_data[num_chars];
int recieved_distanace[num_sensors] = {};

// TX/RX
boolean newData = false;
char startMarker = '<';
char endMarker = '>';
char delim[1] = ",";

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.advertise(ultrasonic);
  nh.subscribe(pwm);

}

void loop() {
  motor_control();
  get_data();
  if (newData == true) {
    parseData();
    //showData();
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

void motor_control(){
  Serial.print("PWM1: ");
  Serial.println(pwm1);
}

void get_data() {
  static boolean receiving_in_progress = false;
  int idx = 0;
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

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
  ultra_msgs.recieved_distanace = recieved_distanace;
  ultra_msgs.recieved_distanace_length = 5;
  ultrasonic.publish( &ultra_msgs);
}

void showData() {
  Serial.print("Distance 1: ");
  Serial.println(recieved_distanace[0]);
  Serial.print("Distance 2: ");
  Serial.println(recieved_distanace[1]);
  Serial.print("Distance 3: ");
  Serial.println(recieved_distanace[2]);
  Serial.print("Distance 4: ");
  Serial.println(recieved_distanace[3]);
  Serial.print("Distance 5: ");
  Serial.println(recieved_distanace[4]);
}
