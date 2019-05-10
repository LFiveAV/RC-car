#include <SoftwareSerial.h>
#include <ros.h>
#include<std_msgs/String.h> // TODO: change msg type

SoftwareSerial portOne(10, 11); // TODO: change pins

ros::NodeHandle nh;
std_msgs::String ultra_msgs; // TODO: change msg type
ros::Publisher ultrasonic("ultrasonic", &ultra_msgs);

int num_sensors = 5;  // atm only 5 works :(
const byte num_chars = 32;
char recieved_data[num_chars];
int recieved_distanace[num_sensors] = {};
boolean newData = false;
char startMarker = '<';
char endMarker = '>';
char delim[1] = ",";

void setup() {
  Serial.begin(9600);

  nh.initNode();
  nh.advertise(ultrasonic);

}

void loop() {
  get_data();
  if (newData == true) {
    parseData();
    //showData();
    newData = false;
  }
  publish_data();
  nh.spinOnce();
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
  ultra_msgs.sensor1 = recieved_distanace[0];
  ultra_msgs.sensor2 = recieved_distanace[1];
  ultra_msgs.sensor3 = recieved_distanace[2];
  ultra_msgs.sensor4 = recieved_distanace[3];
  ultra_msgs.sensor5 = recieved_distanace[4];
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
