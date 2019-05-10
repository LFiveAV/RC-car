// change pins
const int trigPin1 = 10;
const int echoPin1 = 8;

const int trigPin2 = 10;
const int echoPin2 = 8;

const int trigPin3 = 10;
const int echoPin3 = 8;

const int trigPin4 = 10;
const int echoPin4 = 8;

const int trigPin5 = 10;
const int echoPin5 = 8;

int distance1,distance2,distance3,distance4,distance5;

void setup() {
  pinMode(trigPin1, OUTPUT); 
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT); 
  pinMode(trigPin3, OUTPUT); 
  pinMode(echoPin3, INPUT); 
  pinMode(trigPin4, OUTPUT); 
  pinMode(echoPin4, INPUT); 
  pinMode(trigPin5, OUTPUT); 
  pinMode(echoPin5, INPUT); 
  Serial.begin(9600);
}

void loop() {
  distance1 = get_distance(trigPin1, echoPin1);
  distance2 = get_distance(trigPin2, echoPin2);
  distance3 = get_distance(trigPin3, echoPin3);
  distance4 = get_distance(trigPin4, echoPin4);
  distance5 = get_distance(trigPin5, echoPin5);
  
  Serial.print("Distance1: ");
  Serial.println(distance1);
  Serial.print("Distance2: ");
  Serial.println(distance2);
  Serial.print("Distance3: ");
  Serial.println(distance3);
  Serial.print("Distance4: ");
  Serial.println(distance4);
  Serial.print("Distance5: ");
  Serial.println(distance5);

  int distanceArray[] = {distance1,distance2,distance3,distance4,distance5};
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

int get_distance(int trigPin, int echoPin) {
  long duration = get_duration(trigPin,echoPin);
  int distance = duration*0.034/2;
  return distance;
}

void send_data(int distanceArray[]){
  if (Serial.available()) {
    Serial.print("<");
    for(int i=0; i<5; i++)
   {
      Serial.print(distanceArray[i]);
      if (i<4){
        Serial.print(",");
      }
      
   }
   Serial.print(">");
  }
}
