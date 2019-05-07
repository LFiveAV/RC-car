                         
int pwm_pin  = 10; 
int pot_val;
int pwm;
int led_pin = 5;

void setup()

{
    pinMode(pwm_pin,OUTPUT);
    Serial.begin(9600);
    Serial.println("Enter values between 0 - 255");
}

void loop()
  

{
  pot_val = analogRead(A0);
  pwm = map(pot_val, 0, 1023, 0, 255);
  Serial.println(pwm);
  analogWrite(pwm_pin, pwm);  
}
