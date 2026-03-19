#include <ESP32Servo.h>

Servo myservo;
const int servoPin = 19; 
const int buttonPin = 4; 
const int trigPin = 27;
const int echoPin = 26;
float duration, distance;

const int closeAngle = 20; 
const int openAngle = 180; 
bool clawIsClosed=false;
void setup() {
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400); 
  pinMode(trigPin, OUTPUT);//para sensor de ultrassons
  pinMode(echoPin, INPUT);//para sensor de ultrassons
  
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Set to 115200 for faster, cleaner communication
  Serial.begin(9600);
  Serial.println("--- System Initialized ---");
}

void loop() {
  int buttonState = digitalRead(buttonPin);
digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(10);
  if(clawIsClosed == false && distance<10){
myservo.write(closeAngle);

  } else{
  
  if (buttonState == LOW ) {
    // We only toggle once per press
    if (clawIsClosed == false) {
      myservo.write(closeAngle);
      clawIsClosed = true;
      Serial.println("Action: Closing");
    } else {
      myservo.write(openAngle);
      clawIsClosed = false;
      Serial.println("Action: Opening");
    }
    
    // This "While" loop waits for you to let go of the button
    // This prevents the claw from freaking out if it's "Always Pressed"
    while(digitalRead(buttonPin) == LOW) { 
      delay(10); 
    }
  }
  delay(100); 
}
}
