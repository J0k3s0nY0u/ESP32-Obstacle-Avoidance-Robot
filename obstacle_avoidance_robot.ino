#include <ESP32Servo.h> //ESP32 Servo Library
#include <NewPing.h> //For Ultrasonic Sensor Functionality

//Buzzer and ohter Stuffs
const int buzzer = 8; // D8

//L298N(OR WHATEVER WE'RE USING) Control Pins
const int leftMotorForward = 4; // D4
const int leftMotorBackward = 5; // D5
const int rightMotorForward = 6; // D6
const int rightMotorBackward = 7; // D7

//Ultrasonic Sensor Pins
const int trig = T1; //Input 1 A1
const int echo = T2; //Input 2 A2

//Measurement Calibration
const int maxDistance = 200; //Self Explanatory
bool goForward = false; //Self Explanatory
int distance = 100; //Self Explanatory
NewPing sonar(trig, echo, maxDistance); //Sensors Function
Servo servoMotor; //Self Explanatory

//Looking Right
int lookRight() {
  servoMotor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servoMotor.write(115);
  return distance;
}

//Looking Left
int lookLeft(){
  servoMotor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servoMotor.write(115);
  return distance;
  delay(100);
}

//Reading Ping
int readPing(){
  delay(70);
  int cm;
  cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
pinMode(leftMotorForward, OUTPUT);
pinMode(rightMotorForward, OUTPUT);
pinMode(leftMotorBackward, OUTPUT);
pinMode(rightMotorBackward, OUTPUT);
pinMode(buzzer, OUTPUT);

servoMotor.attach(3); //Servo Pin D3
servoMotor.write(115);
delay(2000);
distance = readPing();
delay(100);
distance = readPing();
delay(100);
distance = readPing();
delay(100);
distance = readPing();
delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

  ON_START:
distance = readPing();
Serial.println(distance);
delay(10);
int distanceRight = 0;
int distanceLeft = 0;
delay(10);
goto IF_STATEMENTS_FOR_OBSTACLE_DETECTION;
Serial.println(distance);

//If Statements for Obstacle Detection
IF_STATEMENTS_FOR_OBSTACLE_DETECTION:
if (distance <= 20) {
  goto STOP_MOVEMENT;
  delay(500);
  goto MOVE_BACKWARD;
  delay(250);
  goto STOP_MOVEMENT;
  delay(250);
  distanceRight = lookRight();
  Serial.println(distanceRight);
  delay(500);
  distanceLeft = lookLeft();
  Serial.println(distanceLeft);
  delay(500);
  goto OBSTACLE_AVOIDANCE;
} else {
  goto MOVE_FORWARD;
  distance = readPing();
}

//Obstacle Avoidance
OBSTACLE_AVOIDANCE:
  if (distanceRight >= distanceLeft) {
    goto TURN_RIGHT;
    goto STOP_MOVEMENT;
  } else if (distanceRight <= distanceLeft) {
    goto TURN_LEFT;
    goto STOP_MOVEMENT;
  } else {
    goto DETECTION_ERROR;
  }
  


//Movement Stops
STOP_MOVEMENT:
digitalWrite(rightMotorForward, LOW);
digitalWrite(leftMotorForward, LOW);
digitalWrite(rightMotorBackward, LOW);
digitalWrite(leftMotorBackward, LOW);

//Move Forward
MOVE_FORWARD:
if(!goForward){
    goForward=true;
    digitalWrite(leftMotorForward, HIGH);
    digitalWrite(rightMotorForward, HIGH);
    digitalWrite(leftMotorBackward, LOW);
    digitalWrite(rightMotorBackward, LOW);
  }

  //Move Backward
  MOVE_BACKWARD:
  goForward=false;
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(rightMotorForward, LOW);

  //Turn Right
  TURN_RIGHT:
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorBackward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  delay(500);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorBackward, LOW);

  //Turn Left
  TURN_LEFT:
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
  delay(500);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorBackward, LOW);

//Detection Error
DETECTION_ERROR:
digitalWrite(buzzer, HIGH);

}
