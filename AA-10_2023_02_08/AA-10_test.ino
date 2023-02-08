/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/
#define encodPinA1 2
#define encodPinB1 3
#define MOTOR_DIR 4
#define MOTOR_PWM 5
#include <Servo.h>
#include <NewPing.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

NewPing R_sensor(9,9,400);
  float R_Sonar_distance = 0.0;

NewPing L_sensor(10,10,400);
  float L_Sonar_distance = 0.0;
  
NewPing B_sensor(11,11,400);
  float B_Sonar_distance = 0.0;
  

void read_sonar_sensor(void){ //초음파센서 측정
    R_Sonar_distance = R_sensor.ping_cm()*10.0;
    L_Sonar_distance = L_sensor.ping_cm()*10.0;
    B_Sonar_distance = B_sensor.ping_cm()*10.0;
    if(R_Sonar_distance == 0){R_Sonar_distance = 400 * 10.0;}
    if(L_Sonar_distance == 0){L_Sonar_distance = 400 * 10.0;}
    if(B_Sonar_distance == 0){B_Sonar_distance = 400 * 10.0;}
  }
void setup() {
  myservo.attach(8);
  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);// attaches the servo on pin 9 to the servo object
  Serial.begin(115200);
}
void motor_control(int direction, int speed){
  digitalWrite(MOTOR_DIR,direction);
  analogWrite(MOTOR_PWM,speed);
  }
void loop() {
   read_sonar_sensor();
   motor_control(1,130);
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(97);              // tell servo to go to position in variable 'pos'
  //  delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(97);              // tell servo to go to position in variable 'pos'
 //   delay(15);                       // waits 15 ms for the servo to reach the position
  }
  Serial.print(B_Sonar_distance);
  Serial.print("  ");
  Serial.print(L_Sonar_distance);
  Serial.print("  ");
  Serial.println(R_Sonar_distance);




  
}
