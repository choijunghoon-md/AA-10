#define encodPinA1 2
#define encodPinB1 3
#define MOTOR_DIR 4
#define MOTOR_PWM 5

#define N_ANGLE 85 // 중립값
#define slave   0x05

#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>

Servo myservo;
NewPing R_sensor(9,9,150);
NewPing L_sensor(10,10,150);
NewPing B_sensor(11,11,150);

float B_Sonar_distance = 0.0;
float R_Sonar_distance = 0.0;
float L_Sonar_distance = 0.0;

int buf_cnt = 0;
int start_cnt_data = 0;

unsigned char buf[27];

union{
  short data;
  char  bytedata[2];
  }m_car_angle;

union{
  float data;
  char bytedata[4];
  }m_car_speed;

void steer_control(int angle){
  if(angle >= 30)  angle = 30;
  if(angle <= -30) angle = -30;
  myservo.write(N_ANGLE+angle);
  }

void motor_control(int speed){
  if(speed >= 0){
    digitalWrite(MOTOR_DIR,HIGH);
    analogWrite(MOTOR_PWM,speed);
    }
  else{
    digitalWrite(MOTOR_DIR,LOW);
    analogWrite(MOTOR_PWM,-speed);
    }
  }
  
void read_sonar_sensor(void){ //초음파센서 측정
    R_Sonar_distance = R_sensor.ping_cm();
    L_Sonar_distance = L_sensor.ping_cm();
    B_Sonar_distance = B_sensor.ping_cm();
    if(R_Sonar_distance == 0){R_Sonar_distance = 150;}
    if(L_Sonar_distance == 0){L_Sonar_distance = 150;}
    if(B_Sonar_distance == 0){B_Sonar_distance = 150;}
  }

void receiveEvent(int howMany){
  int x[9]={0,};
  if(Wire.available()){
    for(int i=0; i<9; i++){
      x[i] = Wire.read();
      //Serial.println(x[i]);
      buf[buf_cnt] = x[i];
      
      if(i==0){
        start_cnt_data = buf;
      }
      
      buf_cnt++;
      
      if(buf_cnt == 27){
        buf_cnt = 0;
        }
      }
    }
    int temp_data[9] ={0,};

    for(int i=0; i<9; i++){
      temp_data[i] = x[i];
      //Serial.println(temp_data[i]);
    }
    
    if((x[0] == '#')&&(x[1] == 'C')&&(x[8] == '*')){
      m_car_angle.bytedata[0] = temp_data[2];
      m_car_angle.bytedata[1] = temp_data[3];
      
      m_car_speed.bytedata[0] = temp_data[4];
      m_car_speed.bytedata[1] = temp_data[5];
      m_car_speed.bytedata[2] = temp_data[6];
      m_car_speed.bytedata[3] = temp_data[7];
      }
  }

void requestEvent() { //요청 시 수행 함수
  int send_data[7] = {'R',R_Sonar_distance,'L',L_Sonar_distance,'B',B_Sonar_distance,'*'};
  for(int i=0; i<7; i++){
    Wire.write(send_data[i]);
  }
}

void setup() {
  Wire.begin(slave);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  myservo.attach(8);
  
  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);
  
  Serial.begin(115200);
}
  
void loop() {
  read_sonar_sensor();
  motor_control(m_car_speed.data);
  steer_control(m_car_angle.data);
  Serial.print(B_Sonar_distance);
  Serial.print("  ");
  Serial.print(L_Sonar_distance);
  Serial.print("  ");
  Serial.print(R_Sonar_distance);
  Serial.print("  ");
  Serial.print(m_car_speed.data);
  Serial.print("  ");
  Serial.println(m_car_angle.data);
  
}
