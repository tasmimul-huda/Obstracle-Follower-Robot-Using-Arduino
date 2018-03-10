
//obstracle follower using pid algorithm for two sonar sensor

//Tuning, a great problem, kp ki kd respectively tuned 

/*Auther: Sayed Mohammed Tasmimul Huda
Khulna Univarsity of Engineering & Technology*/

#include<NewPing.h>
#define ECOPIN1 A2
#define TRIGPIN1 A3
#define ECOPIN2 A4
#define TRIGPIN2 A5
#define  MAX_DISTANCE 200
NewPing sonar1(TRIGPIN1, ECOPIN1, MAX_DISTANCE);
NewPing sonar2(TRIGPIN2, ECOPIN2, MAX_DISTANCE);
#define m11 4
#define m12 2
#define m21 7
#define m22 8
#define E1 A0  //pmw pin for left motor
#define E2 A1   //pmw pin for right motor
#define max_speed 255
#define turn_speed 140
#define base_speed 150
int setpoint;
int currentpoint;
int last_propotional;
int Kp,Kd,Ki;
int PID_output;

int right_motor_speed;
int left_motor_speed;

int propotional,integral,derivative;
void setup()
{
 Serial.begin(115200);
 //delay(5000);
 pinMode(m11,OUTPUT);
 pinMode(m12,OUTPUT);
 pinMode(m21,OUTPUT);
 pinMode(m21,OUTPUT);
}
void read_soanr()
{
  //delay(100);
  Serial.print(" ");
  Serial.print("Distance from left sonar= ");
  Serial.print(sonar1.ping_cm());
  Serial.println("cm");
  Serial.print(" ");
  Serial.print("Distance from right sonar= ");
  Serial.print(sonar2.ping_cm());
  Serial.println("cm");
  Serial.print(" ");
}


void sonar_pid()
{
  Kp=10,Kd=2,Ki=0.1;
  currentpoint=sonar1.ping_cm()-sonar2.ping_cm();
  setpoint=0;
  propotional=currentpoint-setpoint;
  integral=integral+propotional;
  derivative=propotional-last_propotional;
  PID_output=propotional*Kp+integral*Ki+derivative*Kd;
  last_propotional=propotional;
  Serial.print("current value:");
  Serial.print(currentpoint);
  Serial.println(" ");
  Serial.print("protional:");
  Serial.print(propotional);
  Serial.println(" ");
  Serial.print("pid output:");
  Serial.print(PID_output);
  Serial.println(" ");
}


void calculating_turn()   //Restricting the PID_output between max_speed.
{
  /*if (PID_output < -base_speed)
  {
    PID_output = base_speed;
  }
  else if (PID_output > base_speed)
  {
    PID_output = base_speed;
  }*/

  if (PID_output > 0)   //Too much right need to go left
  {
    right_motor_speed = base_speed -PID_output ;
    left_motor_speed = base_speed +PID_output;
  }
  else
  {
    //Need to go left
    right_motor_speed = base_speed-PID_output+10;
    left_motor_speed = base_speed +PID_output+40;

  }
  if(right_motor_speed<0)
  right_motor_speed=0;
  if(left_motor_speed<0)
  left_motor_speed=0;
  }
  void motor()
  {
  digitalWrite(m11,HIGH);
  digitalWrite(m12,LOW);
  digitalWrite(m21,HIGH);
  digitalWrite(m22,LOW); 
  analogWrite(E1,right_motor_speed);
  analogWrite(E2,left_motor_speed);
  Serial.print("right motor speed");
  Serial.print(right_motor_speed);
  Serial.println(" ");
  Serial.print("left motor speed");
  Serial.print(left_motor_speed);
  Serial.println(" ");
  
  }

void final()
{
  //delay(5000);
  read_soanr();
  //sonar_mode();
  sonar_pid();
  calculating_turn();
  
  motor();
  /*digitalWrite(m11,HIGH);
  digitalWrite(m12,LOW);
  digitalWrite(m21,HIGH);
  digitalWrite(m22,LOW); 
  analogWrite(E1,right_motor_speed);
  analogWrite(E2,left_motor_speed);
  Serial.print("right motor speed");
  Serial.print(right_motor_speed);
  Serial.println(" ");
  Serial.print("left motor speed");
  Serial.print(left_motor_speed);
  Serial.println(" ");*/
}
void loop()
{
  final();
}

