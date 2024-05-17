#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <Arduino.h>
// #include <ESP32Servo.h>
#define FRPIN 34
#define FLPIN 35
#define RRPIN 32
#define RLPIN 39
#define M1 16
#define M2 17

// Encoder Mappings
#define RENCA 5
#define LENCA 19
#define PPR 48

// Servo motorR; // 16
// Servo motorL; // 17
hw_timer_t *Timer0_Cfg = NULL;
int rightPulses = 0;
int leftPulses = 0;
int pwm = 0;

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 0 49 MAX PWM
// -1.2 1.2 CMD_VEL
void spinMotor(int motorChannel, double cmdvel) {
  if (motorChannel == 1) {
    cmdvel *= -1;    
  }
  pwm = (int)mapd(cmdvel,-2,2,101,159);
  Serial.println(pwm);
  // int microSec = (int)mapd(pwm,0,255,1000,2000);
  // Serial.println(microSec);

  if(cmdvel > 1.2 || cmdvel < -1.2)
    return;

  ledcWrite(motorChannel, pwm);
  // motor.writeMicroseconds(microSec);
}

// callback function for the cmd_vel subscriber
void msg_cb(const geometry_msgs::Twist &msg);

// create a rosserial node
ros::NodeHandle nh;

// create a subscriber
ros::Subscriber<geometry_msgs::Twist> jetson_sub("/cmd_vel", &msg_cb);

// create a publisher
geometry_msgs::Twist twist_msg;
ros::Publisher jetson_pub("/enc_vel", &twist_msg);

void IRAM_ATTR rightEncInterrupt() 
{
  if (pwm > 127)
    rightPulses++;
  else if (pwm < 127)
    rightPulses--;   
}

void IRAM_ATTR leftEncInterrupt()
{
  if (pwm > 127)
    leftPulses++;
  else if (pwm < 127)
    leftPulses--;
}

void IRAM_ATTR Timer0_ISR()
{
  float period = 1;
  float circumference = 0.6604; // meters

  float rightDist = circumference * (float) rightPulses / (float) PPR;
  float leftDist = circumference * (float) leftPulses / (float) PPR;

  // m/s
  float rightVel = rightDist / period;
  float leftVel = leftDist / period;

  twist_msg.linear.x = rightVel;
  twist_msg.linear.y = leftVel;

  jetson_pub.publish(&twist_msg);

  rightPulses = 0;
  leftPulses = 0;
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  Serial.begin(115200, SERIAL_8N1);
  Serial.end();
  Serial.begin(115200, SERIAL_8N1);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(RENCA, INPUT_PULLUP);
  pinMode(LENCA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(rightAPin), rightEncInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(leftAPin), leftEncInterrupt, RISING);

  //initialize the node
  nh.initNode();
  nh.subscribe(jetson_sub);
  nh.advertise(jetson_pub)

  ledcSetup(1, 345, 8);
  ledcAttachPin(M1, 1);
  ledcSetup(2, 345, 8);
  ledcAttachPin(M2, 2);

  // timer setup
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100000, true);
  timerAlarmEnable(Timer0_Cfg);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}

void msg_cb(const geometry_msgs::Twist &msg)
{
  float linear_x = msg.linear.x;
  float angular_z = msg.angular.z;

  float right_wheel = (linear_x + angular_z)/2;
  float left_wheel = (linear_x - angular_z)/2;

  spinMotor(1, right_wheel);
  spinMotor(2, left_wheel);

  Serial.print(right_wheel);
  Serial.print("  ");
  Serial.print(left_wheel);
  Serial.println("  ");
}