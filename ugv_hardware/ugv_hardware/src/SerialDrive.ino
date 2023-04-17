#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
// #include <ESP32Servo.h>
#define FRPIN 34
#define FLPIN 35
#define RRPIN 32
#define RLPIN 39
#define M1 16
#define M2 17

// Servo motorR; // 16
// Servo motorL; // 17

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 0 49 MAX PWM
// -1.2 1.2 CMD_VEL
void spinMotor(int motorChannel, double cmdvel) {
  if (motorChannel == 1) {
    cmdvel *= -1;    
  }
  int pwm = (int)mapd(cmdvel,-2,2,101,159);
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

void setup()
{
  nh.getHardware()->setBaud(115200);
  Serial.begin(115200, SERIAL_8N1);
  Serial.end();
  Serial.begin(115200, SERIAL_8N1);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  
  //initialize the node
  nh.initNode();
  nh.subscribe(jetson_sub);
  
  ledcSetup(1, 345, 8);
  ledcAttachPin(M1, 1);
  ledcSetup(2, 345, 8);
  ledcAttachPin(M2, 2);
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

