#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <Arduino.h>
#include <ugv_msg/RC.h>

// Motor defines
#define M1 16
#define M2 17

// Futaba inputs
#define R_X 36
#define R_Y 39
#define L_X 34
#define L_Y 35
#define E_STOP 33
#define AUTO 32

// Encoder Mappings
#define RENCA 5
#define LENCA 19
#define PPR 48

#define MAX_MPS 2
#define MIN_MPS -2

#define FREQ 330 // PWM frequency for victor SPX motors.

hw_timer_t *Timer0_Cfg = NULL;
int rightPulses = 0;
int leftPulses = 0;
int pwm = 0;

// Task Handles
TaskHandle_t rc_task = NULL;
TaskHandle_t main_task = NULL;

// RC message for publishing
ugv_msg::RC rc_msg;

// Map function
double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Map fucnction with customizable deadzone
double mapd(double x, double in_min, double in_max, double out_min, double out_max, double deadzone_min, double deadzone_max) {
  
  if ((deadzone_min < x && x < deadzone_max))
  {
    return 0;   
  }

  if(x < in_min)
    return in_min;
  
  if(x > in_max)
    return in_max;
  

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 0 49 MAX PWM
// -1.2 1.2 CMD_VEL
void spinMotor(int motorChannel, double cmdvel) {
  if (motorChannel == 1) {
    cmdvel *= -1;    
  }
  pwm = (int)map(cmdvel,-2,2,101,159);
  Serial.println(pwm);

  if(cmdvel > 1.2 || cmdvel < -1.2)
    return;

  ledcWrite(motorChannel, pwm);

}

// callback function for the cmd_vel subscriber
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

// create a rosserial node
ros::NodeHandle nh;

// create a subscriber
ros::Subscriber<geometry_msgs::Twist> jetson_sub("/cmd_vel", &msg_cb);

// create a publisher
geometry_msgs::Twist twist_msg;
ros::Publisher jetson_pub("/enc_vel", &twist_msg);

// create a publisher for the RC message. Note: L_Y = linear.x, R_X = angular.z, just like motors.py script.
ros::Publisher rc_pub("choo_2/rc", &rc_msg);

// Publishes RC message from futaba inputs
void rc_listener(void *param)
{
  // Get values for right and left sticks, mapping them to the -100 - 100 range. Ranges for in_min and in_max are from testing 
  double r_x = mapd(pulseIn(R_X, HIGH), 1100, 2100, -100, 100, 1500, 1530);
  double r_y = mapd(pulseIn(R_Y, HIGH), 920, 2100,  -100, 100, 1510, 1530);
  double l_x = mapd(pulseIn(L_X, HIGH), 1108, 1944, -100, 100, 1460, 1560);
  double l_y = mapd(pulseIn(L_Y, HIGH), 1090, 1935, -100, 100, 1500, 1520);

  // Throw these values into the rc_msg and publish
  rc_msg.right_x = r_x;
  rc_msg.right_y = r_y;
  rc_msg.left_x = l_x;
  rc_msg.left_y = l_y;

  // Get values. 1 if toggled, 0 if not. 
  int d_switch = (pulseIn(AUTO, HIGH) > 2000) ? 1 : 0;
  int e_switch = (pulseIn(E_STOP, HIGH) > 2000) ? 1 : 0;

  // We're only using switches d (AUTO) and e (E_STOP)
  rc_msg.switch_a = 0;
  rc_msg.switch_b = 0;
  rc_msg.switch_d = d_switch;
  rc_msg.switch_e = e_switch;
  rc_msg.switch_f = 0;
  rc_msg.switch_g = 0;
  
  rc_pub.publish(&rc_msg);

}

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

void core0_main(void* param)
{
  nh.spinOnce();
  delay(100);
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

  attachInterrupt(digitalPinToInterrupt(RENCA), rightEncInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA), leftEncInterrupt, RISING);

  //initialize the node
  nh.initNode();
  nh.subscribe(jetson_sub);
  nh.advertise(jetson_pub);
  nh.advertise(rc_pub);

  ledcSetup(1, 345, 8);
  ledcAttachPin(M1, 1);
  ledcSetup(2, 345, 8);
  ledcAttachPin(M2, 2);

  // timer setup
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100000, true);
  timerAlarmEnable(Timer0_Cfg);

  xTaskCreatePinnedToCore(
    rc_listener,
    "rc_listener",
    10000,
    NULL,
    5,
    &rc_task,
    1
  );

  xTaskCreatePinnedToCore(
    core0_main,
    "core0_main",
    10000,
    NULL,
    5,
    &main_task,
    0
  );

}

void loop(){}