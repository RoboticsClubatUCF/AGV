/**
 * @file RXPWM_to_arduino.cpp
 * @author Wesley Fletcker, Tevin Mukudi
 * @brief 
 * @version 0.1
 * @date 2022-05-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>

#define RC_RJ_x 2 // right joystick x-axis - pin 6 on receiver
#define RC_RJ_y 3
#define RC_LJ_y 4
#define RC_LJ_x 5
#define RC_ESTOP 6
#define RC_DIAL 7

// function prototypes
String get_input(int pin);
void print_rc_inputs();

int rc_in[6] = {0,0,0,0,0,0}; // hold RC inputs
int hz = 100;  // loop rate

/**
 * @brief Run once.
 * 
 */
void setup()
{  
  // set-up serial connection
  Serial.begin(9600);

  // configure RC pins as inputs
  pinMode(RC_RJ_x, INPUT);
  pinMode(RC_RJ_y, INPUT);
  pinMode(RC_LJ_y, INPUT);
  pinMode(RC_LJ_x, INPUT);
  pinMode(RC_ESTOP, INPUT);
  pinMode(RC_DIAL, INPUT);
}

/**
 * @brief Run continuously.
 * 
 */
void loop()
{
  // print RC inputs to the Jetson
  print_rc_inputs();

  // attempt to maintain loop rate
  delay(1000 / hz); 
}

/**
 * @brief Print the current value of RC pins to Jetson
 * 
 */
void print_rc_inputs()
{
  Serial.print("$RCX");
  Serial.print(" ");
  Serial.print(get_input(RC_RJ_x));
  Serial.print(" ");
  Serial.print(get_input(RC_RJ_y));
  Serial.print(" ");
  Serial.print(get_input(RC_LJ_y));
  Serial.print(" ");
  Serial.print(get_input(RC_LJ_x));
  Serial.print(" ");
  Serial.print(get_input(RC_ESTOP));
  Serial.print(" ");
  Serial.print(get_input(RC_DIAL));
  Serial.print("\n");
}

/**
 * @brief Read PPM data from a single pin using pulseIn()
 * 
 * @param pin pin to be read
 * @return String
 */
String get_input(int pin)
{
  // use pulseIn to turn PWM signal to values within a range
  int input = pulseIn(pin, HIGH);
  
  // turn the input to a string
  String s_input = String(input);

  // make sure input is 4 bytes
  if(s_input.length() < 4){
    s_input += "_";
  }

  return s_input;
}