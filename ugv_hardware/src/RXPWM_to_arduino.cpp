#include <Arduino.h>

#define RJ_x 2 // right joystick x-axis - pin 6 on receiver
#define RJ_y 3
#define LJ_y 4
#define LJ_x 5
#define ESTOP 6
#define DIAL 7

// function definitions
String get_input(int pin);

void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(RJ_x, INPUT);
  pinMode(RJ_y, INPUT);
  pinMode(LJ_y, INPUT);
  pinMode(LJ_x, INPUT);
  pinMode(ESTOP, INPUT);
  pinMode(DIAL, INPUT);

}

void loop(){

  Serial.print("A");
  Serial.print(get_input(RJ_x));
  Serial.print("B");
  Serial.print(get_input(RJ_y));
  Serial.print("C");
  Serial.print(get_input(LJ_y));
  Serial.print("D");
  Serial.print(get_input(LJ_x));
  Serial.print("E");
  Serial.print(get_input(ESTOP));
  Serial.print("F");
  Serial.print(get_input(DIAL));
}

String get_input(int pin){
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
