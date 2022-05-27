/**
 * @file RXPWM_to_arduino.cpp
 * @author Wesley Fletcher, Tevin Mukudi
 * @brief 
 * @version 0.1
 * @date 2022-05-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdlib.h>
#include <Arduino.h>
#include <Encoder.h>

// RC Receiver
#define RC_RJ_x   2  // right joystick, x-axis - pin 6 on receiver
#define RC_RJ_y   3  // right joystick, y-axis
#define RC_LJ_y   4  // left joystick, y-axis
#define RC_LJ_x   5  // left joystick, x-axis
#define RC_ESTOP  6  // e-stop switch
#define RC_DIAL   7  // dial
// Encoders
#define ENC_1_A   18
#define ENC_1_B   19
#define ENC_2_A   20
#define ENC_2_B   21
// Motor Controller (on Serial3)
#define MC_TX     14
#define MC_RX     15
#define MC_SERIAL 3
// for string parsing
#define ENDSTDIN    255 // NULL
#define NL          10  // NEWLINE
#define CR          13  // CARRIAGE RETURN
// Jetson<-->Arduino message definitions
static const char * MSG_MOTORS = "$MTR\0";      // command for motor controller
static const char * MSG_CMD    = "$CMD\0";      // generic command, passed to SBC through serial
static const char * MSG_ACK    = "$ACK\0";      // an acknowledgement that a message was received
static const char * MSG_ESTOP  = "$STP\0";      // throw the motor controller E-STOP
static const char * MSG_LIGHTS = "$LIT\0";      // TODO: control the lights

// function prototypes
int handle_input(char *incoming);
int get_input(int pin);
void print_rc_inputs();
void print_encoders();
void send_command_ESTOP();
void send_command_SET_RPM(int channel, int RPM);
void send_command_SAFETY_STOP(int channel);

//////////////////////
///// Globals ////////
//////////////////////

int hz = 100;  // max/ideal loop rate

// encoder set up
Encoder enc1(ENC_1_A, ENC_1_B);
Encoder enc2(ENC_2_A, ENC_2_B);
long newPos1, newPos2;
long pos1 = -9999;
long pos2 = -9999;

// parsing USB commands
int bytes_available = 0;
char incoming[256];
int idx = 0;
char ch;

/**
 * @brief Run once.
 * 
 */
void setup()
{  
  // set-up serial connections
  Serial.begin(115200);   // USB-to-Jetson
  Serial3.begin(115200);  // to motor controller

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
  int status;
   
  // print RC inputs to the Jetson
  print_rc_inputs();

  // read encoders
  newPos1 = enc1.read();  // Encoder::read() returns counts, there are 4 counts / rev
  newPos2 = enc2.read();
  if (newPos1 != pos1)
  {
    pos1 = newPos1;
  }
  if (newPos2 != pos2)
  {
    pos2 = newPos2;  
  }
  print_encoders();

  // read input from USB
  bytes_available = Serial.available();
  if (bytes_available)
  {
    ch = Serial.read();
    while (ch != ENDSTDIN)
    {
      incoming[idx++] = ch;
      
      // if the string ends or we run out of space, we're done with this string
      if (ch == CR || ch == NL || idx == (sizeof(incoming)-1))
      {
        incoming[idx-1] = 0; // terminate the string
        idx = 0;    // reset index

        // handle input for dispatch        
        status = handle_input(incoming);
        if (status)
        {
            Serial.print("$ERR Failed to process string: ");
            Serial.print(incoming);
            Serial.print("\n");
        }
        break;
      }
      ch = Serial.read();
    }
  }

  // attempt to maintain loop rate
  delay(1000 / hz); 
}

/**
 * @brief Parse input from Serial into commands, dispatch commands
 * 
 * @param incoming String to be parsed.
 * @return int EXIT_SUCCESS or EXIT_FAILURE
 */
int handle_input(char *incoming)
{
  // for tokenizing input string
  const char *delim = " ";
  char *token;

  int rpm1, rpm2;

  // tokenize string (strtok modifies the original string)
  token = strtok(incoming, delim);

  // switch on token
  if (strcmp(token, MSG_MOTORS) == 0) // MOTORS == "$MTR <RPM1> <RPM2>"
  {
    // unpack message to vars
    token = strtok(NULL, delim);
    rpm1 = atoi(token);
    token = strtok(NULL, delim);
    rpm2 = atoi(token);

    // send motor commands to motor controller
    send_command_SET_RPM(1, rpm1); // channel 1
    send_command_SET_RPM(2, rpm2); // channel 2
    
    return EXIT_SUCCESS;
  }
  else if (strcmp(token, MSG_ESTOP) == 0) // ESTOP == "$STP"
  {
    send_command_ESTOP();

    return EXIT_SUCCESS;
  }
  else if (strcmp(token, MSG_LIGHTS) == 0) // LIGHTS == "$????"
  {
    // ????????
    return EXIT_SUCCESS;
  }
  
  return 0;
}

/**
 * @brief Print the current value of RC pins to Jetson.
 *        format: $RCX <RJ_X> <RJ_Y> <LJ_Y> <LJ_X> <ESTOP> <DIAL> <AUTO>
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
 * @brief Print current encoder values. 
 *        format: $ENC <encoder1> <encoder2>
 * 
 */
void print_encoders()
{
  Serial.print("$ENC");
  Serial.print(" ");
  Serial.print(pos1 / 4); // convert counts to pulses; pulses = counts / 4
  Serial.print(" ");
  Serial.print(pos2 / 4); // convert counts to pulses; pulses = counts / 4
  Serial.print("\n");  
}

/**
 * @brief Read PPM data from a single pin using pulseIn()
 * 
 * @param pin pin to be read
 * @return String
 */
int get_input(int pin)
{
  unsigned long timeout = 100000; // 0.1 seconds
  
  // use pulseIn to turn PWM signal to values within a range
  return pulseIn(pin, HIGH, timeout);
}

/**
 * @brief Send E-STOP command to motor controller.
 * 
 */
void send_command_ESTOP()
{
  Serial3.println("!EX");
  // status = wait_for_response("expected response")
  // if (status)
  //  error handling
}

/**
 * @brief Send RPM command to a motor, then wait for ACK.
 * 
 * @param channel motor to change speed of.
 * @param RPM value to set RPM to.
 */
void send_command_SET_RPM(int channel, int RPM)
{
  // might have to use sprintf and a buffer, instead of char-by-char
  Serial3.print("!S");
  Serial3.print(" ");
  Serial3.print(channel);
  Serial3.print(" ");
  Serial3.print(RPM);
  Serial3.print("\n");
}

/**
 * @brief Slow motor to stop at configured "safety stop" rate.
 * 
 * @param channel motor to stop.
 */
void send_command_SAFETY_STOP(int channel)
{
  Serial3.print("!SFT");
  Serial3.print(" ");
  Serial3.print(channel);
  Serial3.print("\n");
}
