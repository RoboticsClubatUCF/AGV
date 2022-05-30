/**
 * @file RXPWM_to_arduino.cpp
 * @author Wesley Fletcher, Tevin Mukudi, Marc Simmonds
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
#include <FastLED.h>

// RC Receiver
#define RC_RJ_x   2  // right joystick, x-axis - pin 6 on receiver
#define RC_RJ_y   3  // right joystick, y-axis - pin # on receiver
#define RC_LJ_y   4  // left joystick, y-axis - pin # on receiver
#define RC_LJ_x   5  // left joystick, x-axis - pin # on receiver
#define RC_ESTOP  6  // e-stop switch
#define RC_DIAL   7  // dial
#define RC_AUTO   8  // Auto-nav switch - pin 7 on receiver 
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
// Pins for relays 
#define R1        10
#define R2        11
#define R3        12
#define R4        13
// for LEDs
#define LED_PIN   9
#define NUM_LEDS  5
static const char * LIT_FLASH = "X_X\0";
static const char * LIT_SOLID = "XXX\0";
 
// Jetson<-->Arduino message definitions
static const char * MSG_MOTORS = "$MTR\0";      // command for motor controller
static const char * MSG_CMD    = "$CMD\0";      // generic command, passed to SBC through serial
static const char * MSG_ACK    = "$ACK\0";      // an acknowledgement that a message was received
static const char * MSG_ESTOP_ON  = "$STP\0";      // throw the motor controller E-STOP
static const char * MSG_ESTOP_OFF = "$GO\0";
static const char * MSG_LIGHTS = "$LIT\0";      // TODO: control the lights

// function prototypes
int handle_input(char *incoming);
int get_input(int pin);
void print_rc_inputs();
void print_encoders();
void throw_ESTOP();
void release_ESTOP();
void send_command_SET_RPM(int channel, int RPM);
void send_command_SAFETY_STOP(int channel);

//////////////////////
///// Globals ////////
//////////////////////

int hz = 30;  // max/ideal loop rate

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

// LEDs
CRGB leds[NUM_LEDS];
bool flash = False;
int light_freq = hz / 4;
int light_count = 0;

/**
 * @brief Run once.
 * 
 */
void setup()
{  
  // set-up serial connections
  Serial.begin(115200);   // USB-to-Jetson
  Serial3.begin(115200);  // to motor controller

  FastLED.addLEDs<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  fill_solid(leds, NUM_LEDS, CRGB(0, 255, 13)); // turn LED's back to STANDBY color
  fastLED.show();

  // configure RC pins as inputs
  pinMode(RC_RJ_x, INPUT);
  pinMode(RC_RJ_y, INPUT);
  pinMode(RC_LJ_y, INPUT);
  pinMode(RC_LJ_x, INPUT);
  pinMode(RC_ESTOP, INPUT);
  pinMode(RC_DIAL, INPUT);
  pinMode(MC_TX, OUTPUT);
  pinMode(MC_RX, INPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(R3, OUTPUT);
  pinMode(R4, OUTPUT);
  Serial3.print("!EX\n\r");
  digitalWrite(R1,HIGH);
  digitalWrite(R2,HIGH);
  digitalWrite(R3,HIGH);
  digitalWrite(R4,HIGH);
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
//  Serial3.print("!EX\r");

  // read encoders, print to Jetson
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
//  print_encoders();

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

  // if lights are set to flash
  if (flash)
  {
    light_count++;
    if ((light_count % (light_freq * 2)) == 0)
    {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
    }
    else if ((light_count % light_freq) == 0)
    {
      fill_solid(leds, NUM_LEDS, CRGB(255, 98, 0));
    }
    fastLED.show();
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

  Serial3.print(incoming);
  Serial3.print("\n\r");

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
  else if (strcmp(token, MSG_ESTOP_ON) == 0) // ESTOP == "$STP"
  {
    throw_ESTOP();
    
    // set lights
    fill_solid(leds, NUM_LEDS, CRGB::Red); // turn E-STOP LEDs on
    fastLED.show();

    return EXIT_SUCCESS;
  }
  else if (strcmp(token, MSG_ESTOP_OFF) == 0) // ESTOP == "$GO"
  {
    release_ESTOP();

    // set lights
    fill_solid(leds, NUM_LEDS, CRGB(0, 255, 13)); // turn LED's back to STANDBY color
    fastLED.show();

    return EXIT_SUCCESS;
  }
  else if (strcmp(token, MSG_LIGHTS) == 0) // LIGHTS == "$LIT <XXX (solid)/X_X (blink>"
  {
    token = strtok(NULL, delim);
    if (strcmp(token, LIT_SOLID) == 0)
    {
      flash = false;
      fill_solid(leds, NUM_LEDS, CRGB(0, 255, 13)); // turn LEDs on
      fastLED.show();
    }
    else if (strcmp(token, LIT_FLASH) == 0)
    {
      flash = true;
    }
    return EXIT_SUCCESS;
  }
  
  return 0;
}

/**
 * @brief Print the current value of RC pins to Jetson.
 *        format: $RCX <RJ_X> <RJ_Y> <LJ_Y> <LJ_X> <ESTOP> <DIAL>
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
  Serial.print(" ");
  Serial.print(get_input(RC_AUTO));
  Serial.print("\n\r");
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
  Serial.print("\n\r");  
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
 * @brief Activate relay to stop the motors.
 * 
 */
void throw_ESTOP()
{
  digitalWrite(R4,LOW); 
}

/**
 * @brief Deactivate relays to stop the motors.
 * 
 */
void release_ESTOP()
{
  digitalWrite(R4,HIGH); 
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
  Serial3.print("\n\r");
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
  Serial3.print("\n\r");
  
}
