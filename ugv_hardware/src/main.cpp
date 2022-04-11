#include <Arduino.h>

#define rx1 2
#define rx2 3
#define rx3 4
#define rx4 5
#define rx5 6
#define rx6 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(rx1, INPUT);
  pinMode(rx2, INPUT);
  pinMode(rx3, INPUT);
  pinMode(rx4, INPUT);
  pinMode(rx5, INPUT);
  pinMode(rx6, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print(pulseIn(rx1, HIGH));
  Serial.print("    ");
  Serial.print(pulseIn(rx2, HIGH));
  Serial.print("    ");
  Serial.print(pulseIn(rx3, HIGH));
  Serial.print("    ");
  Serial.print(pulseIn(rx4, HIGH));
  Serial.print("    ");
  Serial.print(pulseIn(rx5, HIGH));
  Serial.print("    ");
  Serial.println(pulseIn(rx6, HIGH));
}