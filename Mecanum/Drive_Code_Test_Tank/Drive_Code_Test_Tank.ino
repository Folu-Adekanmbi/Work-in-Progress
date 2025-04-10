#include <Enes100.h>

#include "Tank.h"
#define PIN_MOTOR_1_REVERSE 11 //FLD1
#define PIN_MOTOR_1_FORWARD 10 //FLD2
#define PIN_MOTOR_2_REVERSE 9 //BLD1
#define PIN_MOTOR_2_FORWARD 8 //BLD2
#define PIN_MOTOR_3_FORWARD 4 //FRD1
#define PIN_MOTOR_3_REVERSE 5 //FRD2
#define PIN_MOTOR_4_FORWARD 2 //BRD1
#define PIN_MOTOR_4_REVERSE 3 //BRD2

void motorCW (int m1, int m2) {
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
// controlling speed with PWM

}

void motorCCW (int m1, int m2) {
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
// controlling speed

}

void motorStop (int m1, int m2) {
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
}

void driveForward () {  
motorCW(PIN_MOTOR_3_FORWARD,PIN_MOTOR_3_REVERSE);
motorCCW(PIN_MOTOR_1_FORWARD,PIN_MOTOR_1_REVERSE);
motorCW(PIN_MOTOR_4_FORWARD,PIN_MOTOR_4_REVERSE);
motorCCW(PIN_MOTOR_2_FORWARD,PIN_MOTOR_2_REVERSE);
}
void driveBackwards () { 
motorCCW(PIN_MOTOR_3_FORWARD,PIN_MOTOR_3_REVERSE);
motorCW(PIN_MOTOR_1_FORWARD,PIN_MOTOR_1_REVERSE);
motorCCW(PIN_MOTOR_4_FORWARD,PIN_MOTOR_4_REVERSE);
motorCW(PIN_MOTOR_2_FORWARD,PIN_MOTOR_2_REVERSE);
}
void strafeLeft () {
motorCW(PIN_MOTOR_3_FORWARD,PIN_MOTOR_3_REVERSE);
motorCW(PIN_MOTOR_1_FORWARD,PIN_MOTOR_1_REVERSE);
motorCCW(PIN_MOTOR_4_FORWARD,PIN_MOTOR_4_REVERSE);
motorCCW(PIN_MOTOR_2_FORWARD,PIN_MOTOR_2_REVERSE);
// check if it makes sense plss
}

void strafeRight () {
motorCCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
motorCCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
motorCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
motorCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void rotateCW () { // this is actually counter clockwise
motorCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
motorCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
motorCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
motorCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void rotateCCW () { // this is clockwise 
motorCCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
motorCCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
motorCCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
motorCCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void stopotv () {
motorStop(PIN_MOTOR_3_FORWARD,PIN_MOTOR_3_REVERSE);
motorStop(PIN_MOTOR_1_FORWARD,PIN_MOTOR_1_REVERSE);
motorStop(PIN_MOTOR_4_FORWARD,PIN_MOTOR_4_REVERSE);
motorStop(PIN_MOTOR_2_FORWARD,PIN_MOTOR_2_REVERSE);
}

void setup() {
  Tank.begin(); // Must call in order to use any Tank function

}

void loop() {
  driveForward (); // Show forward locomotion
  delay(18000); // time to get to the other end of the arena under 3 minutes
  driveBackwards ();
  delay(5000);
  strafeLeft ();
  delay(5000);
  strafeRight ();
  delay(5000);
  rotateCW ();
  delay(5000);
  rotateCCW ();
  delay(5000);
  stopotv();
  delay(100000);
}
  


