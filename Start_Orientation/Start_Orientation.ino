#include "Enes100.h"
#include "Tank.h"

// Motor Pin Definitions
#define PIN_MOTOR_1_REVERSE 11 //FLD1
#define PIN_MOTOR_1_FORWARD 10 //FLD2
#define PIN_MOTOR_2_REVERSE 9  //BLD1
#define PIN_MOTOR_2_FORWARD 8  //BLD2
#define PIN_MOTOR_3_FORWARD 4  //FRD1
#define PIN_MOTOR_3_REVERSE 5  //FRD2
#define PIN_MOTOR_4_FORWARD 2  //BRD1
#define PIN_MOTOR_4_REVERSE 3  //BRD2

// Globals
float x, y, t;
bool v;

// Motor Control Functions
void motorCW(int m1, int m2) {
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
}

void motorCCW(int m1, int m2) {
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
}

void motorStop(int m1, int m2) {
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
}

void driveForward() {
  motorCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorCCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorCCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void driveBackwards() {
  motorCCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorCCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void strafeLeft() {
  motorCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorCCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorCCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void strafeRight() {
  motorCCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorCCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void rotateCW() {
  motorCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void rotateCCW() {
  motorCCW(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorCCW(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorCCW(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorCCW(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void stopotv() {
  motorStop(PIN_MOTOR_3_FORWARD, PIN_MOTOR_3_REVERSE);
  motorStop(PIN_MOTOR_1_FORWARD, PIN_MOTOR_1_REVERSE);
  motorStop(PIN_MOTOR_4_FORWARD, PIN_MOTOR_4_REVERSE);
  motorStop(PIN_MOTOR_2_FORWARD, PIN_MOTOR_2_REVERSE);
}

void setup() {
  Tank.begin();
  Enes100.begin("Work in Progress", MATERIAL, 15, 1116, 52, 50);
  delay(200);
  Enes100.println("Successfully connected to the Vision System");
}


void startOrientation() {
  x = Enes100.getX();
  y = Enes100.getY();
  t = Enes100.getTheta();
  v = Enes100.isVisible();

  if (x == 0.55 && y == 0.55) {
    while (t != 1.57079632679) {
      rotateCCW();
      t = Enes100.getTheta();
    }
    driveForward();
    delay(7000);
  } else if (x == 0.55 && y == 1.45) {
    while (t != -1.57079632679) {
      rotateCCW();
      t = Enes100.getTheta();
    }
    driveForward();
    delay(7000);
  }
}

void loop() {
  x = Enes100.getX();
  y = Enes100.getY();
  t = Enes100.getTheta();
  v = Enes100.isVisible();

  if (v) {
    Enes100.print(x);
    Enes100.print(",");
    Enes100.print(y);
    Enes100.print(",");
    Enes100.println(t);

    startOrientation(); // Call this if you want it triggered here
  } else {
    Enes100.println("Not visible");
  }
}
