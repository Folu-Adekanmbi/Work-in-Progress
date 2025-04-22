#include "Enes100.h"

//pinouts update as needed
const int FL_IN1 = 2, FL_IN2 = 3, FL_EN = 9; //FL - front left
const int FR_IN1 = 4, FR_IN2 = 5, FR_EN = 10; //FR - front right
const int BL_IN1 = 6, BL_IN2 = 7, BL_EN = 11;  //BL - back left
const int BR_IN1 = 8, BR_IN2 = 12, BR_EN = 13; //BR - back right

// PD tuning constants
const float kP_position = 1.0;
const float kD_position = 0.2;
const float kP_theta = 1.0;
const float kD_theta = 0.2;

const float positionTolerance = 0.05;
const float thetaTolerance = 0.05;
const unsigned long updateInterval = 20; // ms

//power should be given in range [-1, 1] which corresponds to max speed and minimum speed, 0 is stop
void setMotorPower(String motor, float power) { //Input motor name either FL, FR, BL, BR and power calculated from set driver power
  // Constrain the power to be between -1 and 1
  power = constrain(power, -1.0, 1.0);
  int pwm = (int)(fabs(power) * 255);  // Scale to PWM range

  if (motor == "FL") {
    // Front Left motor
    if (power > 0) {
      digitalWrite(FL_IN1, HIGH);
      digitalWrite(FL_IN2, LOW);
    } else if (power < 0) {
      digitalWrite(FL_IN1, LOW);
      digitalWrite(FL_IN2, HIGH);
    } else {
      digitalWrite(FL_IN1, LOW);
      digitalWrite(FL_IN2, LOW);
    }
    analogWrite(FL_EN, pwm);

  } else if (motor == "FR") {
    // Front Right motor
    if (power > 0) {
      digitalWrite(FR_IN1, HIGH);
      digitalWrite(FR_IN2, LOW);
    } else if (power < 0) {
      digitalWrite(FR_IN1, LOW);
      digitalWrite(FR_IN2, HIGH);
    } else {
      digitalWrite(FR_IN1, LOW);
      digitalWrite(FR_IN2, LOW);
    }
    analogWrite(FR_EN, pwm);

  } else if (motor == "BL") {
    // Back Left motor
    if (power > 0) {
      digitalWrite(BL_IN1, HIGH);
      digitalWrite(BL_IN2, LOW);
    } else if (power < 0) {
      digitalWrite(BL_IN1, LOW);
      digitalWrite(BL_IN2, HIGH);
    } else {
      digitalWrite(BL_IN1, LOW);
      digitalWrite(BL_IN2, LOW);
    }
    analogWrite(BL_EN, pwm);

  } else if (motor == "BR") {
    // Back Right motor
    if (power > 0) {
      digitalWrite(BR_IN1, HIGH);
      digitalWrite(BR_IN2, LOW);
    } else if (power < 0) {
      digitalWrite(BR_IN1, LOW);
      digitalWrite(BR_IN2, HIGH);
    } else {
      digitalWrite(BR_IN1, LOW);
      digitalWrite(BR_IN2, LOW);
    }
    analogWrite(BR_EN, pwm);
  }
}

void setDrivePower(float vx, float vy, float omega) { //given velotiy for x y and theta set drive power of each motor
  float frontLeft  = vx + vy + omega; //update after looking at wheel config
  float frontRight = vx - vy - omega;
  float backLeft   = vx - vy + omega;
  float backRight  = vx + vy - omega;

  // Normalize speeds but keep them proportional
  float maxMagnitude = max(
    1.0,
    max(fabs(frontLeft), max(fabs(frontRight), max(fabs(backLeft), fabs(backRight))))
  );

  // Normalize to keep values within [-1, 1]
  frontLeft  /= maxMagnitude;
  frontRight /= maxMagnitude;
  backLeft   /= maxMagnitude;
  backRight  /= maxMagnitude;

  // Send powers to motors (replace with your motor commands)
  setMotorPower("FL", frontLeft);
  setMotorPower("FR", frontRight);
  setMotorPower("BL", backLeft);
  setMotorPower("BR", backRight);
}


float normalizeAngle(float angle) { //use normalize angle to get optimal pathing
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

void driveTo(float xTarget, float yTarget, float thetaTarget) { //drives to give x y and theta target using PD controllers and tolerances above
  float prevErrorX = 0;
  float prevErrorY = 0;
  float prevErrorTheta = 0;

  unsigned long lastUpdate = millis();

  while (true) {
    unsigned long now = millis();
    if (now - lastUpdate < updateInterval) {
      continue; // Don't update yet, but keep looping
    }

    float dt = (now - lastUpdate) / 1000.0; // convert to seconds
    lastUpdate = now;

    // Get current position
    float x = Enes100.getX();
    float y = Enes100.getY();
    float theta = Enes100.getTheta(); // in radians

    // Calculate errors
    float errorX = xTarget - x;
    float errorY = yTarget - y;
    float errorTheta = normalizeAngle(thetaTarget - theta);

    // Check if we're within tolerance
    if (fabs(errorX) < positionTolerance &&
        fabs(errorY) < positionTolerance &&
        fabs(errorTheta) < thetaTolerance) {
      setDrivePower(0, 0, 0);
      break;
    }

    // Derivatives
    float dErrorX = (errorX - prevErrorX) / dt;
    float dErrorY = (errorY - prevErrorY) / dt;
    float dErrorTheta = (errorTheta - prevErrorTheta) / dt;

    // PD control in field space
    float vxField = kP_position * errorX + kD_position * dErrorX;
    float vyField = kP_position * errorY + kD_position * dErrorY;

    // Convert to robot frame
    float cosTheta = cos(-theta);
    float sinTheta = sin(-theta);
    float vxRobot = vxField * cosTheta - vyField * sinTheta;
    float vyRobot = vxField * sinTheta + vyField * cosTheta;

    // Angular velocity
    float omega = kP_theta * errorTheta + kD_theta * dErrorTheta;

    // Clamp outputs
    vxRobot = constrain(vxRobot, -1.0, 1.0);
    vyRobot = constrain(vyRobot, -1.0, 1.0);
    omega = constrain(omega, -1.0, 1.0);

    // Send power to drive system
    setDrivePower(vxRobot, vyRobot, omega);

    //save values for next loop
    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorTheta = errorTheta;
  }
}
void setup() {
  Enes100.begin("Work in Progress", MATERIAL, 15, 1116, 52, 50);
  delay(200);
  Enes100.println("Successfully connected to the Vision System");
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_EN, OUTPUT);
  pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT); pinMode(BL_EN, OUTPUT);
  pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT); pinMode(BR_EN, OUTPUT);
}

void loop() {

}
  