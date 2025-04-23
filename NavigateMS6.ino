#include "Enes100.h"

//pinouts update as needed
const int FL_IN1 = 13, FL_IN2 = 12, FL_EN = 11; //FL - front left
const int FR_IN1 = 2, FR_IN2 = 4, FR_EN = 3; //FR - front right
const int BL_IN1 = 9, BL_IN2 = 8, BL_EN = 10;  //BL - back left
const int BR_IN1 = 7, BR_IN2 = 6, BR_EN = 5; //BR - back right

// PD tuning constants
const float kP_position = 2;
const float kD_position = 0.1;
const float kP_theta = 0;
const float kD_theta = 0;
int z = 0;

const float positionTolerance = 0.05;
const float thetaTolerance = 0.05;
const unsigned long updateInterval = 20; // ms

//power should be given in range [-1, 1] which corresponds to max speed and minimum speed, 0 is stop
void setMotorPower(int motor, float power) { //Input motor name either FL, FR, BL, BR and power calculated from set driver power
  // Constrain the power to be between -1 and 1
  power = constrain(power, -1.0, 1.0);
  int pwm = (int)(fabs(power) * 255);  // Scale to PWM range

  if (motor == 0) { //FL
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

  } else if (motor == 1) { //FR
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

  } else if (motor == 2) { //BL
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

  } else if (motor == 3) { //BR
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
  float maxMagnitude = max( 1.0, max(fabs(frontLeft), max(fabs(frontRight), max(fabs(backLeft), fabs(backRight)))));

  Serial.println(frontLeft);
  Serial.println(frontRight);
  Serial.println(backLeft);
  Serial.println(backRight);



  // Normalize to keep values within [-1, 1]
  frontLeft  /= maxMagnitude;
  frontRight /= maxMagnitude;
  backLeft   /= maxMagnitude;
  backRight  /= maxMagnitude;

  Serial.println(frontLeft);
  Serial.println(frontRight);
  Serial.println(backLeft);
  Serial.println(backRight);

  // Send powers to motors (replace with your motor commands)
  setMotorPower(0, frontLeft);
  setMotorPower(1, frontRight);
  setMotorPower(2, backLeft);
  setMotorPower(3, backRight);
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
        fabs(errorY) < positionTolerance /*&&
        fabs(errorTheta) < thetaTolerance)*/ ){
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
    float vyRobot = -(vxField * sinTheta + vyField * cosTheta);


    // Angular velocity
    float omega = kP_theta * errorTheta + kD_theta * dErrorTheta;

    // Clamp outputs
    vxRobot = constrain(vxRobot, -1.0, 1.0);
    vyRobot = constrain(vyRobot, -1.0, 1.0);
    omega = constrain(omega, -1.0, 1.0);
    if(z == 100){
    Enes100.print("vx: ");
    Enes100.println(vxRobot);
    Enes100.print("vy: ");

    Enes100.println(vyRobot);
    Enes100.print("omega: ");

    Enes100.println(omega);
    z = 0;
    }
    else{
      z++;
    }



    // Send power to drive system
    setDrivePower(vxRobot, vyRobot, omega);

    //save values for next loop
    prevErrorX = errorX;
    prevErrorY = errorY;
    prevErrorTheta = normalizeAngle(errorTheta);
  }
}
void setup() {
 Enes100.begin("Work in Progress", MATERIAL, 495, 1116, 15, 16);
  delay(200);
  Enes100.println("Successfully connected to the Vision System"); 
  //pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
  pinMode(11, OUTPUT); pinMode(13, OUTPUT); pinMode(12, OUTPUT);
  pinMode(10, OUTPUT); pinMode(9, OUTPUT); pinMode(8, OUTPUT);
    pinMode(2, OUTPUT); pinMode(3, OUTPUT); pinMode(4, OUTPUT);
        pinMode(5, OUTPUT); pinMode(6, OUTPUT); pinMode(7, OUTPUT);


  //pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT); pinMode(BL_EN, OUTPUT);
  //pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT); pinMode(BR_EN, OUTPUT);
  Serial.begin(9600);
  Serial.println("succesful begin");
}

void loop() {
  //delay(3000);
 //setMotorPower(0, -1);
 // setMotorPower(1, 1);
 //setDrivePower(1,-1,0);d
   driveTo(0.89,1.98,-PI/2);
 /* digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, HIGH);
  analogWrite(FL_EN, 255);
*/
Enes100.println("I'm Done ");
delay(500);
   driveTo(2.71,1.98,-PI/2);




delay(3000000000000000000);

//  delay(100000000000);


}
  