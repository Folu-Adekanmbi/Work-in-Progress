//F is front, B is back, R is right, L is left, D is drive, S is speed for these pin assignments
#define FRD1 5
#define FRD2 6
#define FLD1 12
#define FLD2 13
#define BRD1 7
#define BRD2 8
#define BLD1 10
#define BLD2 11
#define S 3

void motorCW (int m1, int m2) {
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(S, HIGH);
}

void motorCCW (int m1, int m2) {
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(S, HIGH);
}

void motorStop (int m1, int m2) {
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(S, LOW);
}

void driveForwards () {
motorCW(FRD1,FRD2);
motorCCW(FLD1,FLD2);
motorCW(BRD1,BRD2);
motorCCW(BLD1,BLD2);
}
void driveBackwards () {
motorCCW(FRD1,FRD2);
motorCW(FLD1,FLD2);
motorCCW(BRD1,BRD2);
motorCW(BLD1,BLD2);
}
void strafeLeft () {
motorCW(FRD1,FRD2);
motorCCW(FLD1,FLD2);
motorCCW(BRD1,BRD2);
motorCW(BLD1,BLD2);
// check if it makes sense plss
}

void strafeRight () {
motorCCW(FRD1, FRD2);
motorCW(FLD1, FLD2);
motorCW(BRD1, BRD2);
motorCCW(BLD1, BLD2);
}

void rotateCW () {
motorCW(FRD1, FRD2);
motorCW(FLD1, FLD2);
motorCW(BRD1, BRD2);
motorCW(BLD1, BLD2);
}

void rotateCCW () {
motorCCW(FRD1, FRD2);
motorCCW(FLD1, FLD2);
motorCCW(BRD1, BRD2);
motorCCW(BLD1, BLD2);
}

void stopotv () {
motorStop(FRD1, FRD2);
motorStop(FLD1, FLD2);
motorStop(BRD1, BRD2);
motorStop(BLD1, BLD2);
}

void setup() {
pinMode (FRD1, OUTPUT);
pinMode (FRD2, OUTPUT);
pinMode (BRD1, OUTPUT);
pinMode (BRD2, OUTPUT);
pinMode (FLD1, OUTPUT);
pinMode (FLD2, OUTPUT);
pinMode (BLD1, OUTPUT);
pinMode (BLD2, OUTPUT);
pinMode (S, OUTPUT);
driveForwards (); // Show forward locomotion
}

void loop() {
  digitalWrite(S, HIGH);
 driveForwards (); // Show forward locomotion
  delay(30000); // time to get to the other end of the arena under 3 minutes
// //   driveBackwards ();
// //   delay(5000);
// //   strafeLeft ();
// //   delay(5000);
// //   strafeRight ();
// //   delay(5000);
// //   rotateCW ();
// //   delay(5000);
// //   rotateCCW ();
// //   delay(5000);
// //   stopotv();
// //   delay(100000);
}
  
