/*    
   This code is used to control the robotic arm based on the inputs from the
   Ras  pberry Pi. The Raspberry Pi will detect the recycle and will tell the Arduino
   to move the robotic arm in the location the object is detected at. Based on
   the material, the robotic arm will place the object in the correct loaction.
*/

// add Servo library
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


int numBlink = 0;
// create variables to store data from RPI

int distance = 1;
int material = 1;
int angle = 90;

// create variables to store servo posistions
int shoulderPos = 90;
int elbowPos = 90;
int wrist1Pos = 90;
int wrist2Pos = 110;
int handPos = 90;
int basePos = 90;


// create flash function (will flash Arduino LED by the number you input)
void flash(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}
int base = 0;
int shoulder = 4;
int elbow =8;
int wrist1 = 3;
int wrist2 = 15;
int hand = 1;
//homeState();

void setup() {
  // begin serial communication with RPI and send default string to RPI to confirm connection
  Serial.begin(9600);
  Serial.println("Connected to Arduino");
  
  // assign Arduino LED to pin 13
  pinMode(13, OUTPUT);
  

  

  pwm.begin(); // Initialize the PWM servo driver
  pwm.setPWMFreq(50); // Set the PWM frequency to 50 Hz

  homeState();
}

void loop() {
  // send default string to RPI to confirm connection
  Serial.println("Connected to Arduino");
  

  // if the RPI sends a signal to Arduino
  if (Serial.available()>0) {

    Serial.println("Connected to Arduino");
    // change values of variables to what the RPI sends
    numBlink = Serial.parseInt();
    material = Serial.parseInt();
    distance = Serial.parseInt();
    angle = Serial.parseInt();

//    numBlink = 1;
//    material = 1;
//    distance = 1;
//    angle = 90;

    Serial.print("Blink: ");
    Serial.println(numBlink);
    Serial.print("Material: ");
    Serial.println(material);
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("angle: ");
    Serial.println(angle);
    

    // flash LED and move the arm to pick and drop the object based on its material
   flash(numBlink);
   pickUp();
   dropOff();
   homeState();

    // send string to let RPI know that the arm is done moving
    Serial.println("Done Moving");
    Serial.flush();
  }
  delay(1000);
}

/*
   sweep function to make servos move slowly!
   input the servo object, the current servo angle, the angle you
   want the servo to move to, and the speed at which you want to turn
   the servo. Bigger number ==> slower speed. Smaller number ==> faster speed.
*/
void sweep(int servoNum, int oldPos, int newPos, int servoSpeed) {

  int pwmMin = 102;
  int pwmMax = 512;
  int pwmValue = map(newPos, 0, 180, pwmMin, pwmMax);

  if (oldPos <= newPos){
    for (oldPos; oldPos <= newPos; oldPos += 1){
      int pulse_width = map(oldPos, 0, 180, pwmMin, pwmMax);
      pwm.setPWM(servoNum, 0, pulse_width); // Set the pulse width
      delay(servoSpeed);
   }
  }

  else if (oldPos >= newPos){
    for (oldPos; oldPos >= newPos; oldPos -= 1){
      int pulse_width = map(oldPos, 0, 180, pwmMin, pwmMax);
      pwm.setPWM(servoNum, 0, pulse_width); // Set the pulse width
      delay(servoSpeed);
    }
  }
}

// pickUp function that will move robotic arm to specified distance and angle sent from RPI
void pickUp() {  
   // if the object detected is closest to the small/closest circle distance preset
  if (distance == 1) {
//    sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(wrist2, wrist2Pos, 20, 30);
//    wrist2Pos = 20;
//    sweep(shoulder, shoulderPos, 120, 30);
//    shoulderPos = 120;
//    sweep(elbow, elbowPos, 200, 30);
//    elbowPos = 200;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

    // sweep(base, basePos, angle, 30);
    // basePos = angle;
    // sweep(wrist2, wrist2Pos, 23, 30);
    // wrist2Pos = 23;
    // sweep(shoulder, shoulderPos, 100, 30);
    // shoulderPos = 100;
    // sweep(elbow, elbowPos, 180, 30);
    // elbowPos = 180;
    // sweep(hand, handPos, 90, 30);
    // handPos = 90;
    // sweep(hand, handPos, 0, 30);
    // handPos = 0;
    // distance = 0;

//    sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 35, 30);
//    wrist2Pos = 35;
//    sweep(elbow, elbowPos, 193, 30);
//    elbowPos = 193;
//    sweep(shoulder, shoulderPos, 110, 30);
//    shoulderPos = 110;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 43, 30);
    wrist2Pos = 43;
    sweep(elbow, elbowPos, 210, 30);
    elbowPos = 210;
    sweep(shoulder, shoulderPos, 120, 30);
    shoulderPos = 120;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
  }

  // if the object detected is between the smallest and medium circle distance preset
  else if (distance == 2) {   
//    sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 23, 30);
//    wrist2Pos = 23;
//    sweep(shoulder, shoulderPos, 100, 30);
//    shoulderPos = 100;
//    sweep(elbow, elbowPos, 185, 30);
//    elbowPos = 185;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

//    sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 25, 30);
//    wrist2Pos = 25;
//    sweep(elbow, elbowPos, 165, 30);
//    elbowPos = 165;
//    sweep(shoulder, shoulderPos, 80, 30);
//    shoulderPos = 80;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

 sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 35, 30);
    wrist2Pos = 35;
    sweep(elbow, elbowPos, 182, 30);
    elbowPos = 182;
    sweep(shoulder, shoulderPos, 90, 30);
    shoulderPos = 90;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
  }

  // if the object detected is closest to the medium circle distance preset
  else if (distance == 3) {
//   sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 27, 30);
//    wrist2Pos = 27;
//    sweep(elbow, elbowPos, 135, 30);
//    elbowPos = 135;
//    sweep(shoulder, shoulderPos, 60, 30);
//    shoulderPos = 60;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

//   sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 27, 30);
//    wrist2Pos = 27;
//    sweep(elbow, elbowPos, 155, 30);
//    elbowPos = 155;
//    sweep(shoulder, shoulderPos, 80, 30);
//    shoulderPos = 80;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

    sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 40, 30);
    wrist2Pos = 40;
    sweep(elbow, elbowPos, 159, 30);
    elbowPos = 159;
    sweep(shoulder, shoulderPos, 80, 30);
    shoulderPos = 80;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
  }

//  // if the object detected is between the medium and big circle distance preset
 else if (distance == 4) {

//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 27, 30);
//    wrist2Pos = 27;
//    sweep(elbow, elbowPos, 123, 30);
//    elbowPos = 123;
//    sweep(shoulder, shoulderPos, 53, 30);
//    shoulderPos = 53;
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

  sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 25, 30);
    wrist2Pos = 25;
    sweep(elbow, elbowPos, 125, 30);
    elbowPos = 125;
    sweep(shoulder, shoulderPos, 50, 30);
    shoulderPos = 50;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
 }
//
//  // if the object detected is closest to the big/farthest circle distance preset
//  else if (distance == 5) {
//    sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(hand, handPos, 160, 30);
//    handPos = 160;
//    sweep(wrist2, wrist2Pos, 32, 30);
//    wrist2Pos = 180;
//    sweep(elbow, elbowPos, 150, 30);
//    elbowPos = 150;
//    sweep(shoulder, shoulderPos, 45, 30);
//    shoulderPos = 45;
//    sweep(hand, handPos, 45, 30);
//    handPos = 45;
//    distance = 0;
//  }
}

// dropOff function that will drop off the object to a location based off its material
void dropOff() {
  /*
  if (material ==1 || material == 2 || material == 3 || material == 4 || material == 5) 
  {

    sweep (elbow, elbowPos, 0 ,30);
    elbowPos = 0;
    sweep(shoulder, shoulderPos, 90 ,30);
    shoulderPos =90;
    sweep (wrist2, wrist2Pos, 90 ,30);
    wrist2Pos = 90;
    sweep (base, basePos, 90 ,30);
    basePos = 90;
    sweep (hand, handPos, 90 ,30);
    handPos = 90;
    material = 0;
  }
  */


 //if the material detected was cardboard
  if (material == 1) {
    sweep(shoulder, shoulderPos, 90, 30);
    shoulderPos = 90;
    sweep(elbow, elbowPos, 0, 30);
    elbowPos = 0;
    sweep(wrist2, wrist2Pos, 90, 30);
    wrist2Pos = 90;
    sweep(base, basePos, 0, 30);
    basePos = 0;
    sweep(hand, handPos, 160, 30);
    handPos = 160;
    material = 0;
  }

  // if the material detected was glass
  else if (material == 2) {
    sweep(shoulder, shoulderPos, 90, 30);
    shoulderPos = 90;
    sweep(elbow, elbowPos, 0, 30);
    elbowPos = 0;
    sweep(wrist2, wrist2Pos, 90, 30);
    wrist2Pos = 90;
    sweep(base, basePos, 45, 30);
    basePos = 45;
    sweep(hand, handPos, 160, 30);
    handPos = 160;
    material = 0;
  }

  // if the material detected was metal
  else if (material == 3) {
    sweep(shoulder, shoulderPos, 90, 30);
    shoulderPos = 90;
    sweep(elbow, elbowPos, 0, 30);
    elbowPos = 0;
    sweep(wrist2, wrist2Pos, 90, 30);
    wrist2Pos = 90;
    sweep(base, basePos, 90, 30);
    basePos = 90;
    sweep(hand, handPos, 160, 30);
    handPos = 160;
    material = 0;
  }

  // if the material detected was paper
  else if (material == 4) {
    sweep(shoulder, shoulderPos, 90, 30);
    shoulderPos = 90;
    sweep(elbow, elbowPos, 0, 30);
    elbowPos = 0;
    sweep(wrist2, wrist2Pos, 90, 30);
    wrist2Pos = 90;
    sweep(base, basePos, 135, 30);
    basePos = 135;
    sweep(hand, handPos, 160, 30);
    handPos = 160;
    material = 0;
  }

  // if the material detected was plastic
  else if (material == 5) {
    sweep(shoulder, shoulderPos, 90, 30);
    shoulderPos = 90;
    sweep(elbow, elbowPos, 0, 30);
    elbowPos = 0;
    sweep(wrist2, wrist2Pos, 90, 30);
    wrist2Pos = 90;
    sweep(base, basePos, 180, 30);
    basePos = 180;
    sweep(hand, handPos, 160, 30);
    handPos = 160;
    material = 0;
  }
}

// homeState function that will hold the camera facing down to perform obejct detection
void homeState() {

  //original homestate
  
    sweep(base, basePos, 95, 30);
    basePos = 95;
    sweep(shoulder, shoulderPos, 108, 30);
    shoulderPos = 108;
    sweep(wrist2, wrist2Pos, 93, 30);
    wrist2Pos = 93;
    sweep(elbow, elbowPos, 177, 30);
    elbowPos = 177;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    

//distance 1
//    sweep(hand, handPos, 90, 30);
//    handPos = 90;
//    sweep(base, basePos, angle, 30);
//    basePos = angle;
//    sweep(wrist2, wrist2Pos, 43, 30);
//    wrist2Pos = 43;
//    sweep(elbow, elbowPos, 210, 30);
//    elbowPos = 210;
//    sweep(shoulder, shoulderPos, 120, 30);
//    shoulderPos = 120;
//    sweep(hand, handPos, 0, 30);
//    handPos = 0;
//    distance = 0;

//distance 2
    /*
    sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 40, 30);
    wrist2Pos = 40;
    sweep(elbow, elbowPos, 189, 30);
    elbowPos = 189;
    sweep(shoulder, shoulderPos, 100, 30);
    shoulderPos = 100;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
    */

  // distance 3
  /*
    sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 40, 30);
    wrist2Pos = 40;
    sweep(elbow, elbowPos, 159, 30);
    elbowPos = 159;
    sweep(shoulder, shoulderPos, 80, 30);
    shoulderPos = 80;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
    */

//distance 4
/*
    sweep(base, basePos, angle, 30);
    basePos = angle;
    sweep(wrist2, wrist2Pos, 25, 30);
    wrist2Pos = 25;
    sweep(elbow, elbowPos, 125, 30);
    elbowPos = 125;
    sweep(shoulder, shoulderPos, 60, 30);
    shoulderPos = 60;
    sweep(hand, handPos, 90, 30);
    handPos = 90;
    sweep(hand, handPos, 0, 30);
    handPos = 0;
    distance = 0;
    */
}
