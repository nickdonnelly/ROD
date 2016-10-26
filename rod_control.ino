// Arduino control software
// Written by: Nick Donnelly (0978802)
// 03 October 2016

// NOTE: All logging/printing functionality over Serial1 behaves oddly, do not use it.
// It forces the connection to be severed for some bizarre reason.

#include <Servo.h>
#include <Console.h>

// Constants
const int BAUDRATE_COMMUNICATION = 115200; // 115200 = wifi or ethernet, 9600 = USB

const int PIN_LED = 13; // Pin for the LED on-board the Yun

// Note the DC Motors have to use the PWM capable pins for speed control.
const int PIN_MOTOR_WHEEL_LEFT = 3;  
const int PIN_MOTOR_WHEEL_LEFT_REVERSE = 5;

const int PIN_MOTOR_WHEEL_RIGHT = 6;
const int PIN_MOTOR_WHEEL_RIGHT_REVERSE = 11;

const int PIN_SERVO_CLAWS_LEFT = A0; // these may be the same? Not sure.
const int PIN_SERVO_CLAWS_RIGHT = A1;
const int PIN_SERVO_RAMP = A2;

const int PIN_SERVO_CAMERA_X = A3; // the 2 axial motors for the camera
const int PIN_SERVO_CAMERA_Y = A4; 

int ROD_CRUISING_SPEED = 255; // tweak this. Our input system is quite rudimentary due most likely to
                                    // network configuration the course administrators wont let us change, so
                                    // this value needs to be correct.


// Servos
Servo servoLeftClaw;
Servo servoRightClaw;
Servo servoRamp;
Servo servoCameraX; // x axis
Servo servoCameraY; // y axis


// Other Globals
bool isTurningLeft = false;
bool isTurningRight = false;
bool isMovingForward = false;
bool isMovingBackward = false;
bool isRampUp = false;
bool clawsInUse = false;

int SERVO_LEFT_CLAW_POS = 0;
int SERVO_RIGHT_CLAW_POS = 0;
int SERVO_RAMP_POS = 0;
int SERVO_CAMERA_X_POS = 0;
int SERVO_CAMERA_Y_POS = 0;

void setup() {
  // Start the serial communication

  // Register the servos to the pins on the Arduino
  servoLeftClaw.attach(PIN_SERVO_CLAWS_LEFT);
  servoRightClaw.attach(PIN_SERVO_CLAWS_RIGHT);
  servoRamp.attach(PIN_SERVO_RAMP);
  servoCameraX.attach(PIN_SERVO_CAMERA_X);
  servoCameraY.attach(PIN_SERVO_CAMERA_Y);

  // Initialize the DC motors
  pinMode(PIN_MOTOR_WHEEL_LEFT, OUTPUT); // not sure these two lines are necessary.
  pinMode(PIN_MOTOR_WHEEL_RIGHT, OUTPUT);
  pinMode(PIN_LED, OUTPUT); // set the LED pin to output
  
  Serial1.begin(115200); // this should be "Serial" if using USB. Serial1 is for ethernet/wifi.

  digitalWrite(PIN_LED, HIGH); // turn on LED
  delay(60000); // wait 60 seconds to allow the connection to establish. This time may not need to be this long, but 
  
  
  // Finally, blink the LED so we know the arduino is done booting.
  digitalWrite(PIN_LED, LOW); // turn off LED
  delay(500);
  digitalWrite(PIN_LED, HIGH); // turn on LED
  delay(250);
  digitalWrite(PIN_LED, LOW); // turn on LED

  // Set the values to be intial.
  resetAllServos();
  updateServoValues();

  while(Serial1.available()){
    Serial1.read(); // Empty the input stream before operation begins.
  }
}

void loop() {
  if(Serial1.available() > 0){
    char incoming = Serial1.read();
    routeInput(incoming);
  }
}

void routeInput(char sym){
  switch(sym){
    case 'w': // wasd = forward, left, backward, right respectively
      toggleBackward();
      break;
    case 's':
      toggleForward();
      break;
    case 'a':
      toggleTurnLeft();
      break;
    case 'd':
      toggleTurnRight();
      break;

    case 'W': 
      toggleBackwardFast();
      break;
    case 'S':
      toggleForwardFast();
      break;
    case 'A':
      toggleTurnLeftFast();
      break;
    case 'D':
      toggleTurnRightFast();

    


    // TODO, make these movements incrememntal. need rod to test.
    case 'e': // ramp up/down
      toggleRamp();
      break;
    case 'c': // claws in
      toggleClaws();
      break;
    case 'x':
      toggleClawsOut();
      break;

    case 't':
      rampUp(); // 0
      break;
    case 'g':
      rampDown(); // 100
      break;
      

    case 'i': // ijkl are up, down, left, right camera respectively
      cameraUp();
      break;
    case 'k':
      cameraDown();
      break;
    case 'j':
      cameraLeft();
      break;
    case 'l':
      cameraRight();
      break;


    case 'o':
      digitalWrite(PIN_LED, HIGH);
      break;
    case 'p':
      digitalWrite(PIN_LED, LOW);
      break;
    }
}



// Secondary functions
void resetAllServos(){ //TODO: these values should be updated to reflect the correct values. The default might be 90?
  servoCameraX.write(100);
  servoCameraY.write(75);
}

void updateServoValues(){ // corrects the global variables for positions of the servos
  SERVO_LEFT_CLAW_POS = servoLeftClaw.read();
  SERVO_RIGHT_CLAW_POS = servoRightClaw.read();
  SERVO_RAMP_POS = servoRamp.read();
  SERVO_CAMERA_X_POS = servoCameraX.read();
  SERVO_CAMERA_Y_POS = servoCameraY.read();
}

int clamp(int in, int low, int high){ // clamps an input value inbetween two bounds inclusively
  if(in <= low) return low;
  if(in >= high) return high;
  return in;
}

void cameraUp(){
  int newVal = clamp(SERVO_CAMERA_Y_POS - 5, 0, 180);
  servoCameraY.write(newVal);
  updateServoValues();
}

void cameraDown(){
  int newVal = clamp(SERVO_CAMERA_Y_POS + 5, 0, 180);
  servoCameraY.write(newVal);
  updateServoValues();
}

void cameraLeft(){
  int newVal = clamp(SERVO_CAMERA_X_POS + 5, 0, 180);
  servoCameraX.write(newVal);
  updateServoValues();
}

void cameraRight(){
  int newVal = clamp(SERVO_CAMERA_X_POS - 5, 0, 180);
  servoCameraX.write(newVal);
  updateServoValues();
}

void rampDown(){
  int newVal = clamp(SERVO_RAMP_POS + 5, 0, 100);
//  servoRamp.write(newVal);
  moveServoGradual(servoRamp, 100, 100, 5);
  updateServoValues();
}

void rampUp(){
  int newVal = clamp(SERVO_RAMP_POS - 5, 0, 100);
//  servoRamp.write(newVal);
  moveServoGradual(servoRamp, 30, 100, 5);
  updateServoValues();
}

void toggleClaws(){
  servoLeftClaw.write(0);
  delay(1000);
  servoLeftClaw.write(90);
}

void toggleClawsOut(){
  servoLeftClaw.write(0);
}


void toggleRamp(){
  if(isRampUp){ // put ramp down
    servoRamp.write(0);
    isRampUp = false;
  }else{ // put ramp up
    servoRamp.write(45);
    isRampUp = true;
  }
}

void startBothMotors(bool forward){ // true = forward, false = reverse
  stopBothMotors();
  if(forward){
    startMotor(PIN_MOTOR_WHEEL_LEFT, ROD_CRUISING_SPEED);
    startMotor(PIN_MOTOR_WHEEL_RIGHT, ROD_CRUISING_SPEED);
    isMovingForward = true;
    isMovingBackward = false;
  }else{
    startMotor(PIN_MOTOR_WHEEL_LEFT_REVERSE, ROD_CRUISING_SPEED);
    startMotor(PIN_MOTOR_WHEEL_RIGHT_REVERSE, ROD_CRUISING_SPEED);
    isMovingForward = false;
    isMovingBackward = false;
  }
}

void toggleForward(){
  
  stopBothMotors();

  startBothMotors(false);
  delay(250);
  stopBothMotors();
}

void toggleBackward(){
  
  stopBothMotors();

  startBothMotors(true);
  delay(250);
  stopBothMotors();
}

void toggleForwardFast(){
  
  stopBothMotors();

  startBothMotors(false);
  delay(1000);
  stopBothMotors();
}

void toggleBackwardFast(){
  
  stopBothMotors();

  startBothMotors(true);
  delay(1000);
  stopBothMotors();
}

// start right wheel, stop left wheel
void toggleTurnLeft(){
  stopBothMotors();
  startMotor(PIN_MOTOR_WHEEL_RIGHT, 255);
  startMotor(PIN_MOTOR_WHEEL_LEFT_REVERSE, 255);
  delay(800);
  stopBothMotors();
}

void toggleTurnRight(){
  stopBothMotors();
  startMotor(PIN_MOTOR_WHEEL_LEFT, 255);
  startMotor(PIN_MOTOR_WHEEL_RIGHT_REVERSE, 255);
  delay(800);
  stopBothMotors();
}

void toggleTurnLeftFast(){
  stopBothMotors();
  startMotor(PIN_MOTOR_WHEEL_RIGHT, 127);
  delay(500);
  stopMotor("right");
}

void toggleTurnRightFast(){
  stopBothMotors();
  startMotor(PIN_MOTOR_WHEEL_LEFT, 127);
  delay(500);
  stopMotor("left");
}

void increaseCruisingSpeed(){
  if(ROD_CRUISING_SPEED >= 255){
    return;
  }else{
    ROD_CRUISING_SPEED += 5; 
    if(ROD_CRUISING_SPEED > 255) ROD_CRUISING_SPEED = 255; // does arduino have clamp? who knows
  }
}

void decreaseCruisingSpeed(){
  if(ROD_CRUISING_SPEED < 100){
    return;
  }else{
    ROD_CRUISING_SPEED -= 5;
  }
}

// Move a single motor at a specified speed
void startMotor(int pin, int motorSpeed){
  analogWrite(pin, motorSpeed);
}

// Stop moving the motor on the specified pin
void stopMotor(String motor){ // left or right
  if(motor == "left"){
    analogWrite(PIN_MOTOR_WHEEL_LEFT, 0);
    analogWrite(PIN_MOTOR_WHEEL_LEFT_REVERSE, 0);
  }else if(motor == "right"){
    analogWrite(PIN_MOTOR_WHEEL_RIGHT, 0);
    analogWrite(PIN_MOTOR_WHEEL_RIGHT_REVERSE, 0);
  }
}

void stopBothMotors(){
  analogWrite(PIN_MOTOR_WHEEL_LEFT, 0);
  analogWrite(PIN_MOTOR_WHEEL_RIGHT, 0);
  analogWrite(PIN_MOTOR_WHEEL_LEFT_REVERSE, 0);
  analogWrite(PIN_MOTOR_WHEEL_RIGHT_REVERSE, 0);
  isTurningLeft = false;
  isTurningRight = false;
  isMovingForward = false;
  isMovingBackward = false;
}

// does the same as moveServo, but in small steps for smoother movement.
// Make sure that degreeStep is a value that will, after a series of steps,
// land exactly on the desired position
void moveServoGradual(Servo servo, int position, int delayBtwnMoves, int degreeStep){
  if(!servo.attached()){
    return;
  }
  int currPos = servo.read();
  while(currPos != position){
    
    if(abs(currPos - position) <= degreeStep){
      servo.write(position); // move directly to position, we are already close
      delay(delayBtwnMoves); // delay, log, break from loop.
      break;
    }


    // This if block makes a small movement
    if(currPos > position){
      servo.write(currPos - degreeStep); // move down the degree
    }else if(currPos < position){
      servo.write(currPos + degreeStep); // move up the degree
    }else{
      break; // exit loop entirely, we are already at the correct place
    }
    delay(delayBtwnMoves); // delay, update the current position, restart loop.
    currPos = servo.read();
  }
  updateServoValues();
}



