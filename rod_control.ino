// Arduino control software
// Written by: Nick Donnelly (0978802)
// 03 October 2016

#include <Servo.h>

// Constants
const int BITRATE_COMMUNICATION = 115200; // 115200 = wifi or ethernet, 9600 = USB

const int PIN_LED = 13; // Pin for the LED on-board the Yun

const int PIN_MOTOR_WHEEL_LEFT = 0; // these are the pins for the motors, don't forget
const int PIN_MOTOR_WHEEL_RIGHT = 0; // to change these values to the correct ones later.

const int PIN_SERVO_RAMP = 0;
const int PIN_SERVO_CLAWS_LEFT = 0; // these may be the same? Not sure.
const int PIN_SERVO_CLAWS_RIGHT = 0;

const int PIN_SERVO_CAMERA_X = 0; // the 2 axial motors for the camera
const int PIN_SERVO_CAMERA_Y = 0; 


// Servos
Servo servoLeftClaw;
Servo servoRightClaw;
Servo servoRamp;
Servo servoCameraX; // x axis
Servo servoCameraY; // y axis


// Other Globals
bool servoIsMoving = false;
bool motorIsMoving = false; // these are on-the-fly updated.

int SERVO_LEFT_CLAW_POS = 0;
int SERVO_RIGHTT_CLAW_POS = 0;
int SERVO_RAMP_POS = 0;
int SERVO_CAMERA_X_POS = 0;
int SERVO_CAMERA_Y_POS = 0;

void setup() {
  // Start the serial communication
  Serial1.begin(BITRATE_COMMUNICATION); // this should be "Serial" if using USB. Serial1 is for ethernet/wifi.

  // Register the servos to the pins on the Arduino
  servoLeftClaw.attach(PIN_SERVO_CLAWS_LEFT);
  servoRightClaw.attach(PIN_SERVO_CLAWS_RIGHT);
  servoRamp.attach(PIN_SERVO_RAMP);
  servoCameraX.attach(PIN_SERVO_CAMERA_X);
  servoCameraY.attach(PIN_SERVO_CAMERA_Y);

  delay(50000); // wait 50 seconds to allow the connection to establish. This time may not need to be this long, but 
                // I am unable to test it at the moment.

  while(Serial.available()){
    Serial1.read(); // Empty the input stream before operation begins.
  }
  
  // Finally, blink the LED so we know the arduino is done booting.
  pinMode(PIN_LED, OUTPUT); // set the LED pin to output
  digitalWrite(PIN_LED, HIGH); // turn on LED
  delay(5000); // wait 5 seconds
  digitalWrite(PIN_LED, LOW); // turn off LED

  // Set the values to be intial.
  resetAllServos();
  updateServoValues();
}

void loop() {
  // TODO: read the input constantly, when messages come from servo, make appropriate state change.

}


// Secondary functions
void resetAllServos(){ // these values should be updated to reflect the correct values.
  servoIsMoving = true;
  servoLeftClaw.write(0);
  servoRightClaw.write(0);
  servoRamp.write(0);
  servoCameraX.write(0);
  servoCameraY.write(0);
  delay(500);
  servoIsMoving = false;
  netLog("[SERVO] All servos reset to initial positions.");
}

void updateServoValues(){ // corrects the global variables for positions of the servos
  SERVO_LEFT_CLAW_POS = servoLeftClaw.read();
  SERVO_RIGHTT_CLAW_POS = servoRightClaw.read();
  SERVO_RAMP_POS = servoRamp.read();
  SERVO_CAMERA_X_POS = servoCameraX.read();
  SERVO_CAMERA_Y_POS = servoCameraY.read();
  netLog("[SERVO] All servo position values updated.");
}

void moveServo(Servo servo, int position){
  if(!servo.attached()) {  // exit if the servo isn't correctly attached
    netLog("[SERVO] Attempted to move servo that wasn't attached!");
    return;
  }
  servoIsMoving = true; // set global flag
  int currPos = servo.read(); // gets the current position value of the servo (0-180);
  if(position > 180 || position < 0){
    netLog("[SERVO] Position value must be between 0-180.");
    return;
  }
  servo.write(position); // move the servo
  delay(500); // wait 1/2 second to allow servo to move
  servoIsMoving = false; // reset global flag
  updateServoValues();
  netLog("[SERVO] Servo movement complete.");
}


// does the same as moveServo, but in small steps for smoother movement.
// Make sure that degreeStep is a value that will, after a series of steps,
// land exactly on the desired position
void moveServoGradual(Servo servo, int position, int delayBtwnMoves, int degreeStep){
  if(!servo.attached()){
    netLog("[SERVO] Attempted to move servo that wasn't attached!");
    return;
  }
  servoIsMoving = true;
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
  servoIsMoving = false;
  updateServoValues();
  netLog("[SERVO] Servo movement complete.");
}

// Logs various messages to the Serial1 output.
void netLog(char logMessage[]){
  // print this to the console of the 4WBB0.exe software
  // TODO
  Serial1.println(logMessage);
}



