// Arduino control software
// Written by: Nick Donnelly (0978802)
// 03 October 2016

#include <Servo.h>
#include <Console.h>

// Constants
const int BAUDRATE_COMMUNICATION = 115200; // 115200 = wifi or ethernet, 9600 = USB

const int PIN_LED = 13; // Pin for the LED on-board the Yun

// Note the DC Motors have to use the PWM capable pins for speed control.
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
int SERVO_RIGHT_CLAW_POS = 0;
int SERVO_RAMP_POS = 0;
int SERVO_CAMERA_X_POS = 0;
int SERVO_CAMERA_Y_POS = 0;

void setup() {
  // Start the serial communication
  Serial1.begin(BAUDRATE_COMMUNICATION); // this should be "Serial" if using USB. Serial1 is for ethernet/wifi.
  Bridge.begin(); // for testing
  Console.begin(); // for testing
  // Register the servos to the pins on the Arduino
  servoLeftClaw.attach(PIN_SERVO_CLAWS_LEFT);
  servoRightClaw.attach(PIN_SERVO_CLAWS_RIGHT);
  servoRamp.attach(PIN_SERVO_RAMP);
  servoCameraX.attach(PIN_SERVO_CAMERA_X);
  servoCameraY.attach(PIN_SERVO_CAMERA_Y);

  // Initialize the DC motors
  pinMode(PIN_MOTOR_WHEEL_LEFT, OUTPUT); // not sure these two lines are necessary.
  pinMode(PIN_MOTOR_WHEEL_RIGHT, OUTPUT);

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
  while(Serial1.available() > 0){
    Serial1.read();
  }
  Serial1.print(F("test")); // still trying to figure out how these work 
  Console.println("test!"); // with PuTTY.
  
}


// Secondary functions
void resetAllServos(){ //TODO: these values should be updated to reflect the correct values. The default might be 90?
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
  SERVO_RIGHT_CLAW_POS = servoRightClaw.read();
  SERVO_RAMP_POS = servoRamp.read();
  SERVO_CAMERA_X_POS = servoCameraX.read();
  SERVO_CAMERA_Y_POS = servoCameraY.read();
  netLog("[SERVO] All servo position values updated.");
}


// Move a single motor at a specified speed
void startMotor(int motorSpeed, int pin){
  // motorSpeed should be between 0 and 255
  Serial1.println("[DCMOTOR] Started DC motor on %d with speed %d."); // TODO: fix this interp
  analogWrite(pin, motorSpeed);
}

// Stop moving the motor on the specified pin
void stopMotor(int pin){
  String logStr = "[DCMOTOR] Stopping motor on pin " + pin + ".";
  netLog(logStr);
  analogWrite(pin, 0); // stop the motor.
  netLog("[DCMOTOR] Motor stopped.");
}

// This function takes a motor on the provided pin and turns it on at the
// specified speed for the number of milliseconds provided
void controlSingleMotor(int motorPin, int motorSpeed, int duration){
  String logStr = "[DCMOTOR] Motor on pin " + motorPin + " with speed " + motorSpeed + " will be turned on for " + duration + " milliseconds.";
  netLog(logStr);
  analogWrite(motorPin, motorSpeed); // turn on to specificed speed
  delay(duration); // wait
  analogWrite(motorPin, 0); // turn the motor off
  netLog("[DCMOTOR] Motor stopped.");
}


// Move the ROD forward the distance specified
void moveForwardDistance(int distance){ // distance is in cm
  //TODO: this function will require real-world measurements before it can be written fully

  Serial1.println("[DCMOTOR] Moving the ROD forward.");
  analogWrite(PIN_MOTOR_WHEEL_LEFT, 127); // the number here will need to be tweaked.
  analogWrite(PIN_MOTOR_WHEEL_RIGHT, 127); 
  delay(500); // again, tweak to be calculated programatically
  analogWrite(PIN_MOTOR_WHEEL_LEFT, 0); // stop the motors
  analogWrite(PIN_MOTOR_WHEEL_RIGHT, 0);
  Serial1.println("[DCMOTOR] Motors have stopped moving.");
}

// Move the servo to the specified position
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
  // TODO: make this actually work.
  Serial1.println(logMessage); 
}
