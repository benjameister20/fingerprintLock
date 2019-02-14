#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Adafruit_Fingerprint.h>

// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
// comment these two lines if using hardware serial
SoftwareSerial mySerial(2, 3);
#include <SoftwareSerial.h>
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// creates a pwm object with default I2C and address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

void setup() {
  // begin serial monitor
  Serial.begin(9600);
  delay(100);
  
  // initialize the servo shield
  Serial.println("Initializing servo motors...");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
  pwm.setPWM(1, 0, servoPos[0]);
  pwm.setPWM(2, 0, servoPos[1]);
  pwm.setPWM(3, 0, servoPos[2]);
  pwm.setPWM(4, 0, servoPos[3]);
  pwm.setPWM(5, 0, servoPos[4]);
  pwm.setPWM(6, 0, servoPos[5]);
  Serial.println("Sucessfully initialized motors.");

  // initialize fingerprint sensor
  Serial.println("Initializing fingerprint sensor...");
  finger.begin(57600);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }
  finger.getTemplateCount();
  Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  Serial.println("Sucessfully initialized fingerprint sensor.");

  Serial.println("Starting lock program...");
}

void moveServo(int xy, int joystick, int dir) {

  int servo = servos[xy+joystick];        // determine which servo to move
  int pos = servoPos[xy+joystick];        // find current position
  int newPos = pos+dir*2;                 // calculate new position
  //if (servo == 1)
    //newPos += 50*dir;
  Serial.print("Servo: ");
  Serial.print(servo);
  Serial.print(" and newPos: ");
  Serial.println(newPos);
  
  if (newPos < SERVOMAX-10 && newPos > SERVOMIN + 10) {     // if possible then move
    Serial.println("In if Statement");
    Serial.print("Servo: "); Serial.print(servo);Serial.print(" newPos: ");Serial.print(newPos);Serial.print(" xy+joystick: ");Serial.println(xy+joystick);
    pwm.setPWM(servo, 0, newPos);
    servoPos[xy+joystick] = newPos;           // set current position as the new position
    delay(10);
  }
  
}

void loop() {

  // x movement
  if (analogRead(RJVRx) > 550) {
    moveServo(0, 2+RJState, 1);
    Serial.print("RJVRx: ");
    Serial.println(analogRead(RJVRy));
  }
  if (analogRead(RJVRx) < 480) {
    moveServo(0, 2+RJState, -1);
    Serial.print("RJVRx: ");
    Serial.println(analogRead(RJVRx));
  }
  if (analogRead(LJVRx) > 550) {
    moveServo(0, 1, 1);
    Serial.print("LJVRx: ");
    Serial.println(analogRead(LJVRx));
  }
  if (analogRead(LJVRx) < 480) {
    moveServo(0, 1, -1);
    Serial.print("LJVRx: ");
    Serial.println(analogRead(LJVRx));
  }

  // y movement
  if (analogRead(RJVRy) > 550) {
    moveServo(1, 2+RJState, 1);
    Serial.print("RJVRy: ");
    Serial.println(analogRead(RJVRy));
  }
  if (analogRead(RJVRy) < 480) {
    moveServo(1, 2+RJState, -1);
    Serial.print("RJVRy: ");
    Serial.println(analogRead(RJVRy));
  }
  if (analogRead(LJVRy) > 670) {
    moveServo(-1, 1, 1);
    Serial.print("LJVRy: ");
    Serial.println(analogRead(LJVRy));
  }
  if (analogRead(LJVRy) < 600) {
    moveServo(-1, 1, -1);
    Serial.print("LJVRy: ");
    Serial.println(analogRead(LJVRy));
  }

  // switch which motors are used if button is pressed
  if (digitalRead(RJSWButton) == LOW && (millis() - timer > 500) ) {
    Serial.println("button changing state");
    if (RJState == 0)
      RJState = 2;
    else
      RJState = 0;
    delay(20);
    timer = millis();
  }
  
  delay(10);
}
