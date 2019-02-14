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

bool locked;

void setup() {
  // begin serial monitor
  Serial.begin(9600);
  delay(100);
  
  // initialize the servo shield
  Serial.println("Initializing servo motors...");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
  pwm.setPWM(1, 0, 100);
  pwm.setPWM(2, 0, 600);
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

  locked = false;
  
  Serial.println("Starting lock program...");
  delay(50);
}

void loop() {

  int id = getFingerprintIDez();
  if (id != -1) {
    Serial.println("Valid fingerprint detected.");
    if (locked)
      Serial.println("The door is now unlocked.");
    else
      Serial.println("The door is now locked.");
    delay(10);
    
    changeState();
  }
   
  delay(50);
  
}

void changeState() {
  if (locked) {
    pwm.setPWM(1, 0, 100);
    pwm.setPWM(2, 0, 600);
  }
  else {
    pwm.setPWM(1, 0, 600);
    pwm.setPWM(2, 0, 100);
  }

  locked = !locked;
}

// function taken from Adafruit fingerprint tutorial
// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;
  
  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID); 
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID; 
}
