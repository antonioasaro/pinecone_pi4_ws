/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
  delay(500);
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object
  Serial.println("Exiting setup");
}

void loop() {
  myservo.attach(2);
  delay(500);
  for (pos = 45; pos <= 135; pos += 1) {  // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                            // waits 15 ms for the servo to reach the position
  }
  Serial.println("Servo is at position = 135");
  myservo.detach();
  delay(2000);
  
  myservo.attach(2);
  delay(500);
  for (pos = 135; pos >= 45; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                            // waits 15 ms for the servo to reach the position
  }
  Serial.println("Servo is at position = 45");
  myservo.detach();
  delay(2000);

  myservo.attach(2);
  delay(500);
  for (pos = 45; pos <= 90; pos += 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                            // waits 15 ms for the servo to reach the position
  }
  Serial.println("Servo is at position = 90");
  myservo.detach();
  delay(10000);
}
