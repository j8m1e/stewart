#include <Servo.h>
#include <Arduino.h>     // Every sketch needs this
//Servo servo1;
//Servo servo2;
//Servo servo3;
//Servo servo4;
//Servo servo5;
//Servo servo6;
Servo servo[6];
int pin[6] = {3,5,6,9,10,11};
int angle[6];
int temp;


void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 6; i++){
    servo[i].attach(pin[i]);
    servo[i].write(90);
    
    }
//    servo1.attach(3);
//    servo1.write(90);  // set servo to mid-point
//    servo2.attach(5);
//    servo2.write(90);  // set servo to mid-point
//    servo3.attach(6);
//    servo3.write(90);  // set servo to mid-point
//    servo4.attach(9);
//    servo4.write(90);  // set servo to mid-point
//    servo5.attach(10);
//    servo5.write(90);  // set servo to mid-point
//    servo6.attach(11);
//    servo6.write(90);  // set servo to mid-point
    Serial.begin(115200);
    Serial.println("sup");

}

void loop() {
  // put your main code here, to run repeatedly:
while(Serial.available() >= 6){
    // fill array
    for (int i = 0; i < 6; i++){
      temp = Serial.read();
      servo[i].write(temp);
    }
    Serial.println("done");
}
//  for (int i = 0; i < 6; i++){
//    servo[i].write(angle[i]);
//    
//    }

}
