#include <Servo.h>

Servo servo1;

void setup() {
  servo1.attach(6);  // S1 on Longfly dRehm
  Serial.begin(115200);
  Serial.println("Setup Good");
}

void loop() {
  servo1.write(90);
  delay(1000);
  servo1.write(40);
  delay(1000);
  servo1.write(120);
  delay(1000);
}
