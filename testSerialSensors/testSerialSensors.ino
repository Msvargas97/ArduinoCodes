#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 11); // RX, TX
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.println("Hello World");
  mySerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
