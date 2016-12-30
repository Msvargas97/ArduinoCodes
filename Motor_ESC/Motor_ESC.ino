#include <Servo.h>

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
Servo myESC;

int pos = 0;
void setup() {
  pinMode(A1, INPUT_PULLUP);
  inputString.reserve(200);
  myESC.attach(11);
  myESC.write(0);
  delay(1000);
  myESC.write(0);
}

void loop() {
  static int count;
  myESC.write(80);
  delay(200);
  //  if (!digitalRead(A1)) {
  //    count++;
  //    delay(200);
  //    if (count == 1 ) {
  //      myESC.write(40);
  //      delay(10);
  //      for (unsigned char i = 40; i < 60; i++) {
  //        myESC.write(i);
  //        delay(100);
  //      }
  //    } else if (count == 2) {
  //      myESC.write(0);
  //      delay(100);
  //      count = 0;
  //    }
  //  }
  /* serialEvent();
    if (stringComplete) {
     Serial.println(inputString);
     pos=inputString.toInt();
     if(pos != 0){
     pos=map(pos,0,100,40,180);
     }
    myESC.write(pos);
    delay(2);
     inputString = "";
     stringComplete = false;
    }*/
}
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


