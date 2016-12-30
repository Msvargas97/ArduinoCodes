#include <Servo.h>

#define OPEN 0
#define CLOSE 90
#define pinCodo 9
#define pinPinza 8
#define pinBrazo 6
#define pinBase 12
Servo servoCodo,servoPinza,servoBrazo,servoBase;  // create servo object to control a servo

// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
servoCodo.attach(pinCodo);
servoPinza.attach(pinPinza);
servoBrazo.attach(pinBrazo);
servoBase.attach(pinBase);
servoPinza.write(CLOSE);
for(int i=0;i<=90;i++){
servoBrazo.write(i+5);
servoCodo.write(i);
servoPinza.write(OPEN);
servoBase.write(i);
delay(10);
}
delay(1500);
servoCodo.write(0);
servoBrazo.write(17);
for(int i=90;i>=15;i--){
servoBrazo.write(i+5);
servoCodo.write(i-20);
servoPinza.write(OPEN);
delay(10);
}
delay(1500);
servoPinza.write(CLOSE);
delay(1000);
for(int i=0;i<=45;i++){
servoBrazo.write(i+10);
servoCodo.write(i);
delay(10);
}
}

void loop() {

}
