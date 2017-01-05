#include "cordic-32bit.h"
#include <math.h> // for testing only!

void setup() {
  Serial.begin(9600);
  double p;
  int32_t s, c;
  int i;

  for (i = 0; i < 180; i++)
  {
    p = (i / 360.0) * M_PI / 2;
    //use 32 iterations
    cordic((p * MUL), &s, &c, 16);
   //fsin((p * MUL),&s,32);
  //  fcos((p * MUL),&c,32);
    //these values should be nearly equal
    Serial.print( s / MUL*150);
    Serial.print( ":");
    Serial.print(sin(p)*150);
    Serial.print("  ");
    Serial.print( c / MUL,10);
    Serial.print( ":");
    Serial.println(cos(p)- 1,10);
  }


}
void loop() {

}

