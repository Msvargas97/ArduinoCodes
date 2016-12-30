volatile unsigned int ticksEncoder,rev,rpm;
unsigned long lastTime;
boolean Flag,sensor;
#define ENC_PIN 2
void setup() {
  // put your setup code here, to run once:
pinMode(4,INPUT);
pinMode(6,INPUT);
Serial.begin(9600);
while(!Serial);
Serial.println("Hello World");
noInterrupts();
attachInterrupt(digitalPinToInterrupt(ENC_PIN),encoderCounter,RISING);
interrupts();
}
void loop() {
/*sensor = digitalRead(ENC_PIN);
if(sensor== true  && Flag==false){

Flag=true;
}else if(!sensor){
  Flag=false;
}
*/
if(millis() - lastTime >= 500){
  detachInterrupt(digitalPinToInterrupt(ENC_PIN));
  rpm=rev*60;
  rev=0;
  Serial.print(rpm);
  Serial.println(" RPM");
  attachInterrupt(digitalPinToInterrupt(ENC_PIN),encoderCounter, RISING);
  lastTime=millis();
}
}
void encoderCounter(){
ticksEncoder++;
if(ticksEncoder==250){
rev++;
ticksEncoder=0;
}
}


