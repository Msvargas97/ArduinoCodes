void setup() {
  // put your setup code here, to run once:
pinMode(10,OUTPUT);
pinMode(11,OUTPUT);
digitalWrite(10,1);
digitalWrite(11,1);
DDRB |= (1<<6);
}

void loop() {
  // put your main code here, to run repeatedly:
for(int i= 255; i>=0; i--){
  analogWrite(10,i);
  analogWrite(11,i);
  delay(100);
}
}
