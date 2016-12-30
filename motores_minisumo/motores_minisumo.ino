void setup() {
  // put your setup code here, to run once:
pinMode(4,OUTPUT);
pinMode(8,OUTPUT);
pinMode(6,OUTPUT);
pinMode(9,OUTPUT);
pinMode(13,OUTPUT);
pinMode(5,OUTPUT);
pinMode(10,OUTPUT);
delay(1000);
}

void loop() {
  digitalWrite(9,1);
  digitalWrite(8,1);
  digitalWrite(6,0);
    digitalWrite(10,1);
  digitalWrite(13,1);
  digitalWrite(5,0);
 delay(2000);
  digitalWrite(10,0);
  digitalWrite(13,0);
  digitalWrite(5,0);
  digitalWrite(9,0);
  digitalWrite(8,0);
  digitalWrite(6,0);
 delay(2000);
   digitalWrite(10,1);
  digitalWrite(13,0);
  digitalWrite(5,1);
  digitalWrite(9,1);
  digitalWrite(8,0);
  digitalWrite(6,1);
 delay(2000);
}
