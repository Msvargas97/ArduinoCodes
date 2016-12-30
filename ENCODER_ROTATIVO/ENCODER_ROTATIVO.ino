char A=8,B=6,SW=12;
void setup() {
  // put your setup code here, to run once:
pinMode(9,OUTPUT);
pinMode(A,INPUT);
pinMode(B,INPUT);
pinMode(SW,INPUT);
Serial.begin(9600);
while(!Serial);

}

void loop() {
  // put your main code here, to run repeatedly:
Serial.print(digitalRead(A));
Serial.print('\t'+(String)digitalRead(B));
Serial.println('\t'+(String)digitalRead(SW));

}
