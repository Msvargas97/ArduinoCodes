void setup() {
  // put your setup code here, to run once:
analogReference(INTERNAL);
Serial.begin(9600);
while(!Serial);
Serial.println("Hi");
DDRE |= (1<<6);
delay(100);
}

void loop() {
PORTE |= (1<<6);
  // put your main code here, to run repeatedly:
int s1,s2,s3,s4;
s1= analogRead(A3);
s2=analogRead(A2);

Serial.print(s1);
Serial.print( "   ");
Serial.println(s2);

}
