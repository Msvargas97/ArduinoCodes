byte encoder=9;
unsigned long timeold;
unsigned int count,vueltas,lastVueltas;
boolean state=false,Flag=false;
unsigned int rpm;
void setup() {
  // put your setup code here, to run once:
pinMode(encoder,INPUT);
while(!Serial);
Serial.begin(9600);

}

void loop() {
if(digitalRead(encoder)==true && state==false){
count++;
state=true;
}
if(!digitalRead(encoder)){
  state=false;
}
if(count==8){
  if(!Flag){
vueltas++;
Flag=true;
  count=0;
  }
}

}
