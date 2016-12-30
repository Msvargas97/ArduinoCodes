#include <SoftwareSerial.h>
/*
Sustituye xx  por simbolo menor qué o embudo menor
y xxx por simbolo Mayor que o embudo mayor
Es un problema de la web.
*/

#define RxD 4
#define TxD 2

SoftwareSerial BTSerial(RxD, TxD);

void setup()
{   
  BTSerial.flush();
  delay(500);
  BTSerial.begin(57600);
  Serial.begin(57600);
  Serial.println("Preparado para enviar comandos AT:");

  BTSerial.print("ATrn");
  delay(1000);

}

void loop()
{
  //BTSerial.print("AT");
  if (BTSerial.available())
    Serial.write(BTSerial.read());

  if (Serial.available())     
    BTSerial.write(Serial.read());   

}
