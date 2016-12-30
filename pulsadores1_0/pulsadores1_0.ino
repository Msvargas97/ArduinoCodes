/*
Leidy Carolina Gómez 
Estudiante Ing. Sistemas
Semillero Zion(UDI)
Cubo con 4 pulsadores. Reconoce la secuencia correcta
de oprimido 1324.
*/
int estado1;//estado del pin 1
int estado2;//estado del pin 2 
int estado3;//estado del pin 3
int estado4;//estado del pin 4
int tiempo=200;//tiempo de espera
int pin=10;//pin conectado al led
int pin1=8;//pulsador 1
int pin2=9;//pulsador 2
int pin3=11;// pulsador 3 
int pin4=12;// pulsador 4
int puntos;//puntuacion obtenida

void setup() 
{
  pinMode(pin,OUTPUT);
  //pines de entrada. conectados a los pulsadores.
  pinMode(pin1,INPUT_PULLUP);
  pinMode(pin2,INPUT_PULLUP);
  pinMode(pin3,INPUT_PULLUP);
  pinMode(pin4,INPUT_PULLUP);
  Serial.begin(115200);
}
void loop()
{
  estado1=!digitalRead(pin1);
  estado2=!digitalRead(pin2);
  estado3=!digitalRead(pin3);
  estado4=!digitalRead(pin4);
   //do-while que espera mientras preciona el boton 1 para iniciar.
   do
   {
     Serial.println("Esperando que precione el boton 1");
     delay(tiempo);
     estado1=!digitalRead(pin1);
   }while(estado1==HIGH);
   /////////////////////////////////// estado 1//////////////////////////////
   
   while(estado1==LOW)//mientras el boton 1 este precionado
   {
     //digitalWrite(pin,HIGH);//encender el led
     //Serial.println("Preciono el boton 1");
     Serial.print("cuboa1");delay(tiempo);//a GUI
     estado1=!digitalRead(pin1);
   }
   //digitalWrite(pin,LOW);//apagar el led
   //Serial.print("Puntuacion Obtenida : ");
   //Serial.println(puntos);
   //espera a que precione el boton 3(el orden correcto)
   do
   {
    Serial.print("cuboa1");delay(tiempo);
     Serial.println("Esperando que precione el boton 3");
     delay(tiempo);
     estado3=!digitalRead(pin3);
   }while(estado3==HIGH);
   /////////////////////////////////// estado 2//////////////////////////////
   
   while(estado3==LOW)//mientras preciona el boton 3
   {
      //digitalWrite(pin,HIGH);//encender el led
      //Serial.println("Preciono el boton 3");
      Serial.print("cuboa2");delay(tiempo);
      estado3=!digitalRead(pin3);
   }
    //digitalWrite(pin,LOW);//apagar el led
   //Serial.print("Puntuacion Obtenida : ");
   //Serial.println(puntos);
   //espera a que precione el boton 2
   do
   {
    Serial.print("cuboa2");delay(tiempo);
     Serial.println("Esperando que precione el boton 2");
     delay(tiempo);
     estado2=!digitalRead(pin2);
   }while(estado2==HIGH);
   /////////////////////////////////// estado 3//////////////////////////////
   
   while(estado2==LOW)//mientras preciona el boton 2
   {
      //digitalWrite(pin,HIGH);//encender el led
      //Serial.println("Preciono el boton 2");
      Serial.print("cuboa3");delay(tiempo);
      estado2=!digitalRead(pin2);
   }
    //digitalWrite(pin,LOW);//apagar el led
   //Serial.print("Puntuacion Obtenida : ");
   //Serial.println(puntos);
   //espera a que precione el 4
   do
   {
    Serial.print("cuboa3");delay(tiempo);
     Serial.println("Esperando que precione el boton 4");
     delay(tiempo);
     estado4=!digitalRead(pin4);
   }while(estado4==HIGH);
   /////////////////////////////////// estado 4//////////////////////////////
   
    while(estado4==LOW)//mientras preciona el boton 4
   {
      //digitalWrite(pin,HIGH);//encender el led
      //Serial.println("Preciono el boton 4");
      Serial.print("cuboa4");
      delay(200);
      Serial.print("cuboa4");
      delay(200);
      Serial.print("cuboa4");
      delay(200);
      estado4=!digitalRead(pin4);
   }
   //digitalWrite(pin,LOW);//apagar el led
   //Serial.print("Puntuacion Obtenida : ");
   //Serial.println(puntos);

   while(1)
   {
    Serial.print("terminino cuboa");
    delay(200);
   }
}
