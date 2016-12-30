#include <DriverMotors.h>

/*
  Ejemplo para el uso de la libreria Driver Motors Usando dos motores y el modo 3 conexiones
  Powered by Michael Vargas © 2016

  More info about pin Modes conections:
  http://webbot.org.uk/WebbotLibDocs/42594.html

  (*)Stop mode:
  SHORT_BRAKE or OFF_HIGH_IMPEDANCE
  
  Cuando en la entrada de la funcion que asigna la velocidad al motor,
  su valor ingresado es igual a cero, activa el freno segun sea escogido ShortBrake o OFF(High-Impedance)
  Se puede cambiar el modo de frenado con la funcion StopMode():
  Ejemplo:
  motors.StopMode(SHORT_BRAKE);
  */

#define NUM_MOTORS 2 //Numero de motores a usar 
#define STOP_MODE SHORT_BRAKE //Tipo de freno aplicado cuando la velocidad sea igual a 0

//Instanciar Objeto
                        //PWM pins: PWMA,PWMB      //Control pin :INA INB...
DriverMotors motors((unsigned char []){10,9},(unsigned char []){ 13 , 5, 6, 12},STOP_MODE,NUM_MOTORS);

void setup() {
  delay(2000);
//  DDRE &= ~(1<<2);
//  PORTF |= (1<<2);
//
//  DDRD |= (1<<5);
 PORTD &= ~_BV(5);
DDRD |= _BV(5);
//DDRB |= (1<<0);
// PORTD |= 0b101111;
// PORTB |= (1<<0);

}
       int input;

void loop() {
for(int i= 0; i < 256; i++){
motors.setSpeeds(50,0);
digitalWrite(6,LOW);
digitalWrite(9,LOW);
digitalWrite(12,LOW);
}
}
 //Realiza un test al motor para observar su tiempo de respuesta en diferentes velocidades y con ambos tipos de freno
  //############## TEST SHORT_BRAKE ##########################
//
//      if(Serial.available()){
//        input = Serial.parseInt();
//        Serial.println(input);
//        if(input != 0)
//        motors.setSpeeds(30,input);//Giro sobre su propio eje
//      }
//    }
    
    /*
    //Tambien puede controlarse cada lado del motor independientemente con las funciones:
  motors.setSpeedRight(i);
  motors.setSpeedLeft(i);
  */
 
//
//  //############## TEST OFF(High-Impedance) ###################
//  for (int i = -255; i < 256; i += 5) {
//    motors.setSpeeds(i,-i);//Giro sobre su propio eje
//    delay(100);
//  }
//  motors.Stop(); //Deja girando libre el motor
//  delay(2000);//Tiempo de espera para observar el frenado
 
 
