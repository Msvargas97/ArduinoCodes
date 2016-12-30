#include <DriverMotors.h>
#include <IRremote.h>
#include <avr/interrupt.h>

#define DEBUG 0
#define TIME_DETECT_ATTACK 20
#define TIME_DETECT_SEARCH 200
#define READ_BTN() (PINE & 0b100)
#define MIN_ANALOG_DISTANCE 650
#define QTR_L A5 //Pin Sensor QTR_IZQUIERDO
#define QTR_R A0 //Pin Sensor QTR_DERECHO

//Creacion de obejtos
IRrecv irrecv(0);
decode_results results;
//PWM pins           //Control pin    INA INB...
DriverMotors motors((unsigned char []) {
  9, 10
}, (unsigned char []) {
  8 , 6 , 5 , 13
});

//######################### Espacio para Variables ######################################
unsigned long thread1, thread2; //Variables para almacenar los threads
//Variables tipo Volatiles que se van a usar dentro de interrupciones
volatile int  speed = 255; //Asigna la velocidad max
volatile unsigned int sensorAnalog, speedLeft, speedRight; //Variables para controla la velocidad de los motores
volatile bool stateSensorF, stateQTRL, stateQTRR, isRun,FlagTime3 = false,backAttack=false; //Banderas y variables que almacena el estado de los pines
volatile uint8_t sensorByte; //Almacena el valor en binario de los sensores
volatile char direction, lastDirection; //Almacena la direcion segun el estad de los sensores

//#######################################################################################
void setup() {
#if DEBUG
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Ready..");
#endif
//Inicializa perifericos y configura registros
  irrecv.enableIRIn(); // Inicia el control remoto
  motors.Stop(); //Detener motores
  PORTE |= (1 << 2); //Activa el pull-up del pulsador
  DDRE |= (1 << 2);
  DDRD |= ((1 << 5) | (1 << 4)); //LEDS
  DDRE |= (1 << 6); //Desactiva pull-up
  PORTE &= ~(1 << 6); //Transistor de sensores
  DDRB |= (1 << 7); //BUZZER
  PORTB &= ~(1 << 7); //Desactiva pull-up
  PORTD &= (~(1 << 4) & ~(1 << 5));
  DDRF = 0b1100011;//SENSORES
  PORTF  = 0b1100011; //Activar pull-up en todos los sensores
  noInterrupts();
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // prescalar = 64 = 250 Khz
  ADMUX  |= ((1 << REFS1) | (1 << REFS0));
  ADMUX |= 4; //Seleciona el canal 4 para leer el sensor del frente
  ADCSRA |= (1 << ADEN) | (1 << ADIE);
  DIDR0 |= (1 << ADC4D) ; //Deshabilita la funcion digital de los pines analogos
  //Deshabilitar JTAG para usar el puerto F. (Si no deshabilita el ADC no lee valores por debajo de 200)
  MCUCR |= (1 << JTD);
  ADCSRA |= (1 << ADHSM);
  ADCSRB = 0;
  ADCSRA |= (1 << ADSC); // start A2D conversions
  TCCR3A = TCCR3C = 0;
  TCCR3B = 0b00000010;
  TIMSK3 |= (1 << TOIE3);
  interrupts();
  TCNT3 = 63536;
  PORTD &= ~(1 << 5);
  pinMode(QTR_R, INPUT_PULLUP);
  pinMode(QTR_L, INPUT_PULLUP);

  while (true) {
    if (!READ_BTN()) break;
    if (irrecv.decode(&results)) {
      if (results.value == 0xFFC23D) {
        break;
      }
#if DEBUG
      Serial.println(results.value, HEX);
#endif
      irrecv.resume();
    }
    delay(100);
  }
  irrecv.resume();
  PORTD |= (1 << 4);
  PORTE |= (1 << 6);
  for (unsigned int i = 0; i < 5; i++) {
#if DEBUG
    Serial.println((String)(i + 1) + " second");
#endif
    PORTB |= (1 << 7);
    PORTD ^= (1<<4);
    delay(200);
    PORTB &= ~(1 << 7);
    delay(790);
  }

  #if DEBUG
  Serial.print("Init. Direction:");
  Serial.println(direction);
  #endif
    if(direction == 'S'){
  //  FlagTime3 = true;
    backAttack = true;
  }
  isRun = true;
};

void loop() {
  if (micros() - thread2 >= 1500) {
    switch (direction) {
      case 'A': speedLeft = speed; speedRight = speed; break;
      case 'R': speedLeft = speed; speedRight = -speed;
        break;
      case 'L': speedLeft = -speed; speedRight = speed;
        break;
      case 'S': 
         if(!backAttack){
         if (lastDirection == 'R') {
          speedLeft = 170; speedRight = -170;
        }
        if (lastDirection == 'L') {
          speedLeft = -170; speedRight = 170;
        }
        else {
          speedLeft = -170;
          speedRight = 170;
        }
        }else{
          speedLeft = -speed;
          speedRight = speed;
        }
        break;
    }
    motors.setSpeeds(speedLeft, speedRight);
    thread2 = micros();
  }
  if (millis() - thread1 >= 100) {
#if DEBUG
    Serial.print("Speed:"+(String)speed);
    Serial.print("\tD:" + (String) direction);
    Serial.print("\tLD:" + (String) lastDirection);
    Serial.print("\tF:" + String(stateSensorF) + "\t");
    Serial.print(sensorAnalog);
    Serial.print("\t sensorByte:");
    Serial.print(sensorByte, BIN);
    Serial.print('\t');
    Serial.print(stateQTRR);
    Serial.println(stateQTRL);
#endif
    if (irrecv.decode(&results)) {
      if (results.value == 0xFFC23D) {
        while (true) {
          motors.Stop();
          PORTD ^= (1 << 4);
          delay(300);
        }
      }
      irrecv.resume();
    }
    thread1 = millis();
  }
};
ISR(TIMER3_OVF_vect) {
  static uint8_t aux, last, cont, lastBuffer;
  static bool enable, FlagTime = false, FlagTime2 = false;
  static uint16_t time,time2,time3;
  static uint8_t timeDessac;
  TCNT3 = 63536;
  aux = ~(PINF) & 0b01000010;
  bitWrite(sensorByte, 1, stateSensorF);
  stateQTRL = PINF & 0b1;
  stateQTRR = PINF & 0b10000000;

  if ((aux == 0 && last > 0) || enable == true)
  {
    if (aux == 0) {
      enable = true;
      cont++;
    } else if (cont < 20 ) {
      aux = last;
    }
    if (cont > 20 ) {
      cont = aux = enable = last = 0;
    }
  }
  if (!enable) {
    bitWrite(sensorByte, 0, aux & 0b10);
    bitWrite(sensorByte, 2, aux & 0b01000000);
    last = aux;
  }

  if (sensorByte != lastBuffer) {
    lastDirection = direction;
  }

  switch (sensorByte) {
    case 0: if (time2 >= TIME_DETECT_SEARCH) {
        FlagTime2 = false;
        direction = 'S';
        FlagTime3 = true;
      } else {
      direction =(lastDirection != 'F') ? lastDirection : 'L';
        FlagTime2 = true;
      }
      if (FlagTime2) {
        time2++;
      } break;
    case 0b010: if (time >= TIME_DETECT_ATTACK) {
        FlagTime = false;
        direction = 'A';
        backAttack = false;
      } else {
        direction = 'F';
        FlagTime = true;
      }
      if (FlagTime) {
        time++;
      }
      break;
    case 0b100:  FlagTime3=true; direction = 'R';  break;
    case 0b001:  FlagTime3=true; direction = 'L'; break;
  }
  if(lastDirection != direction && backAttack==false ){
    speed = 255;
    FlagTime3 = false;
  }
  if(backAttack) timeDessac=20;
  else timeDessac=10;
  if(FlagTime3 && isRun){
    time3++;
    if(time3 >=timeDessac){
      time3 = 0;
    if(speed < 0 ) {
      speed = 0;
      FlagTime3=false;
    }
    else{
      if(speed > 0)
      speed-=30;
      else speed = 0;
    }
    }
  }
  
  if (direction != 'A' && direction != 'F') time = 0;
  if(direction != 'S' && lastDirection == 'S' ) time2=0;


  if (direction == 'F' && stateSensorF == false) {
    direction = lastDirection;
  }

  lastBuffer = sensorByte;
}

/*ADC Conversion Complete Interrupt Service Routine (ISR)*/
ISR(ADC_vect)
{
  static unsigned long sum;
  static uint8_t cont;
  sum += ADC;
  cont++;
  if (cont >= 255) {
    sensorAnalog = sum / cont;
    if (sensorAnalog >= MIN_ANALOG_DISTANCE) stateSensorF = true;
    else stateSensorF = false;
    if(isRun){
    if (stateSensorF) PORTD |= (1 << 4);
    else PORTD &= ~(1 << 4);
    }
    cont = sum = 0;
  }

  ADCSRA |= 1 << ADSC;  // Start Conversion
}
