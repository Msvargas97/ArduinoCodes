#include <DriverMotors.h>
#include <IRremote.h>
#include <avr/interrupt.h>

#define DEBUG 0 //Activa el monitor serial
#define TIME_DETECT_ATTACK 20 //Tiempo en ms para que el ataca de frente
#define TIME_DETECT_SEARCH 200 //Tiempo de espera para que busque
#define READ_BTN() (PINE & 0b100)
#define MIN_ANALOG_DISTANCE 625 //Distancia del sensor del frente 
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
unsigned long thread1, thread2, thread3; //Variables para almacenar los threads
volatile uint8_t timeDessac;
//Variables tipo Volatiles que se van a usar dentro de interrupciones
volatile int  speed = 0xFF, speedMax = 0xFF; //Asigna la velocidad max
volatile unsigned int sensorAnalog, speedLeft, speedRight; //Variables para controla la velocidad de los motores
volatile bool stateSensorF, stateQTRL, stateQTRR, isRun, FlagTime3 = false, backAttack = false, toggleTurn; //Banderas y variables que almacena el estado de los pines
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
  DDRF = 0b1101111;//SENSORES
  PORTF  = 0b1101111; //Activar pull-up en todos los sensores
  noInterrupts();
/* ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // prescalar = 64 = 250 Khz
  ADMUX  |= ((1 << REFS1) | (1 << REFS0));
  ADMUX |= 4; //Seleciona el canal 4 para leer el sensor del frente
  ADCSRA |= (1 << ADEN) | (1 << ADIE);
  DIDR0 |= (1 << ADC4D) ; //Deshabilita la funcion digital de los pines analogos
  //Deshabilitar JTAG para usar el puerto F. (Si no deshabilita el ADC no lee valores por debajo de 200)
  MCUCR |= (1 << JTD);
  ADCSRA |= (1 << ADHSM);
  ADCSRB = 0;
  ADCSRA |= (1 << ADSC); // start A2D conversions*/
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
      } else if (results.value == 0xFF02FD) {
        toggleTurn = true;
        PORTD |= (1 << 5);
        delay(300);
#if DEBUG
        Serial.println("Giro a la der. asignado");
#endif
      } else if (results.value == 0xFF22DD) {
        PORTD |= (1 << 5);
        delay(300);
        toggleTurn = false;
#if DEBUG
        Serial.println("Giro a la izq. asignado");
#endif
      }
#if DEBUG
      Serial.println(results.value, HEX);
#endif
      irrecv.resume();
    }
    delay(100);
    PORTD &= ~(1 << 5);
  }
  irrecv.resume();
  PORTD |= (1 << 4);
  PORTE |= (1 << 6);
  for (unsigned int i = 0; i < 5; i++) {
#if DEBUG
    Serial.println((String)(i + 1) + " second");
#endif
    PORTB |= (1 << 7);
    PORTD ^= (1 << 4);
    delay(200);
    PORTB &= ~(1 << 7);
    delay(790);
  }

#if DEBUG
  Serial.print("Init. Direction:");
  Serial.println(direction);
#endif
  if (direction == 'S') backAttack = true;
  if (backAttack) timeDessac = 20;
  else timeDessac = 10;
  isRun = true;
  thread3 = millis();
};

void loop() {
  //Si gira hacia atras y no encuentra oponenete desactiva la bandera para el ataque hacia atras y comienza a buscar
  if (millis() - thread3 >= 400) {
    if (direction == 'S') {
      if (backAttack)
      backAttack = false;
      speedMax = 127;
      timeDessac = 60;
    }
    thread3 = millis();
  }
  if (micros() - thread2 >= 1500) {//Thread que se ejecuta cada 1.5ms, 0.5ms dsps de calcular la direcion del oponente

    switch (direction) {
      case 'A': if (timeDessac < 50 ) {
          speedLeft = speed;
          speedRight = speed;
        }
        else {
          speedLeft = 200;
          speedRight = 200;
        }
        break;
      case 'R': speedLeft = speed; speedRight = -speed;
        break;
      case 'L': speedLeft = -speed; speedRight = speed;
        break;
      case 'S':
        if (!backAttack) {
          if (lastDirection == 'R') {
            speedLeft = speed; speedRight = -speed;
          }
          if (lastDirection == 'L') {
            speedLeft = -speed; speedRight = speed;
          }
          else {
            if (!toggleTurn) {
              speedLeft = -speed;
              speedRight = speed;
            } else {
              speedRight = -speed;
              speedLeft = speed;
            }
          }
        } else {
          if (!toggleTurn) {
            speedLeft = -speed;
            speedRight = speed;
          } else {
            speedRight = -speed;
            speedLeft = speed;
          }
        }
        break;
    }
    motors.setSpeeds(speedLeft, speedRight);
    thread2 = micros();
  }
  if (millis() - thread1 >= 100) {
#if DEBUG
    Serial.print("Speed:" + (String)speed);
    Serial.print("\tTDes:" + (String)timeDessac);
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
        isRun = false;
        PORTE &= ~(1 << 6);
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
//Esta interrupcion determina la velocidad y la direcion a la cual se va a controlar el robot
ISR(TIMER3_OVF_vect) { //Interrupcion timer3 overflow
  //Creacion de variables estaticas
  static uint8_t aux, last, cont, lastBuffer;
  static bool enable, FlagTime = false, FlagTime2 = false,stateSensorL,stateSensorR;
  static uint16_t time, time2, time3, time4;

  TCNT3 = 63536; //Precarga el timer
  //aux = ~(PINF) & 0b01000010; //Obtiene el estado de los sensores en el puerto F
  aux = ~(PINF) & 0b1100; //Obtiene el estado de los sensores en el puerto F
  //bitWrite(sensorByte, 1, stateSensorF); //Asigna el bit del valor del sensor frontal
  stateQTRL = PINF & 0b1;//Obtiene el valor de los sensores de linea
  stateQTRR = PINF & 0b10000000;
  //Filtro para los sensores digitales
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
  //Si el filtro esta deshabilitado asigna los bits en e sensorByte
  if (!enable) {
    stateSensorL = aux & 0b1000;
    stateSensorR = aux & 0b100;
    bitWrite(sensorByte, 0, stateSensorL );
    bitWrite(sensorByte, 2, stateSensorR );
    last = aux;
  }

  //Si cambia de posicion el oponente Guarda la ultima direccion de ataque
  if (sensorByte != lastBuffer) {
    lastDirection = direction;
  }
  //Segun el byte que contiene el estado de los sensores
  //         0b00111
  switch (sensorByte) {
    case 0: if (time2 >= TIME_DETECT_SEARCH) {
        FlagTime2 = false;
        direction = 'S';
        FlagTime3 = true;
      } else {
        direction = (lastDirection != 'F') ? lastDirection : 'L';
        FlagTime2 = true;
      }
      if (FlagTime2) {
        time2++;
      } break;
    case 0b010: if (time >= TIME_DETECT_ATTACK) {
        FlagTime = false;
        direction = 'A';
        speed = 255;
        backAttack = false;
      } else {
        direction = 'F';
        FlagTime = true;
      }
      if (FlagTime) {
        time++;
      }
      break;
    case 0b100:

      if (backAttack) {
        time4++;
        if (time4 >= 200) {
          time4 = 0;
          backAttack = false;
          FlagTime3 = true;  direction = 'R';
        }
      } else {
        FlagTime3 = true;  direction = 'R';
      }
      break;
    case 0b001:
      if (backAttack) {
        time4++;
        if (time4 >= 200 ) {
          time4 = 0;
          FlagTime3 = true; direction = 'L';
          backAttack = false;
        }
      } else {
        FlagTime3 = true; direction = 'L';
      }
      break;
  }

  if (lastDirection != direction && backAttack == false ) {
    speed = speedMax;
    FlagTime3 = false;
  }



  if (FlagTime3 && isRun) {
    time3++;
    if (time3 >= timeDessac) {
      time3 = 0;
      if (speed < 0 ) {
        speed = 0;
        FlagTime3 = false;
      }
      else {
        if (speed > 0)
          speed -= 30;
        else speed = 0;
      }
    }
  }

  if (direction != 'A' && direction != 'F') time = 0;
  if (direction != 'S' && lastDirection == 'S' ) time2 = 0;


  if (direction == 'F' && stateSensorF == false) {
    direction = lastDirection;
  }
  if(stateSensorL) digitalWrite(4,1);
else digitalWrite(4,0);
if(stateSensorR) PORTD |=(1<<5);
else PORTD &= ~(1<<5);

  lastBuffer = sensorByte;
}

/*ADC Conversion Complete Interrupt Service Routine (ISR)*/
/*ISR(ADC_vect) //Lee el sensor analogo cada vez que este disponible el modulo ADC con una frecuencia de 250Khz
{
  static unsigned long sum;
  static uint8_t cont;
  sum += ADC;
  cont++;
  if (cont >= 255) {
    sensorAnalog = sum / cont;
    if (sensorAnalog >= MIN_ANALOG_DISTANCE) stateSensorF = true;
    else stateSensorF = false;
    if (isRun) {
      if (stateSensorF) PORTD |= (1 << 4);
      else PORTD &= ~(1 << 4);
    }
    cont = sum = 0;
  }

  ADCSRA |= 1 << ADSC;  //Inicia conversion
}*/
