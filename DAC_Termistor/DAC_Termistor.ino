#define F_CPU 20000000UL //Frecuencia de trabajo
#define NUM_SAMPLES 10 //Numero de muestras 
//Ajuste de voltajes de salida
#define MAX_VOLT  2.83 // voltaje de Temperatura de apagado
#define MIN_VOLT  2.16 //voltaje de Temperatura de enecendido
#define CUSTOM_VOLT 1.5 //Voltaje medio para dejarla encedida
#define VOLT_TEMP_USER 3.1 //voltaje de Temperatura deseada

#define OFFSET    1
#define BLINK_TIME 500 //Tiempo de parpadeo en milisegundos
#define TEST  false

int sensorValue = 0,
    minValue = 0,
    maxValue = 0,
    customValue = 0,
    tempUser;

boolean blinkLed = false,
        FlagDown = false; //Desactiva la temperatura maxima ya cuando fue alcanzada
uint64_t thread;
class smoothAnalog {
  public:
    // smoothAnalog();
    int readings[NUM_SAMPLES];     //Lecturas
    int readIndex = 0;   //indice de lecturas
    int total = 0;       //Suma total
    //Funcion smooth Arduino
    int readSmooth(int inputPin) {
      total = total - readings[readIndex];
      readings[readIndex] = analogRead(inputPin);
      total = total + readings[readIndex];
      readIndex = readIndex + 1;
      if (readIndex >= NUM_SAMPLES) {
        readIndex = 0;
      }
      return (total / 10);
    }
};

smoothAnalog termistor;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital
  pinMode(A2, OUTPUT);
  //Establecer DAC como salida
  DDRD = 0xFF;
  //Asigna los rangos minimos y máximos
  minValue = calculateADCvalue(MIN_VOLT);
  maxValue =  calculateADCvalue(MAX_VOLT);
  customValue = calculateADCvalue(CUSTOM_VOLT);
  tempUser =  calculateADCvalue(VOLT_TEMP_USER);
}

// the loop function runs over and over again forever
void loop() {
  //Leer el voltaje del termistor filtrado y ajustado a 8 bits para el DAC
  sensorValue =  termistor.readSmooth(A6);
#if !TEST
  if (sensorValue < (maxValue)) {
    PORTD = customValue >> 2;
    blinkLed = false;
  } else if (sensorValue >= tempUser) {
    PORTD =   minValue >> 2;
    blinkLed = true;
  }
#else
  PORTD = customValue >> 2;
  blinkLed = true;
#endif
  if (millis() - thread >= BLINK_TIME) {
    if (blinkLed)PORTC ^= (1 << 2);
    else PORTD |= ( 1 << 2);
    thread = millis();
  }
  delay(2);
}
//Retorna el valor decimal de dicho voltaje
int calculateADCvalue(double volt) {
  //          X * 1024
  //          -------
  //            5.0
  return ( volt * 1024 / 5.0);
}

