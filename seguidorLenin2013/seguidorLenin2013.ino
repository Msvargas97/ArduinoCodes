
#define NUM_SAMPLES_PER_SENSOR  4    
#define EMITTER_PIN             11   // pin emisor del QTR

#define ENCODERPIN 10                // pin del encoder
#define LEDPIN     13                // número del pin de test
#define BUTTONPIN  2                 // boton externo

// función para pulsar el botón y esperar que deje de pulsarlo
#define esperarBoton() while(!digitalRead(BUTTONPIN)); while(digitalRead(BUTTONPIN))

// estructura para los sensores
QTRSensorsAnalog qtra((unsigned char[]) 
  {A0, A1, A2, A3, A4, A5, A6, A7}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

// arreglo para almacenamiento de valores por sensor
unsigned int sensorValues[NUM_SENSORS];

// función Velocidad Motor Izquierdo
void setMotorLeft(int value)
{
  if ( value >= 0 )
  {
    // si valor positivo vamos hacia adelante
    digitalWrite(MOTORRIGHT_DIR_A,HIGH);
    digitalWrite(MOTORRIGHT_DIR_B,LOW);
  }
  else
  {
    // si valor negativo vamos hacia atras
    digitalWrite(MOTORRIGHT_DIR_A,LOW);
    digitalWrite(MOTORRIGHT_DIR_B,HIGH);
    value *= -1;
  }

  // Setea Velocidad
  analogWrite(MOTORRIGHT_PWM,value);
}

// función Velocidad Motor Derecho
void setMotorRight(int value)
{  
  if ( value >= 0 )
  {
    // si valor positivo vamos hacia adelante
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,LOW);
  }
  else
  {
    // si valor negativo vamos hacia atras
    digitalWrite(MOTORLEFT_DIR_A,LOW);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    value *= -1;
  }    

  // Setea Velocidad
  analogWrite(MOTORLEFT_PWM,value);
}

// función Velocidad Motores
void setMotors(int left, int right)
{
  digitalWrite(STANDBY,HIGH);
  setMotorLeft(left);
  setMotorRigh(right);
}

// función Freno en Motores
void setBrake(boolean left, boolean right, int value)
{
  // pin STAND BY
  digitalWrite(STANDBY,HIGH);

  if ( left )
  {
    // pines LEFT motor
    digitalWrite(MOTORRIGHT_DIR_A,HIGH);
    digitalWrite(MOTORRIGHT_DIR_B,HIGH);
    analogWrite (MOTORRIGHT_PWM, value);
  }

  if ( right )
  {
    // pines RIGHT motor
    digitalWrite(MOTORLEFT_DIR_A,HIGH);
    digitalWrite(MOTORLEFT_DIR_B,HIGH);
    analogWrite (MOTORLEFT_PWM, value);
  }
}

void setup()
{
  // inicializar pines de salida
  pinMode(LEDPIN          ,OUTPUT);
  pinMode(STANDBY         ,OUTPUT);
  pinMode(MOTORRIGH_DIR_A ,OUTPUT);
  pinMode(MOTORRIGH_DIR_B ,OUTPUT);
  pinMode(MOTORRIGH_PWM   ,OUTPUT);
  pinMode(MOTORLEFT_DIR_A ,OUTPUT);
  pinMode(MOTORLEFT_DIR_B ,OUTPUT);
  pinMode(MOTORLEFT_PWM   ,OUTPUT);
  pinMode(BUTTONPIN       ,INPUT);

  // presiona botón para activar calibración
  while ( !digitalRead(BUTTONPIN) );

  // calibrar sensores QTRA, titilando LED como guía
  for ( int i=0; i<70; i++)
  {
    digitalWrite(LEDPIN, HIGH); delay(20);
    qtra.calibrate();
    digitalWrite(LEDPIN, LOW);  delay(20);
  }

  // apagar LED
  digitalWrite(LEDPIN, LOW);

  // presionar botón para correr el robot
  while ( !digitalRead(BUTTONPIN) );

  // esperar 5 segundos 
  delay(5000);
  
  // mover el robot suave para ganar inercia
  setMotors(90, 90);
  
  // durante 0.3 segundos
  delay(300);
}

unsigned int position = 0; // posición actual de los sensores
int derivative = 0;        // derivada
int proportional = 0;      // proporcional
int power_difference = 0;  // velocidad diferencial

// Máxima velocidad en el poder diferencial
int max = 123;

// Ultima Proporcional
int last_proportional;

// Constantes Proporcional y Derivativa
float KP = 0.17;
float KD = 2.2;

// Constante para Rango de Freno (Range Brake)
#define RANGEBRAKE 3500

void loop()
{   
  // Obtiene la posición de la linea
  // Aquí no estamos interesados ​​en los valores 
  // individuales de cada sensor
  position = qtra.readLine(sensorValues);

  // El término proporcional debe ser 0 cuando estamos en línea
  proportional = ((int)position) - 3500;

  // Si entra en el rango de freno, aplicarlo en la 
  // direccion de la curva
  if ( proportional <= -RANGEBRAKE )
  {
    setMotorRigh(0);
    setBrake(true,false,255);
    delay(1);
  }
  else if ( proportional >= RANGEBRAKE )
  {
    setMotorLeft(0);
    setBrake(false,true,255);
    delay(1);
  }

  // Calcula el término derivativo (cambio) de la posición
  derivative = proportional - last_proportional;

  // Recordando la última posición
  last_proportional = proportional;

  // Calcula la diferencia entre la potencia de los dos motores [ m1 - m2 ]. 
  // Si es un número positivo, el robot gira a la [ derecha ] 
  // Si es un número negativo, el robot gira a la [ izquierda ]
  //  y la magnitud del número determina el ángulo de giro.
  int power_difference = ( proportional * KP ) + ( derivative * KD );

  // Si velocidad diferencial es mayor a la posible tanto positiva como negativa,
  // asignar la máxima permitida
  if ( power_difference > max ) power_difference = max; 
  else if ( power_difference < -max ) power_difference = -max;

  // Asignar velocidad calculada en el poder diferencial de los motores
  ( power_difference < 0 ) ? 
    setMotors(max+power_difference, max) : setMotors(max, max-power_difference);
}
