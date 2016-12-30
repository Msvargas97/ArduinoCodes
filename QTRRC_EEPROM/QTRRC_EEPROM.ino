/*
 * Codigo para almacenar  y restaurar valores de calibracion QTR en la EEPROM

   Requisitos para el circuito:
              - 2 pulsadores
              - 1 matriz sensores QTR Pololu

   Elaborado por: Michael Vargas
   Bucaramanga - Colombia
   16/10/2016
*/

#include <QTRSensors.h>
#include <EEPROM.h>


#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define ADD_START     0 //Direccion  de inicio de datos EEPROM 0 - 1023 (Arduino UNO)
#define PULSADOR_1   1 //Pin donde se conecta el pulsador para calibrar
#define PULSADOR_2   2 //Pin donde se conecta el pulsador para restaurar

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  3, 4, 5, 6, 7, 8, 9, 10
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position;

//Crear estructura para facilitar almacenar datos usando la libreria EEPROM
volatile struct MyCalibrate {
  volatile unsigned int calMax[NUM_SENSORS];
  volatile unsigned int calMin[NUM_SENSORS];
}  calibrateQTR;

//Funcion para guardar la calibración en la EEPROM
void saveCal() {
  memcpy((void*)calibrateQTR.calMin, &(*qtrrc.calibratedMinimumOn),  NUM_SENSORS * sizeof(int));
  memcpy((void*)calibrateQTR.calMax, &(*qtrrc.calibratedMaximumOn),  NUM_SENSORS * sizeof(int));
  EEPROM.put(ADD_START,  calibrateQTR );
}
//Funcion para restaurar los datos almacenados en la EEPROM
void restoreCal() {
  qtrrc.calibrate(); //Se llama la calibracion para crear espacio de memoria para la calibracion
  qtrrc.resetCalibration();
  EEPROM.get(ADD_START,  calibrateQTR); //Restaura los valores calibrados en la struct calibrateQTR
  memcpy(&(*qtrrc.calibratedMinimumOn), (void*) calibrateQTR.calMin,  NUM_SENSORS * sizeof(int)); //Copia el valor restaurado a los valores de calibracion
  memcpy(&(*qtrrc.calibratedMaximumOn), (void*) calibrateQTR.calMax,  NUM_SENSORS * sizeof(int));
}
void setup() {
  // put your setup code here, to run once:
  pinMode(PULSADOR_1, INPUT_PULLUP);
  pinMode(PULSADOR_2, INPUT_PULLUP);

  while (true) {
    if (!digitalRead(PULSADOR_1)) { //Pulsador para calibrar
      for (int i = 0; i < 250; i++) {
        qtrrc.calibrate();
      }
      saveCal();
      break; //salir del while
    } else if (!digitalRead(PULSADOR_2)) {
      restoreCal();
      break; //Salir del while
    }
  }
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values

  delay(250);
}
