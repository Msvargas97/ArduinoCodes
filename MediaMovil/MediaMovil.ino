#define SENSORF A3
#define BAUD 115200
#define SAMPLES_FAST_AVERAGE   64
#define SAMPLES_TO_AVERAGE    256
#define NSensor_MediaMovil 1
#define Muestras_MediaMovil 5

#define PIN_ENABLE_SENSORS 7

int rawValue, sensorValueF;
//Pilas de datos para el cálculo
int  PilaSensor[NSensor_MediaMovil][Muestras_MediaMovil];
void setup() {
  Serial.begin(BAUD);

  while (!Serial);
  Serial.println("Test");
  // put your setup code here, to run once:
  pinMode(PIN_ENABLE_SENSORS, OUTPUT);
  digitalWrite(PIN_ENABLE_SENSORS, HIGH);
  analogReference(INTERNAL);

}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("Test");
  sensorValueF = MediaMovil(1, analogRead(SENSORF));
  Serial.println(sensorValueF);

}



//Devuelve un nuevo valor de la media al añadir un nuevo dato a la pila
int MediaMovil(int ntabla, int valor)
{ static int i;
  static long Ynuevo;
  Ynuevo = 0;
  //Hace hueco moviendo los datos hacia el fondo y suma los datos para la media
  for (i = Muestras_MediaMovil - 1; i > 0; i--) {
    PilaSensor[ntabla][i] = PilaSensor[ntabla][i - 1];
    Ynuevo += PilaSensor[ntabla][i];
  }
  //Empuja el nuevo dato en la pila y suma el nuevo dato
  PilaSensor[ntabla][0] = valor;
  Ynuevo += PilaSensor[ntabla][0];
  //Calcula el nuevo valor medio
  Ynuevo /= Muestras_MediaMovil; //calcula la media
  return Ynuevo;
}
