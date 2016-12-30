#define TE A1 //Habilita la transmision de datos
#define LED A4 //LED rojo en la tarjeta del receptor
#define BUZZER A5 //enciende el buzzer en la tarjeta del receptor
#define TRIG 5
#define ECHO 6
#define DIP1 8
#define DIP2 7
#define TIEMPO_ESPERA 7000
#define DEBUG true
volatile uint32_t time,time2;
long distancia;
long tiempo;
int onBuzzer = 10;
boolean isDetected = false, Flag = false, FlagTime = false;
int maximumRangeCm = 300;
int minimumRangeCm = 10;
int count;
int RANGE_DETECT, SAMPLE_TIME = 200;
unsigned long thread1;
void setup() {
  pinMode(TE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(TRIG, OUTPUT); /*activación del pin 9 como salida: para el pulso ultrasónico*/
  pinMode(ECHO, INPUT); /*activación del pin 8 como entrada: tiempo del rebote del ultrasonido*/
  pinMode(DIP1, INPUT_PULLUP);
  pinMode(DIP2, INPUT_PULLUP);

  Serial.begin(9600);
  if (digitalRead(DIP2) && !digitalRead(DIP1)) {
    RANGE_DETECT = 200;
  } else if (!digitalRead(DIP2) && digitalRead(DIP1)) {
    RANGE_DETECT = 220;
  } else if (!digitalRead(DIP2) && !digitalRead(DIP1)) {
    RANGE_DETECT = 230;
  } else {
    RANGE_DETECT =  180;
  }
  Serial.println("Detector de objetos ultrasonico");
  Serial.println("Codigo Elaborado por Michael Vargas");
  Serial.println("############07/02/2016#############");
  Serial.print("Rango de deteccion-> ");
  Serial.print(RANGE_DETECT);
  Serial.println(" cm");
  Serial.print("Tiempo de espera-> ");
  Serial.print(TIEMPO_ESPERA);
  Serial.println(" ms");
  digitalWrite(BUZZER, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(TE, LOW);

delay(1000);
}

void loop() {
  digitalWrite(TRIG,LOW); /* Por cuestión de estabilización del sensor*/
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); /* envío del pulso ultrasónico*/
  delayMicroseconds(10);
  tiempo=pulseIn(ECHO, HIGH);
  distancia= int(0.017*tiempo); 
  
  if(distancia <= 30 || distancia >= 500){
      digitalWrite(LED,HIGH);  
  }else{
  digitalWrite(LED,0);  
  Serial.println("Distancia ");
  Serial.println(distancia);
  Serial.println(" cm");
    if(distancia <= (RANGE_DETECT-5)){
    isDetected=true;
    digitalWrite(TE,LOW);
    digitalWrite(BUZZER,HIGH);    
    delay(250);
     digitalWrite(BUZZER,LOW);
    delay(6000);
  }
  }

  delay(250);
}

