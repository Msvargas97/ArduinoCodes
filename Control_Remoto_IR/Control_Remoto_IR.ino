// Cantidad de pulsos
#define TRAIN_LENGTH 32
// En microsegundos
#define LOW_LIMIT 600
#define HIGH_LIMIT 1800
#define INIT_LIMIT 4000
 
#define IN A0
#define LED 4
 
volatile uint32_t value;
volatile uint8_t pos = 0;
volatile boolean has_value = false;

const unsigned int key[21]={
 0x9768, //0
 0xCF30, //1
 0xE718, //2
 0x857A, //3
 0xEF10, //4
 0xC738, //5
 0xA55A, //6
 0xBD42, //7
 0xB54A, //8
 0xAD52, //9
 0x5DA2, // CH-
 0x9D62, //CH
 0x1DE2,//CH+
 0xDD22,//|<<
 0xFD02,//>>|
 0x3DC2,//>||
 0x1FE0,//-
 0x57A8,//+
 0x6F90,//EQ
 0x6798,//100+
 0x4FB0//200+
};
void inputPin() {
  static long start, delta = 0;
  if (has_value) return;
  
  noInterrupts();
  if (digitalRead(IN) == HIGH) {
      digitalWrite(LED, LOW);
    start = micros();
  }
  else {
    delta = micros() - start;
    if (delta < LOW_LIMIT) {
      digitalWrite(LED, LOW);
      value <<= 1;
      value |= 1;
      ++pos;
    }
    else if (delta < HIGH_LIMIT) {
      digitalWrite(LED, HIGH);
      value <<= 1;
      value |= 0;
      ++pos;
    }
    else if (delta > INIT_LIMIT) {
      value = 0;
      pos = 0;
    }
 
    if (pos == TRAIN_LENGTH) {
      digitalWrite(LED, LOW);
      value &= 0x0000FFFF;  //Elimina los '1' al final de cada direccion, para asi poder convertir la direccion en variables de 16 bits
      has_value = true;
    }
  }
  interrupts();
}
 
void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Hello World");
  pinMode(IN, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  attachInterrupt(digitalPinToInterrupt(0), inputPin , CHANGE);
}
 
void loop()
{
  int i;
  if (has_value) {
    Serial.print("V: ");
    Serial.println(value, HEX);
    i = 0;
    while(i<21 && (key[i] != value))++i;
    Serial.println(i);
    has_value = false;
  }
}
