#include <LiquidCrystal.h>
#include <Keypad.h>
#include <Servo.h>
#include <EEPROM.h>
#include <avr/interrupt.h>

Servo myESC;

#define ESC_PIN 3
#define ACS_CH A6
#define LED_RED 15
#define ENC_PIN 2
#define TOL_INIT 100e-3
#define TOL_RPM 100
#define ROWS 4 //Numero de filas para el teclado
#define COLS 4 //Numero de Columnas para el teclado
#define colA 12 //Numero de columnas para asignas el valor cuando presione la tecla A
#define colB 16 // "                                                            " B
#define colC 9 // "  " C
#define colD 10 // " " D
//Asigna los caracteres del teclado matricial
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
//Crea los pines de las filas y columnas del teclado
byte colPins[ROWS] = {2, 13, 5, 10}; //connect to the row pinouts of the keypad
byte rowPins[COLS] = {9, 8,  6, 12}; //connect to the column pinouts of the keypad
const byte pinA = 16;  // Connected to CLK on KY-040
const byte pinB = 14;  // Connected to DT on KY-040
int encoderPosCount = 0, lastValue;
int pinALast;
int aVal;
boolean bCW;
int pos = 0;
int addr = 0;

//Instancia los objetos de la LCD y el teclado
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//Crea las variables para usar la funcion millis()
unsigned long thread1, thread2, thread3;
//Crea variables booleans de configuracion
boolean toggleCursor = false, restoreValues = false, toggleVel = false;
char flecha = 0b01111110;
//Se crean cadena de caracteres para almacenar cada valor
String inputBuffer[4] = {"00000", "0", "0000", "00"};
//Variables para el amperimetro
double voltage, current, lastCurrent;
float tol;
int sensorValue;
boolean readFlag;
unsigned char samples;
//Variables para el tacometro
volatile unsigned int ticksEncoder, rev, RPM;
unsigned int lastRPM;
//Variables para el control del motor
int setPoint;
int output = 45;
float currentMax;
unsigned char tempo;
//Vectores para caracteres personalizados en la LCD
byte customChar[8] = {
  0b01000,
  0b01100,
  0b01110,
  0b01111,
  0b01110,
  0b01100,
  0b01000,
  0b00000
};
byte customChar2[8] = {
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111
};
byte customChar3[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};
struct EEPROMValues {
  int setPoint;
  bool mode;
  float current;
  unsigned char tempo;
};
void setup() {
  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  pinMode(LED_RED,OUTPUT);
  PORTD &= ~(1 << 5);
  lcd.begin(20, 4);
  lcd.createChar(0, customChar);
  lcd.createChar(1, customChar2);
  lcd.createChar(2, customChar3);
  lcd.setCursor(0, 0);
  lcd.print("  Control Vectorial");
  lcd.setCursor(0, 1);
  lcd.print("     Motor BLDC    ");
  lcd.setCursor(0, 2);
  lcd.print("      UDI 2016      ");
  lcd.setCursor(0, 3);
  lcd.write((byte)0);
  lcd.print("New[*]");
  lcd.print("      ");
  lcd.write((byte)0);
  lcd.print("Old[#]");
  //Bienvenida
  while (true) {
    static  String msg;
    char key = keypad.getKey();
    if (key == '*' || key == '#') {
      if (key == '#') restoreValues = true;
      if (restoreValues) msg = "Valores restaurados!";
      else msg = " Ingresando al menu";
      loadingLCD(msg, (restoreValues) ? 100 : 50);
      delay(1500);
      break;
    }
  }
  lcd.clear();
  lcd.home();
  if (!restoreValues) { //Si no se restauran los valores desde la EEPROM ingresa al MENU
    printMenu(); // Imprime en la LCD el menu
    for (unsigned char j = 0; j < 4; j++) {
      for (unsigned char i = 0; i < inputBuffer[j].length(); i++) inputBuffer[j][i] = 0;
    }
    while (true) {
      static unsigned char menu = 'A';
      static byte cont, contMax = 4;
      static char key;
      static unsigned int x;
      blinkCursor(); //Hace parpadear al cursor cada 500 ms
      key = keypad.getKey(); //Lee la tecla presionada
      if (key) { //Si la tecla se presionada es diferente de ' '
        if (key == 'A' || key == 'B' || key == 'C' || key == 'D') { //Si se escogio cambiar de fila
          for (unsigned i = 0; i < 4; i++) { //Borra el indicador o flecha para saber en que opcion se esta
            lcd.setCursor(0, i);
            lcd.print(" ");
          }
          menu = key; //Se le asigna el menu escodigo
          switch (menu) { //Asigna el cursor y el numero de caracteres maximos a ingresar
            case 'A': cont = 0; lcd.setCursor(0, 0); lcd.write((byte)0);  contMax = 4; lcd.setCursor(colA, 0);  break;
            case 'B': cont = 0; lcd.setCursor(0, 1); lcd.write((byte)0);  contMax = 0; lcd.setCursor(colB + ((toggleVel) ? -2 : 0), 1);  break;
            case 'C': cont = 0; lcd.setCursor(0, 2); lcd.write((byte)0);  contMax = 3; lcd.setCursor(colC, 2);  break;
            case 'D': cont = 0; lcd.setCursor(0, 3); lcd.write((byte)0);  contMax = 1; lcd.setCursor(colD, 3);  break;
          }
        } else {
          if (key == '#' || key == '*') { //Con el '#' se puede cambiar de caracter y con el '*' sale del menu
            if (key == '#') {
              if (menu != 'B' && cont > 0) cont--;
              switch (menu) {
                case 'A': lcd.setCursor(colA + cont, 0); lcd.print(" "); lcd.setCursor(colA + cont, 0);   break;
                case 'B': cont++; lcd.setCursor(8, 1); toggleVel = !toggleVel; if (toggleVel) lcd.print(" Auto[#]   "); else  lcd.print(" Manual[#]  "); inputBuffer[menu - 65][0] = toggleVel + 48; break;
                case 'C':  lcd.setCursor(colC + cont, 2); lcd.print(" mA  "); lcd.setCursor(colC + cont, 2); break;
                case 'D':  lcd.setCursor(colD + cont, 3); lcd.print(" m "); lcd.setCursor(colD + cont, 3); break;
              }
            } else {
              if ( x == 0) setPoint = inputBuffer[0].toInt();
              else setPoint = x;
              currentMax = inputBuffer[2].toFloat() / 1e+3;
              tempo = inputBuffer[3].toInt();
              delay(500); //espera medio segundo y sale del bucle
              break;
            }
            //cont++; //Cada vez que se presione una tecla aumenta el contador
          } else { //Si se presiona un numero
            if (key != '#' && key != '*' && menu != 'B' && menu != 'E') {
              lcd.print(key);
              if (menu == 'C') {
                lcd.print(" mA");
                lcd.setCursor(colC + cont + 1, 2);
              } else if (menu == 'D') {
                lcd.print(" m");
                lcd.setCursor(colD + cont + 1, 3);
              }

              inputBuffer[menu - 65][cont] = key;
              cont++;
            }
          }

          if (cont > contMax) { //Si el numero de caracteres ingresado es mayor que el permitido por la opcion del menu se regresa al inicio el cursor
            cont = 0;
            menu++;
            for (unsigned i = 0; i < 4; i++) { //Borra el indicador o flecha para saber en que opcion se esta
              lcd.setCursor(0, i);
              lcd.print(" ");
            }
            switch (menu) { //Asigna el cursor y el numero de caracteres maximos a ingresar
              case 'A': lcd.setCursor(0, 0); lcd.write((byte)0);  contMax = 4; lcd.setCursor(colA, 0);  break;
              case 'B': lcd.setCursor(0, 1); lcd.write((byte)0);  contMax = 0; lcd.setCursor(colB + ((toggleVel) ? -2 : 0), 1);  break;
              case 'C': lcd.setCursor(0, 2); lcd.write((byte)0);  contMax = 3; lcd.setCursor(colC, 2);  break;
              case 'D': lcd.setCursor(0, 3); lcd.write((byte)0);  contMax = 1; lcd.setCursor(colD, 3);  break;
              case 'E': lcd.setCursor(14, 3); lcd.write((byte)0);  contMax = 1; lcd.setCursor(18, 3);  break;
            }
          }
        }

      }
      if (menu == 'A') {
        readRotaryEncoder();
        if (encoderPosCount < 0 ) encoderPosCount = 0;
        if (encoderPosCount != lastValue) {
          x = encoderPosCount * 500;
          if (x > 20e+3) x = 20e+3;
          lcd.setCursor(colA, 0);
          lcd.print("0      ");
          lcd.setCursor(colA, 0);
          lcd.print(x);
        }
        lastValue = encoderPosCount;
      }
    }
  }
  //Guardar valores en la EEPROM
    //Data to store.

  //Imprime los valores asignados o restaurados
  lcd.noCursor();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0, 1);
  lcd.print(" VALORES ASIGNADOS  ");
  lcd.setCursor(0, 2);
  lcd.print("         OK!         ");
  delay(1000);
  lcd.clear();
  lcd.home();
  if (setPoint > 20000 || setPoint < 0) setPoint = 20000;
  if (currentMax > 5.0 || currentMax == 0) currentMax = 5.0;
  setPoint = (int) setPoint;
  //Guardar valores ingresados o restaurar los ultimos valores ingresados
  if(!restoreValues){

  EEPROMValues customVar = {
    setPoint,
    toggleVel,
    currentMax,
    tempo
  };
  //Guarda la estructura completa
  EEPROM.put(0, customVar);
  }else{ 
    digitalWrite(LED_RED,HIGH);
    //Crea una estructura del mismo tipo
  EEPROMValues customVar; //Variable to store custom object read from EEPROM.
  //Obtiene los valores guardados la ultima vez
  EEPROM.get(0, customVar);
  setPoint = customVar.setPoint;
  toggleVel = customVar.mode;
  currentMax = customVar.current;
  tempo = customVar.tempo;
  delay(100);
  }

  printFlecha("SetPoint", (String)setPoint + " RPM");
  lcd.setCursor(0, 1);
  lcd.print("Velocidad");
  lcd.print(flecha);
  lcd.print((toggleVel == true) ? "Auto" : "Manual");
  lcd.setCursor(0, 2);
  lcd.print("I Max");
  lcd.print(flecha);
  lcd.print((String)(currentMax * 1e+3) + " mA");
  lcd.setCursor(0, 3);
  lcd.print("Tiempo");
  lcd.print(flecha);
  lcd.print((String)tempo + " m    RUN[*]");
  if(restoreValues){
   digitalWrite(LED_RED,0);
   delay(500);
   digitalWrite(LED_RED,1);
   delay(500);
   digitalWrite(LED_RED,0);
  }
  while (true) if ( keypad.getKey() == '*') break;
  thread1 = thread2 = thread3 = encoderPosCount = 0;
  lcd.clear();
  lcd.home();
  lcd.noCursor();
  pinALast = digitalRead(pinA);
  lcd.home();
  printFlecha("Rev", "0RPM");
  lcd.setCursor(0, 1);
  lcd.print("Vel");
  lcd.print(flecha);
  lcd.print(map(encoderPosCount + 40, 40, 180, 0, 100));
  lcd.print(" %  ");
  lcd.setCursor(0, 2);
  lcd.print("I");
  lcd.print(flecha);
  lcd.print(current, 3);
  lcd.print("A  ");
  noInterrupts();
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // prescalar = 64 = 250 Khz
  ADMUX |= 8; //Seleciona el canal 4 para leer el sensor del frente
  ADCSRA |= (1 << ADEN);
  DIDR1 |= (1 << ADC8D) ; //Deshabilita la funcion digital de los pines analogos
  //Deshabilitar JTAG para usar el puerto F. (Si no deshabilita el ADC no lee valores por debajo de 200)
  MCUCR |= (1 << JTD);
  ADCSRA |= (1 << ADHSM);
  ADCSRB = 0;
  ADCSRA |= (1 << ADSC); // start A2D conversions
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encoderCounter, RISING);
  interrupts();
  analogReference(INTERNAL); //Asignar voltaje de refenrecia interna de 2.56V para darle precision a la lectura
  lcd.setCursor(0, 3);
  lcd.print("Time");
  lcd.print(flecha);
  //printFlecha("Out", (String)" ");
  thread1 = micros();
  arm();
  if (tempo == 0) tempo = 0xFF;
  thread2 = thread3 =  millis();
}

void loop() {
  static byte medioSecond;
  static bool isRun = true;
  static unsigned long seconds = (tempo != 0xFF ) ? tempo * 60000 : 30000;
  //Lee el teclado
  char key = keypad.getKey();
      if (key == '*') {
      isRun = !isRun;
      if (isRun && seconds==0) {
        seconds = (tempo != 0xFF ) ? tempo * 60000 : 30000;
        arm();
        output = 45;
      }
    }
  //############# Thread para leer el voltage del sensor de efecto Hall ################3
  if (micros() - thread1 >= 5) {
    if (!readFlag) {
      samples++;
      sensorValue += analogRead(ACS_CH);
      if (samples == 5 ) readFlag = true;
    }
    thread1 = micros();
  }
  //#############Thread para leer el encoder rotatorio ###############################
  if (millis() - thread2 >= 3) {
    readRotaryEncoder(); //Lectura del encoder
    if (encoderPosCount > 140) encoderPosCount = 140;
    if (encoderPosCount < 0 ) encoderPosCount = 0;

    if (lastValue != encoderPosCount && !toggleVel) {
      lcd.setCursor(4, 1);
      int x = map(encoderPosCount + 45,0, 180, 0, 100);
      x = constrain(x,0,100);
      lcd.print(x);
      lcd.print(" %  ");
      if (encoderPosCount >= 0 && isRun) {
        if (encoderPosCount == 6 && lastValue < encoderPosCount) {
          arm();
        }
        pos = encoderPosCount + 45;
        if(RPM < setPoint || (lastValue < encoderPosCount)) myESC.write(pos);
      }
    } 
    if(toggleVel){
      lcd.setCursor(4, 1);
      if (output >= 45) lcd.print(constrain(map(RPM, 0, setPoint, 0, 100),0,100));
      else lcd.print("0");
      lcd.print(" %  ");
    }

    if (readFlag) {
      voltage = (sensorValue / 5) * (2.56 / 1023.0);
      voltage = 2.50 - voltage;
      current = voltage / 185e-3; //Calcula la intesidad de corriente del ACS712 -> 185mv/A
      if (current >= currentMax && isRun){
        isRun = false;
        seconds = 0;
      }
      if (current < 0 || RPM == 0) current = 0;
      sensorValue = samples = 0;
      tol = TOL_INIT;
      if (current >= 1 ) tol += (current / 10);
      if (abs(current - lastCurrent) >= tol) {
        lcd.setCursor(2, 2);
        if (current < 1) {
          lcd.print(current * 1e+3, 0);
          lcd.print(" mA   ");
        } else {
          lcd.print(current, 2);
          lcd.print(" A  ");
        }
      } else {
        current = lastCurrent;
      }
      if (RPM == 0) {
        current = 0;
        lcd.setCursor(2, 2);
        lcd.print(current);
        lcd.print(" A     ");
      }
      lastCurrent = current;
      readFlag = false;
    }

    lastValue = encoderPosCount;
    thread2 = millis();
  }
  //Se calculan las rpms cada 500 ms
  if (millis() - thread3 >= 500) {
    if (isRun && seconds <= 5940000) seconds -= 500;

    if (seconds == 0) {
      isRun = false;
    }
    //Deshabilita la interrupcion para evitar lecturas erroneas
    detachInterrupt(digitalPinToInterrupt(ENC_PIN));
    RPM = rev * 60;
    rev = 0;
    if (abs(RPM - lastRPM) >= TOL_RPM) {
      if( RPM > setPoint || RPM >= (setPoint-300)  ) RPM = setPoint;
      lcd.setCursor(4, 0);
      lcd.print(RPM);
      lcd.print(" RPM     ");
    } else {
      RPM = lastRPM;
    }
    if (toggleVel && isRun) {
      if (RPM < (setPoint - 500)) {
        if (output <= 45)  arm();
        output++;
        if (output > 120 ) output = 120;
        myESC.write(output);
      }
    }
    if (!isRun) {
      output -= 5;
      if (output < 45) output = 0;
      myESC.write(output);
      encoderPosCount = 0;
    }
    lcd.setCursor(5, 3);
    lcd.print("        ");
    lcd.setCursor(5, 3);
    lcd.print(seconds / 1000);
    lcd.print(" s");
    if ((seconds / 1000) < 100) lcd.print(" ");
    if ((seconds / 1000) < 10) lcd.print(" ");
    
    if (isRun){
      digitalWrite(LED_RED,0);
      lcd.print("  [ENABLE]");
    }else{
      digitalWrite(LED_RED,1);
      lcd.print("    [STOP]");
    }
    lastRPM = RPM;
    attachInterrupt(digitalPinToInterrupt(ENC_PIN), encoderCounter, RISING);
    thread3 = millis();
  }
}

void printMenu() {
  lcd.clear();
  lcd.home();
  lcd.write((byte)0);
  printFlecha("A.SetPoint", " [RPM]");
  lcd.setCursor(0, 1);
  printFlecha(" B.Vel.", " Manual[#]");
  lcd.setCursor(0, 2);
  printFlecha(" C.I Max", " mA");
  lcd.setCursor(0, 3);
  printFlecha(" D.Tiempo", " m");
  lcd.print("   OK[*]");
  lcd.setCursor(colA, 0);
  delay(500);
  lcd.setCursor(colA, 0);
  lcd.print("      ");
  lcd.setCursor(colA, 0);
}

void printFlecha(String msg1, String msg2) {
  lcd.print(msg1);
  lcd.print(flecha);
  lcd.print(msg2);
}

void blinkCursor() {
  if (millis() - thread1 >= 500) {
    if (!toggleCursor) lcd.noCursor();
    else lcd.cursor();
    toggleCursor = !toggleCursor;
    thread1 = millis();
  }
}
void readRotaryEncoder() {
  aVal = digitalRead(pinA);
  if (aVal != pinALast) { // Means the knob is rotating
    // if the knob is rotating, we need to determine direction
    // We do that by reading pin B.
    if (digitalRead(pinB) != aVal) {  // Means pin A Changed first - We're Rotating Clockwise
      encoderPosCount ++;
      bCW = true;
    } else {// Otherwise B changed first and we're moving CCW
      bCW = false;
      encoderPosCount--;
    }
  }
  pinALast = aVal;
}
void loadingLCD(String msg, unsigned int time) {
  unsigned char i;
  char a = 0xFF;
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("     Loading...");
  lcd.setCursor(1, 2);
  for (i = 0; i < 18; i++) {
    lcd.write(1);
  }
  lcd.setCursor(1, 2);
  for (i = 1; i < 19; i++) {
    lcd.setCursor(i, 2);
    lcd.print(a);
    if (i == 17) {
      lcd.setCursor(8, 3);
      lcd.print("100%");
    }
    delay(time);
  }
  lcd.setCursor(0, 3);
  lcd.print("       Ready!     ");
  lcd.setCursor(0, 1);
  lcd.print(msg);
}

void arm() {
  myESC.attach(ESC_PIN);
  myESC.write(40);
  delay(10);
}

void encoderCounter() {
  ticksEncoder++;
  if (ticksEncoder == 250) {
    rev++;
    ticksEncoder = 0;
  }
}


