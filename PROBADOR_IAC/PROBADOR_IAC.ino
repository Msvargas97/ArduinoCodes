#include <Stepper.h> //Importamos la librería para controlar motores paso a paso
#include <LiquidCrystal.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define STEPS 400 //Ponemos el número de pasos que necesita para dar una vuelta
#define BTN_SET 3
#define BTN_LESS 2
#define BTN_MORE 5
#define BTN_RST 6
#define STEPS_AUTO 290
#define OUT_R  A4
#define OUT_L A3
#define NUM_SAMPLES 10

#define soft_restart()      \
  do                          \
  {                           \
    wdt_enable(WDTO_15MS);  \
    for(;;)                 \
    {                       \
    }                       \
  }while(0)
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(A2, A1, A0, 13, 12, 11);

// Ponemos nombre al motor, el número de pasos y los pins de control
Stepper stepper(STEPS, 8, 7, 10, 9); //Stepper nombre motor (número de pasos por vuelta, pins de control)
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
smoothAnalog bobinaL, bobinaR;

int direction,
    speed = 200,
    analogOutL,
    analogOutR;

char flecha_out = 0b01111110,
     flecha_in = 0b01111111,
     cuadro = 0b11011011,
     ohms = 0xF4;
boolean isRun,
        mode,
        type,
        vel;
byte customChar[8] = {
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111
};
byte customChar2[8] = {
  B00011,
  B00011,
  B00011,
  B00011,
  B00011,
  B00011,
  B00011,
  B00011
};
byte customChar3[8] = {
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000,
  B11000
};
void setup()
{
  // Velocidad del motor en RPM
  stepper.setSpeed(50);
  pinMode(BTN_SET, INPUT_PULLUP);
  pinMode(BTN_LESS, INPUT_PULLUP);
  pinMode(BTN_MORE, INPUT_PULLUP);
  pinMode(BTN_RST, INPUT_PULLUP);
  lcd.begin(16, 2);
  lcd.createChar(0, customChar);
  lcd.createChar(1, customChar2);
  lcd.createChar(2, customChar3);
  lcd.setCursor(0, 0);
  lcd.print("Servicio Tecnico");
  lcd.setCursor(0, 1);
  lcd.print(" Victor Sanchez");
  pinMode(A5, OUTPUT);
  isRun = false;
  delay(100);
  while (!isRun) {
    static long thread;
    static bool toggle;

    if (millis() - thread >= 5000) {
      lcd.setCursor(0, 0);
      if (toggle) {
        lcd.print("Servicio Tecnico");
      } else {
        lcd.print("  Probador IAC  ");
      }
      toggle = ! toggle;
      thread = millis();
    }
    if (!digitalRead(BTN_SET)) {
      isRun = true;
    }
  }
  isRun = false;
  lcd.print("Servicio Tecnico");
  lcd.setCursor(0, 1);
  lcd.print(" Victor Sanchez");
  loadingLCD(60);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(flecha_out);
  lcd.print("Valvula:Marelli");
  lcd.setCursor(0, 1);
  lcd.print(" Modo:Auto");
  static unsigned int count = 0, pos = 0;
  boolean Flag;
  while (!isRun) {
    reset();
    if (!digitalRead(BTN_SET)) {
      pos++;
      if (pos == 1) {
        lcd.setCursor(0, 0);
        lcd.print(" ");
        lcd.setCursor(0, 1);
        lcd.print(flecha_out);
        count = 0;
      } else if (pos == 2 && mode == true) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(flecha_out);
        lcd.print("Vel:Fast");
        lcd.setCursor(0, 1);
        lcd.print("         [START]");
        count = 0;
      } else isRun = true;
      delay(250);

    } else if (!digitalRead(BTN_MORE)) {
      if (pos != 0) {
        if (count < 2) {
          count++;
          Flag = true;
        }
      } else {
        if (count < 3) {
          count++;
          Flag = true;
        }
      }
      delay(250);

    } else if (!digitalRead(BTN_LESS)) {
      if (count > 0) {
        count--;
        Flag = true;
      }
      delay(250);
    }

    if (count > 0 && Flag == true) {
      if (pos == 0 ) {
        switch (count) {
          case 1:  lcd.setCursor(1, 0); lcd.print("Valvula:Marelli"); Flag = false; type = false;   stepper.changePin(2, 7);  stepper.changePin(3, 10); break;
          //(STEPS, 8, 7, 10, 9)
          // 10,7,8,9
          case 2: lcd.setCursor(1, 0); lcd.print("Valvula:Bosch   "); Flag = false; type = true;      stepper.changePin(2, 10);  stepper.changePin(3, 7); break;
          case 3:  lcd.setCursor(1, 0); lcd.print("Valvula:6-Pines"); Flag = false; type = false;     stepper.changePin(2, 7);  stepper.changePin(3, 10); break;
        }
      } else if (pos == 1) {
        switch (count) {
          case 1: lcd.setCursor(1, 1);  lcd.print("Modo:Auto  "); Flag = false; mode = false; break;
          case 2: lcd.setCursor(1, 1);  lcd.print("Modo:Manual  "); Flag = false; mode = true; break;
        }
      } else if (pos == 2) {
        switch (count) {
          case 1:  lcd.setCursor(1, 0);  lcd.print("Vel:Fast "); Flag = false; vel = false; break;
          case 2: lcd.setCursor(1, 0);  lcd.print("Vel:Low   "); Flag = false; vel = true; break;
        }
      }
    }
  }
  measureImpedance();
  delay(2500);
  if (analogOutR < 10 || analogOutL < 10) {
    lcd.clear();
    lcd.home();
    lcd.print("WARNING:");
    lcd.setCursor(0, 1);
    lcd.print("Baja impedancia!");
    while (digitalRead(BTN_SET));
  } else if ((analogOutR >= 50 && analogOutR <= 60) && (analogOutL >= 50 && analogOutL <= 60)) {
  lcd.clear();
    lcd.home();
    lcd.print("Bobinas");
    lcd.setCursor(0, 1);
    lcd.print("En buen estado...");
    delay(1500);
  }
  lcd.clear();
  lcd.home();
  if (mode) lcd.print("Modo Manual:Fast");
  else lcd.print("Modo Auto:0");
    if (!mode) stepper.step(STEPS_AUTO);
      delay(500);
    }

void loop()
{
  static int stepCount;
  reset();
  if (mode) {
    if (!digitalRead(BTN_SET)) {
      vel = !vel;
      lcd.setCursor(12, 0);
      if (vel) lcd.print("Low ");
      else lcd.print("Fast");
      if (vel) {
        stepper.setSpeed(5);
      } else {
        stepper.setSpeed(50);
      }
      delay(600);
    }
    if (!digitalRead(BTN_LESS))
    {
      lcd.setCursor(0, 1);
      lcd.print("    ");
      lcd.setCursor(13, 1);
      lcd.print(flecha_out);
      lcd.print(flecha_out);
      lcd.print(flecha_out);
      lcd.setCursor(7, 1);
      lcd.print("  ");
      if (!vel) {
        stepper.step(50);
        stepCount += 50;
      }
      else {
        stepper.step(5);
        stepCount += 5;
      }
    } else if (!digitalRead(BTN_MORE))
    {
      lcd.setCursor(13, 1);
      lcd.print("    ");
      lcd.setCursor(0, 1);
      lcd.print(flecha_in);
      lcd.print(flecha_in);
      lcd.print(flecha_in);
      lcd.setCursor(7, 1);
      lcd.print("  ");
      if (!vel) {
        stepper.step(-50);
        stepCount -= 50;
      }
      else {
        stepper.step(-5);
        stepCount -= 5;
      }
    } else {
      lcd.setCursor(13, 1);
      lcd.print("    ");
      lcd.setCursor(0, 1);
      lcd.print("     ");
      lcd.setCursor(7, 1);
      lcd.write((byte)1);
      lcd.write((byte)2);
      off();
    }
  } else {
    static unsigned int x, num;
    stepCount++;
    if (stepCount % 18 == 0) {

      if (stepCount <= STEPS_AUTO) {
        lcd.setCursor(x, 1);
        lcd.print(">");
        x++;
      } else if (stepCount > STEPS_AUTO)  {
        x--;
        if (x >= 0) {
          lcd.setCursor(x, 1);
          lcd.print("<");
        }

      }
    }

    if (!digitalRead(BTN_SET)) {
      mode = !mode;
      lcd.clear();
      lcd.home();
      lcd.print("Modo Manual:       ");
      vel = !vel;
    }
    if (stepCount <= STEPS_AUTO) {
      stepper.step(-1);
    }
    else if (stepCount > (STEPS_AUTO) && stepCount <= (STEPS_AUTO * 2)) {
      stepper.step(1);
    }
    else {
      stepCount = 0;
      x = 0;
      lcd.setCursor(0, 1);
      lcd.print("                ");
      off();
      num++;
      lcd.setCursor(10, 0);
      lcd.print(num);
      delay(500);

    }
  }
}

void reset() {
  if (!digitalRead(BTN_RST)) {
    lcd.home();
    lcd.clear();
    lcd.print("Reiniciando...");
    delay(500);
    off();
    soft_restart();
  }
}
void measureImpedance() {
  resetMillis();
  digitalWrite(A5, 1);
  lcd.clear();
  lcd.home();
  lcd.print("    Midiendo");
  lcd.setCursor(0, 1);
  lcd.print("   Impedancia....");
  delay(50);
  while (millis() < 3000) {
    analogOutL = bobinaL.readSmooth(OUT_L);
    analogOutR = bobinaR.readSmooth(OUT_R);
    delay(20);
  }
  digitalWrite(A5, 0);
  analogOutL = calculateR2(analogOutL);
  analogOutR = calculateR2(analogOutR);
  if (analogOutR > 100) analogOutR = 0;
  if (analogOutL > 100) analogOutL = 0;
  lcd.clear();
  lcd.home();
  lcd.print("L1");
  lcd.print(flecha_out);
  lcd.print(analogOutL);
  lcd.print(ohms);
  lcd.setCursor(0, 1);
  lcd.print("L2");
  lcd.print(flecha_out);
  lcd.print(analogOutR);
  lcd.print(ohms);
}
int calculateR2(int analogValue) {
  return (((unsigned long)analogValue * 100) / 880) + 1;
}
void off() {
  digitalWrite(7, 0);
  digitalWrite(8, 0);
  digitalWrite(9, 0);
  digitalWrite(10, 0);
}
void loadingLCD(unsigned int time) {
  unsigned char i;
  char a = 0xFF;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   Loading...");
  lcd.setCursor(1, 2);
  for (i = 1; i < 15; i++) {
    lcd.write((byte)0);
  }
  lcd.setCursor(0, 1);
  for (i = 1; i < 15; i++) {
    lcd.setCursor(i, 1);
    lcd.print(a);
    if (i == 14) {
      lcd.setCursor(0, 0);
      lcd.print("      100%     ");
      delay(400);
    }
    delay(time);
  }
}

