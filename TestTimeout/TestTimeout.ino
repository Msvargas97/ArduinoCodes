/*
   Test tiempo de cambio de una variable
*/
int sensor;
int timeOut = 3000, min = 800, max = 1024;
boolean Flag ;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor = analogRead(A5);
  Flag = timeOutValue(sensor, timeOut, min, max);
  Serial.print(sensor);
  Serial.print('\t');
  Serial.println(Flag);
}
bool timeOutValue(int input, unsigned long timeOut, int min, int max) {
  static unsigned long lastTime;
  static unsigned int time;
  static bool lastValue, value;
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);

  if (timeChange >= 5 && lastValue == false)
  {
    if ( time < (timeOut / 5)) {
      if (input < min && input > max ) {
        time = 0;
        value = false;
      }
      time++;
    } else {
      time = 0;
      if (input >= min && input <= max) value = true;
    }
    lastTime = now;
  } else {
    lastTime = 0;
    if (input >= min && input <= max) value = true;
    else value = false;
  }
  lastValue = value;
  return value;
}

