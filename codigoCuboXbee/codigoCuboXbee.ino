String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial.begin(9600);
  pinMode(9,OUTPUT);
  pinMode(13,OUTPUT);   
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  serialEvent();
  // print the string when a newline arrives:
  if (stringComplete) {
     //  Serial.println(inputString);

  if (inputString.substring(0,6) == "cuboA1") {
     digitalWrite(9,HIGH);
  }
  if (inputString.substring(0,6) == "cuboA2") {
     digitalWrite(9,LOW);
  } 

  if (inputString.substring(0,6) == "cuboB1") {
     digitalWrite(13,HIGH);
  }
  if (inputString.substring(0,6) == "cuboB2") {
     digitalWrite(13,LOW);
  } 
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
        if (inChar == '\n') {
      stringComplete = true;
    }else{
   inputString += inChar;
    }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:

  }
}

