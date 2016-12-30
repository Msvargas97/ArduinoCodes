#include <Stepper.h> //Importamos la librería para controlar motores paso a paso

#define STEPS 200 //Ponemos el número de pasos que necesita para dar una vuelta. 200 en nuestro caso

// Ponemos nombre al motor, el número de pasos y los pins de control
Stepper stepper(STEPS, 2, 4, 7, 8); //Stepper nombre motor (número de pasos por vuelta, pins de control)

int pot;  //Variable lectura potenciómetro
int derecha = 3; //Pulsador derecha
int izquierda = 2; //Pulsador izquierda
int direccion;  //Variable para indicar la direccón

void setup()
{
  pinMode(derecha, INPUT);
  pinMode(izquierda, INPUT);
}

void loop()
{
  pot = analogRead(A0); //Lectura potenciómetro
  pot = map(pot, 0, 1023, 30, 150); //Establecemos la velocidad entre 30 y 150 rpm

  stepper.setSpeed(pot); //Indicamos la velocidad al motor

  stepper.step(direccion); //Indicamos la dirección al motor

  if (digitalRead(izquierda) == HIGH)
  {
    direccion = 200; //Si pulsamos el pulsador izquierdo, el motor gira a la izquierda
  }

  if (digitalRead(derecha) == HIGH)
  {
    direccion = -200; //Si pulsamos el pulsador derecho, el motor gira a la derech
  }
}

