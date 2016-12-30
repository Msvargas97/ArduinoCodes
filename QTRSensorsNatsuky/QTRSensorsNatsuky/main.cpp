/*
 * QTRSensorsNatsuky.cpp
 *
 * Created: 12/02/2016 12:20:42 a. m.
 * Author : Michael Vargas
 */ 
#include "QTRSensors.h"

#include <avr/io.h>

#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2
unsigned char pins[] = {0, 1, 2, 3, 4, 5};
QTRSensorsAnalog qtra(pins,
NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

