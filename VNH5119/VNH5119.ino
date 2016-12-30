/*Ejemplo para el uso de la libreria Driver Motors */
#include <DriverMotors.h>
                        //PWM pins           //Control pin    INA INB...
DriverMotors motors((unsigned char []){9,10},(unsigned char []){2 ,4 ,7 ,8});

void setup() {

}

void loop() {
motors.setSpeeds(-255,255);
} 
