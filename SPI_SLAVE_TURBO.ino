#include <stdlib.h>
#include <avr/interrupt.h>

#define F_CPU 8000000UL
#define NUM_SENSORS	10
#define TIMEOUT_DEFAULT	2000
#define TIME_CAL	5000
#define DDR_SPI DDRB
#define DD_MISO	PORTB4
#define LED	PORTC0
#define EMITTER_PIN	PORTC1
#define ACK 0xFE //Acuse de recibido
#define ERR	0xCA
#define ON_AND_OFF	true
#define emittersOn()	PORTC |= (1<<EMITTER_PIN)
#define emittersOff()	PORTC &= ~(1<<EMITTER_PIN)

unsigned char pins[] = {A4,A5,0,1,2,3,4,5,6,7};
//Declaración de variables
volatile uint16_t position,timeout = TIMEOUT_DEFAULT,sendSPI=0,timeCalibration = TIME_CAL;
volatile uint8_t numSensors = NUM_SENSORS;
volatile boolean enableSensors,calibrated,enableCal;
//unsigned char readMode =0;

//Declararion de punteros
volatile unsigned int  *sensorValues = (unsigned int*)malloc(sizeof(unsigned int) * numSensors),*calibratedMinimum,*calibratedMaximum;
volatile unsigned int *sensorValuesSPI = (unsigned int*)malloc(sizeof(unsigned int) * numSensors);

void setup()
{
	/*
	Output MISO, LED Y emitterPin
	*/
	DDRB |= (1<<6) | (1<<7);
	DDR_SPI |= (1<<DD_MISO);
	DDRC |= ((1<<LED) | (1<<EMITTER_PIN));
	/* Enable SPI  and Interrupt Routine*/
	SPCR = (1<<SPE) | (1<<SPIE);
	//SPCR = (1<<SPE);
	PORTC |= (1<<LED);
}
/*Rutina de interrupcion SPI */
ISR (SPI_STC_vect)
{

	byte cData = SPDR;  // grab byte from SPI Data Register
	SPCR &= ~(1<<SPIE); //Desactiva las interrupciones
	PORTB &= ~(1<<7); //Apaga el LED rojo para saber que esta entrando al a rutina
	switch(cData & 0b01111111){
		case 0x01:	 SPDR = ACK; PORTC |= (1<<LED); break; //Enciende el LED
		case 0x02:   SPDR = ACK; PORTC &= ~(1<<LED); break; //Apaga el LED
		case 0x03:	 SPDR = ACK; enableSensors = true; break; //Activa la lectura de los sensores
		case 0x04:	 SPDR = ACK; enableSensors = false; break; //Desactiva la lectura de los sesores
		case 0x05:	 SPDR = ACK; enableCal=true; break; //Habilita la calibración
		case 0x06: 	for (unsigned char i=0;i<numSensors;i++) SPI_Transmit(sensorValuesSPI[i]); break; //Envia los valores de cada sensor
		case 0x07:  SPDR = ACK; defaultCal(); break; //Asigna los valores de calibración por defecto
		case 0x08:  SPI_Transmit(position); break; //Envia el ponderado de los sensores
		case 0x40: //Write or Read el numero de sensores a usar
		if (bitRead(cData,7)) numSensors = SPI_Transmit8(0);
		else SPI_Transmit8(numSensors);
		break;
		case 0x41: //Envia los datos de calibración
		if (calibrated)
		{
			if (bitRead(cData,7)){
				for (unsigned char i=0;i<(numSensors*2);i++)
				{
					if(i < numSensors)calibratedMinimum[i] = SPI_Transmit(0);
					else calibratedMaximum[i-numSensors] = SPI_Transmit(0);
				}
				}else{
				for (unsigned char i=0;i<(numSensors*2);i++)
				{
					if(i < numSensors) SPI_Transmit(calibratedMinimum[i]);
					else SPI_Transmit(calibratedMaximum[i-numSensors]);
				}
			}
		}
		break;
		case 0x42:
		if (bitRead(cData,7)) timeCalibration = SPI_Transmit(0);
		else SPI_Transmit(timeCalibration);
		break;
		default: SPDR = ERR; break;
	}
	SPCR |= (1<<SPIE); //Activa las interrupciones
}
void loop()
{
	if(enableSensors == true && enableCal == false){
		
		PORTB |= (1<<7);
		PORTB &= ~(1<<6);
		
		if(!calibrated) readRaw();
		else position =readLine(0);

		for (unsigned char i=0;i<numSensors;i++) sensorValuesSPI[i] = sensorValues[i];
		
		}else if(enableCal == true){
		calibrated = false;
		/* Disable SPI  and Interrupt Routine*/
		//SPCR &= ~(1<<SPE) & ~(1<<SPIE);
		resetMillis();
		while(millis() < timeCalibration){
			calibration();
			PORTC ^= (1<<LED);
			_delay_ms(25);
		}
		/* Enable SPI  and Interrupt Routine*/
		//SPCR = (1<<SPE) | (1<<SPIE);
		PORTC &= ~(1<<LED);
		enableCal = false;
		calibrated  =true;
	}
}

unsigned int readLine(unsigned char white_line){
	static unsigned int lastValue = 0;
	unsigned char i, on_line = 0;
	unsigned long avg; // this is for the weighted total, which is long
	// before division
	unsigned int sum; // this is for the denominator which is <= 64000
	readCalibrated();
	avg = 0;
	sum = 0;

	for(i=0;i<numSensors;i++) {
		int value = sensorValues[i];
		if(white_line)
		value = 1000-value;
		// keep track of whether we see the line at all
		if(value > 200) {
			on_line = 1;
		}
		// only average in values that are above a noise threshold
		if(value > 50) {
			avg += (long)(value) * (i * 1000);
			sum += value;
		}
	}

	if(!on_line)
	{
		// If it last read to the left of center, return 0.
		if(lastValue < ((numSensors-1)*1000/2))
		return 0;
		// If it last read to the right of center, return the max.
		else
		return (numSensors-1)*1000;
	}

	lastValue = avg/sum;

	return lastValue;
}
void readRaw(){
	unsigned char i;
	for(i = 0; i < numSensors; i++)
	{
		sensorValues[i] = timeout;
		digitalWrite(pins[i], HIGH);   // make sensor line an output
		pinMode(pins[i], OUTPUT);      // drive sensor line high
	}
	_delay_us(40);              // charge lines for 3 us
	for(i = 0; i < numSensors; i++)
	{
		pinMode(pins[i], INPUT);       // make sensor line an input
		digitalWrite(pins[i], LOW);        // important: disable internal pull-up!
	}

	unsigned long startTime = micros();
	while (micros() - startTime < timeout)
	{
		unsigned int time = micros() - startTime;
		for (i = 0; i < numSensors; i++)
		{
			if (digitalRead(pins[i]) == LOW && time < sensorValues[i])
			sensorValues[i] = time;
		}
	}
}
void readCalibrated()
{
	int i;
	// if not calibrated, do nothing
	if(!calibratedMinimum || !calibratedMaximum) return;
	// read the needed values
	readRaw();
	for(i=0;i<numSensors;i++)
	{
		unsigned int calmin,calmax;
		unsigned int denominator;
		calmax = calibratedMaximum[i];
		calmin = calibratedMinimum[i];
		denominator = calmax - calmin;
		signed int x = 0;
		if(denominator != 0)
		x = (((signed long)sensorValues[i]) - calmin) * 1000 / denominator;
		if(x < 0)
		x = 0;
		else if(x > 1000)
		x = 1000;
		sensorValues[i] = x;
	}
}
boolean calibration(){
	int i;
	unsigned int *max_sensor_values,*min_sensor_values;
	//Reserva espacion de memoria RAM
	if(calibratedMaximum == 0){
		calibratedMaximum = (unsigned int*)malloc(sizeof(unsigned int)*numSensors);
		if(calibratedMaximum == NULL) return 0;
	}
	
	if(calibratedMinimum == 0){
		calibratedMinimum = (unsigned int*)malloc(sizeof(unsigned int)*numSensors);
		if(calibratedMinimum == NULL) return 0;
		resetCalibration();
	}
	
	max_sensor_values = (unsigned int*)malloc(sizeof(unsigned int)*numSensors);
	if(max_sensor_values == NULL) return 0;
	min_sensor_values = (unsigned int*)malloc(sizeof(unsigned int)*numSensors);
	if(min_sensor_values == NULL) return 0;
	PORTB |= (1<<6);

	int j;
	for(j=0;j<10;j++)
	{
		readRaw();
		for(i=0;i<numSensors;i++)
		{
			// set the max we found THIS time
			if(j == 0 || max_sensor_values[i] < sensorValues[i])
			max_sensor_values[i] = sensorValues[i];
			// set the min we found THIS time
			if(j == 0 || min_sensor_values[i] > sensorValues[i])
			min_sensor_values[i] = sensorValues[i];
		}
	}
	// record the min and max calibration values
	for(i=0;i< numSensors;i++)
	{
		if(min_sensor_values[i] > calibratedMaximum[i])
		calibratedMaximum[i] = min_sensor_values[i];
		if(max_sensor_values[i] < calibratedMinimum[i])
		calibratedMinimum[i] = max_sensor_values[i];
	}
	//Libera memoria
	free(min_sensor_values);
	free(max_sensor_values);
	min_sensor_values = NULL;
	max_sensor_values = NULL;
	return 1;
}
void resetCalibration()
{
	unsigned char i;
	for(i=0;i<numSensors;i++){
		calibratedMinimum[i] = timeout;
		calibratedMaximum[i] = 0;
	}
}
void defaultCal(){
	calibration();
	for(unsigned char i=0;i<numSensors;i++) calibratedMinimum[i] = 16;
	calibratedMaximum[0] = 1808;
	calibratedMaximum[1] = 1232;
	calibratedMaximum[2] = 1664;
	calibratedMaximum[3] = 1944;
	calibratedMaximum[4] = 1664;
	calibratedMaximum[5] = 1256;
	calibratedMaximum[6] = 1528;
	calibratedMaximum[7] =1816;
	calibratedMaximum[8] =1128;
	calibratedMaximum[9] =984;
}
inline static uint8_t SPI_Transmit8(uint8_t cData)
{
	//Comunicación Full-duplex con acuse de recibido (ACK)
	SPDR = cData; //Envia el MSB
	/* Wait for reception complete */
	while (!(SPSR & (1 << SPIF) ));
	asm volatile("nop");
	return SPDR;
}
inline static unsigned int SPI_Transmit(uint16_t cData)
{
	//Comunicación Full-duplex con acuse de recibido (ACK)
	static unsigned int data;
	SPDR =cData >> 8; //Envia el msb del ACK
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF) ));
	asm volatile("nop");
	data = ((unsigned char)SPDR) << 8;
	SPDR = cData & 0xFF; //Envia el lsb del ACK
	while(!(SPSR & (1<<SPIF) ));
	asm volatile("nop");
	data |= SPDR;
	/* Return Data Register */
	return data;
}
