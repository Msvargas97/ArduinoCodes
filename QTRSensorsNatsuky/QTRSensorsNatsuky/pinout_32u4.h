#ifndef pinout_32u4_h__
#define pinout_32u4_h__

#if defined(__AVR_ATmega32U4__) || defined (__AVR_ATmega16u4__)
#include <avr/io.h>

#define INPUT 				0
#define OUTPUT				1
#define LOW					0
#define HIGH				1
#define TOGGLE				0xFF
#define HIGH_IMPEDANCE		0
#define PULL_UP_ENABLED		1


//Puerto B
#define IO_PB0			0
#define IO_PB1			1
#define IO_PB2			2
#define IO_PB3			3 
#define IO_PB4			4
#define IO_PB5			5		
#define IO_PB6			6
#define IO_PB7			7
//Puerto D
#define IO_PD0			8
#define IO_PD1			9
#define IO_PD2			10
#define IO_PD3			11
#define IO_PD4			12
#define IO_PD5			13
#define IO_PD6			14
#define IO_PD7			15
//Puerto F
#define IO_PF0			16
#define IO_PF1			17
#define IO_PF4			18
#define IO_PF5			19
#define IO_PF6			20
#define IO_PF7			21
//Puerto C
#define IO_PC6			22
#define IO_PC7			23
//Puerto E
#define IO_PE6			24

//Estructura y clase creada gracias a libreria de Pololu
struct IOStruct
{
	// if these aren't volatile, the compiler sometimes incorrectly optimizes away operations involving these registers:
	volatile unsigned char* pinRegister;
	volatile unsigned char* portRegister;
	volatile unsigned char* ddrRegister;
	unsigned char bitmask;
};


#ifdef __cplusplus

class Analog {
	public:
	Analog();
	inline static void init(){
			ADCSRA = (1<<ADPS2)|(1<<ADPS1); // 12Mhz/64 = 187.5 KHz
			ADMUX = (1<<REFS0); //Volataje de referencia 3.3v
			ADCSRA = (1<<ADEN); //Habilitar ADC
			ADCSRA |= (1<<ADSC); //Hacer conversion inicial
	}
	inline static uint16_t read(unsigned char ch)
	{
 if (ch >=8)//
 {
	 ch -= 0x08;//ch - 8
	 ADCSRB |=  (1 << MUX5);   // ADC8-ADC13
 }
 else
 {
	 ADCSRB &=  ~(1 << MUX5);   // clear MUX 5 bit part of ADCSRB to read upper bit
 }
 ADMUX &= 0b01000000; //Borra el canal anterior
 ch &= 0x07; // check for sanity data - 0-7 allowed
 ADMUX |= ch; // selecting channel
 ADCSRA|=((1<<ADSC) | (1<<ADEN));   // start conversion
 while(!(ADCSRA & (1<<ADIF))); // waiting for ADIF, conversion complete
 ADCSRA|=(1<<ADIF);   // clearing of ADIF, it is done by writing 1 to it
 ADCSRA &= ~(1<<ADEN);
 return (ADC);
	}
	inline static unsigned char getInputValue(struct IOStruct* ioPin)
	{
		return *(ioPin->pinRegister) & ioPin->bitmask;
	}
};
class Digital
{
	public:

	// constructor (doesn't do anything)
	Digital();
	// gets a structure with pointers to the three digital I/O registers associated
	// with the specified pin (DDR, PORT, and PIN) along with a bitmask with a
	// 1 in the position of the specified pin and 0s everywhere else.
	inline static void getIORegisters(struct IOStruct* io, unsigned char pin)
	{
		io->pinRegister = 0;
		io->portRegister = 0;
		io->ddrRegister = 0;
		io->bitmask = 0;
		
		if (pin < 8)			// pin 0 = PB0, ..., 7 = PB7
		{
			io->pinRegister = (unsigned char*)&PINB;
			io->portRegister = (unsigned char*)&PORTB;
			io->ddrRegister = (unsigned char*)&DDRB;
			io->bitmask = 1 << pin;
		}
		else if (pin < 16)		
		{
			io->pinRegister = (unsigned char*)&PIND;
			io->portRegister = (unsigned char*)&PORTD;
			io->ddrRegister = (unsigned char*)&DDRD;
			io->bitmask = 1 << (pin - 8);
		}
		else if (pin < 22)		// Puerto F
		{   
			io->pinRegister = (unsigned char*)&PINF;
			io->portRegister = (unsigned char*)&PORTF;
			io->ddrRegister = (unsigned char*)&DDRF;
			io->bitmask = 1 << ((pin - 16) > 1 ? (pin-14) : (pin - 16));
		}else if (pin < 24){
			io->pinRegister = (unsigned char*)&PINC;
			io->portRegister = (unsigned char*)&PORTC;
			io->ddrRegister = (unsigned char*)&DDRC;
			io->bitmask = 1 << (pin-16);
		}else if (pin==24){
			io->pinRegister = (unsigned char*)&PINE;
			io->portRegister = (unsigned char*)&PORTE;
			io->ddrRegister = (unsigned char*)&DDRE;
			io->bitmask = 1 << 6;
		}
		
	}


	// low-level method for setting the data direction (i.e. input or output) of an pin or set of pins
	// described by an IOStruct pointer.
	inline static void setDataDirection(struct IOStruct* ioPin, unsigned char val)
	{
		if (val)
		*(ioPin->ddrRegister) |= ioPin->bitmask;
		else
		*(ioPin->ddrRegister) &= ~ioPin->bitmask;
	}


	// low-level method for setting the PORT register value of an pin or set of pins
	// described by an IOStruct pointer.  If the pin is an input, this lets you choose between
	// setting it as high-impedance (val = 0) or enabling the internal pull-up (val = 1).  If the pin is an
	// output, this lets you choose between driving low (val = 0) and driving high (val = 1).
	// NOTE: if val is 0xFF (255), this method will toggle the PORT register pin(s).
	inline static void setOutputValue(struct IOStruct* ioPin, unsigned char val)
	{
		if (val == 0xFF)
		*(ioPin->portRegister) ^= ioPin->bitmask;
		else if (val)
		*(ioPin->portRegister) |= ioPin->bitmask;
		else
		*(ioPin->portRegister) &= ~ioPin->bitmask;
	}


	// low-level method for reading the value of the PIN register for an pin or set of pins
	// described by an IOStruct pointer.
	inline static unsigned char getInputValue(struct IOStruct* ioPin)
	{
		return *(ioPin->pinRegister) & ioPin->bitmask;
	}


	// high-level method for setting the specified pin as an output with the specified output state.
	// An outputState value of 0 will cause the pin to drive low; a value of 1 will cause the pin to
	// drive high.  A value of 0xFF (255) will toggle the output state of the pin (i.e. high -> low and
	// low -> high).
	inline static void setOutput(unsigned char pin, unsigned char outputState)
	{
		struct IOStruct registers;
		getIORegisters(&registers, pin);
		setOutputValue(&registers, outputState);
		setDataDirection(&registers, 1);
	}


	// high-level method for setting the specified pin as an input with the specified input state.
	// An inputState value of 0 will cause the pin to be a high-impedance input; a value of 1 will enable the
	// pin's internal pull-up resistor, which weakly pulls it to Vcc.  A value of 0xFF (255) will toggle the
	// input state.
	inline static void setInput(unsigned char pin, unsigned char inputState)
	{
		struct IOStruct registers;
		getIORegisters(&registers, pin);
		setDataDirection(&registers, 0);
		setOutputValue(&registers, inputState);
	}


	// high-level method for reading the input value of the specified pin.  If the voltage on the pin is low,
	// this method will return 0.  Otherwise, it will return a non-zero result that depends on the value of
	// the pin.
	inline static unsigned char isInputHigh(unsigned char pin)
	{
		struct IOStruct registers;
		getIORegisters(&registers, pin);
		return getInputValue(&registers);
	}

};

#endif
#endif
#endif // pinout_32u4_h__
