#ifndef __FONT_H
#define __FONT_H		

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
//unsigned char  image[];

#ifdef __AVR__
#define __PROG_TYPES_COMPAT__
#include <avr/pgmspace.h>
#else
typedef unsigned char prog_uint8_t;
#endif

extern uint8_t const font8x6[][6] PROGMEM;
extern uint8_t const font16x8[][16] PROGMEM;

					  		 
#endif  
	 
	 



