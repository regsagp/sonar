#pragma once

#ifdef USE_DISPLAY
//#include "TM1637/PietteTech_DHT.h"
//#include <TM1637/TM1637Display.h> 
#include <TM1637.h> 

// Module connection pins (Digital Pins)
#define CLK DISPLAY_CLK
#define DIO DISPLAY_DIO

// The amount of time (in milliseconds) between tests
#define TEST_DELAY   2000

//const uint8_t SEG_DONE[] = {
//	SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
//	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
//	SEG_C | SEG_E | SEG_G,                           // n
//	SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
//};
//
//TM1637Display display(CLK, DIO);
TM1637 tm(CLK, DIO);

#define TM_setup()  tm.begin(); tm.setBrightness(3);

#define displayNum(n) {tm.clearScreen(); tm.display(n);} 

#else
#define TM_setup() 
#define displayNum(n) 
#endif


