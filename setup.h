#pragma once

#define NEW_PING 1
#define ULTRASONIC 2
#define SIMPLE 3

#ifdef PRESCALER
#include <prescaler.h>
#else
#define rescaleTime(A) A
#define rescaleDuration(A) A
#endif

#define LED_ON HIGH
#define LED_OFF LOW

#ifdef USE_SERIAL
#define DebugPrint(A) Serial.print(A)
#define DebugPrintln(A) Serial.println(A)
#else
#define DebugPrint(A) 
#define DebugPrintln(A) 
#endif

