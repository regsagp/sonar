//#define USE_VOLT
//#define USE_WIFI
//#define USE_SERIAL

#include <ESP8266WiFi.h> // for WiFi.forceSleepBegin();

#ifdef PRESCALER
#include <prescaler.h>
#else
#define rescaleTime(A) A
#define rescaleDuration(A) A
#endif
#include "pins.h"

//#define delayMicroseconds delay
#define NEW_PING 1
#define ULTRASONIC 2
#define SIMPLE 3

#define SONAR_LIB SIMPLE

#ifdef PRESCALER
inline unsigned long trueMicros()
{
  return micros() * getClockDivisionFactor();
}

#define millis trueMillis
#define micros trueMicros
#endif

#ifdef USE_VOLT
#include <CPUVolt.h>
#endif

#if SONAR_LIB == NEW_PING
#include <NewPing.h>
// указываем пины и макс. расстояние в сантиметрах
NewPing sonar(HC_TRIG, HC_ECHO, 200);
#elif SONAR_LIB == ULTRASONIC 
#include <Ultrasonic.h>
Ultrasonic sonic(HC_TRIG, HC_ECHO);
#elif SONAR_LIB == SIMPLE 
#else 
#error("Unexpected SONAR_LIB")
#endif


#define CLOSED_DIST 110
#define HAS_HUMAN(x) (x < CLOSED_DIST - 30 && x > 20)
#define LED_OFF_DELAY 5000
const int MIN_VOLTAGE_MV = 3200;
const int SLEEP_VOLTAGE_MV = 3000;

float distFilt = 0;
uint32_t tmr_led;
uint32_t tmr_led_off;
uint32_t tmr_print;
uint32_t tmr_volts;
int led_val = 0;
bool dir = true;
signed long millivolts;
bool led_on;

void setup() {
  #ifdef PRESCALER
  setClockPrescaler(CLOCK_PRESCALER_2);
  #endif
#ifdef USE_SERIAL
  Serial.begin(rescaleTime(115200));       // для связи
#endif
  pinMode(LED, OUTPUT);

#ifdef USE_WIFI
  setup_wifi();
#else
  WiFi.forceSleepBegin();
#endif

#if SONAR_LIB == SIMPLE
  pinMode(HC_TRIG, OUTPUT); // trig выход
  pinMode(HC_ECHO, INPUT);  // echo вход
#endif

  //pinMode(LED_BUILTIN, OUTPUT);
  // using LED_BUILTIN confilcts with NewPing timer
  //digitalWrite(LED_BUILTIN, LOW);
}

// функция возвращает скорректированное по CRT значение
byte getBrightCRT(byte val) {
  return (0.0003066 * pow(val, 2.46));
}
// для 8 бит ШИМ
byte getBrightCRT2(byte val) {
  return ((long)val * val * val + 130305) >> 16;
}

void SwitchLed() {
  uint32_t current_time = millis();
  if (millivolts < MIN_VOLTAGE_MV && millivolts > 100) {
      if (millivolts < SLEEP_VOLTAGE_MV)
          ESP.deepSleep(0);

      set_led_mode(3);
      loop_led();
      return;
  }

  if (distFilt < 200)
    led_on = HAS_HUMAN(distFilt);

  if (led_on || led_val > 30) {
    tmr_led_off = current_time;
    if (current_time - tmr_led >= 20) {
      int step = 3;
      tmr_led = current_time;
      if (dir) led_val += step; // увеличиваем яркость
      else led_val -= step;   // уменьшаем
      if (led_val >= 255 || led_val <= 0) dir = !dir; // разворачиваем
      analogWrite(LED, getBrightCRT(led_val) / 4);
      //analogWrite(LED_BUILTIN, getBrightCRT(val) / 4);
    }
  }
  else {
    if (current_time - tmr_led_off > LED_OFF_DELAY) {
      analogWrite(LED, 0);
      //analogWrite(LED_BUILTIN, 0);
    }
  }
}

void loop() {
#ifdef USE_WIFI
    loop_wifi();
#endif
    
    SwitchLed();

  if (millis() - tmr_print >= 400) {
    float dist = readDist();
    tmr_print = millis();
    
#ifdef USE_SERIAL
    uint32_t current_time = tmr_print;
    Serial.print("led=");       Serial.print(current_time - tmr_led_off > LED_OFF_DELAY ? 0 : current_time - tmr_led_off);
    Serial.print(", dist=");    Serial.print(distFilt);
    Serial.print(", ");         Serial.print(dist);
    Serial.print(", voltage: "); Serial.print(millivolts); Serial.print(", esp: "); Serial.println(ESP.getVcc());
#endif
  }

  if (millis() - tmr_volts >= 1 * 1000) {
    tmr_volts = millis();
    long vcc = 0;
#ifdef USE_VOLT
    vcc = readVcc();
  #endif

    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float v_ref = 3.300;
    const float r1 = 10000;
    const float r2 = 7800;
    float delitelK = r2 / (r1 + r2);
    int voltage = 1000 * sensorValue * (v_ref / (delitelK * 1023.0));
    if (voltage > 500 && (vcc == 0 || voltage < vcc))
        millivolts = voltage - 100;
    else
        millivolts = vcc;

    //Serial.print(", voltage: "); Serial.println(sensorValue);
  }
}

float newPingDist;


float readDist() {
#if SONAR_LIB == NEW_PING
#ifndef TIMER_ENABLED
#error("");
#else
#pragma message("TIMER_ENABLED")
#endif
    //Serial.println((int)SONAR_LIB);
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
    float dist;
    if (newPingDist >= 0)
        dist = newPingDist;
    //else
    //    return -1;
#elif SONAR_LIB == ULTRASONIC 
    float dist = Ultrasonic_read(CM);
    //float dist = sonic.read(CM);
#elif SONAR_LIB == SIMPLE 
    float dist = getDist();       // получаем расстояние
#else 
#error("Unexpected SONAR_LIB")
#endif

  if (dist > 200)
    dist = 0;

  distFilt += (dist - distFilt) * 0.2;  // фильтруем
  return dist;
}

unsigned long previousMicros;
unsigned int Ultrasonic_timing() {
  //  if (threePins)
  //    pinMode(trig, OUTPUT);

  digitalWrite(HC_TRIG, LOW);
  delayMicroseconds(rescaleDuration(2));
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(rescaleDuration(10));
  digitalWrite(HC_TRIG, LOW);
  //delayMicroseconds(rescaleDuration(5));
  //digitalWrite(HC_TRIG, HIGH);
  //delayMicroseconds(rescaleDuration(5));
  //digitalWrite(HC_TRIG, LOW);

  //  if (threePins)
  //    pinMode(HC_TRIG, INPUT);

  unsigned long timeout = rescaleDuration(20000UL);
  previousMicros = micros();
  while (!digitalRead(HC_ECHO) && (micros() - previousMicros) <= timeout); // wait for the echo pin HIGH or timeout
  previousMicros = micros();
  while (digitalRead(HC_ECHO)  && (micros() - previousMicros) <= timeout); // wait for the echo pin LOW or timeout

  return micros() - previousMicros; // duration
}
unsigned int Ultrasonic_read(uint8_t und) {
  return Ultrasonic_timing() / und / 2;  //distance by divisor
}

#if SONAR_LIB == NEW_PING

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
    if (sonar.check_timer()) { // This is how you check to see if the ping was received.
        newPingDist = rescaleTime(sonar.ping_result / US_ROUNDTRIP_CM);
        // Here's where you can add code.
        //Serial.print("Ping: ");
        //Serial.print(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
        //Serial.println("cm");
    }
}
#elif SONAR_LIB == SIMPLE 

// сделаем функцию для удобства
float getDist() {
  // импульс 10 мкс
  digitalWrite(HC_TRIG, LOW);
  delayMicroseconds(rescaleDuration(2));
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(rescaleDuration(10));
  digitalWrite(HC_TRIG, LOW);

  // измеряем время ответного импульса
  auto us = pulseIn(HC_ECHO, HIGH);

  //Serial.print(", us: "); Serial.println(us);
  // считаем расстояние и возвращаем
  return (us / 58.3);
}
#endif
