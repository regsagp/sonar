#define USE_WIFI
//#include <TM1637.h>
#include <dummy.h>
#define USE_SERIAL
//#define USE_DISPLAY

#include <ESP8266WiFi.h> // for WiFi.forceSleepBegin();


#include "pins.h"
#include "setup.h"
#include "display.h"
#include <PolledTimeout.h>

#define SONAR_LIB SIMPLE

#define SONAR_TIMEOUT 1000/2
#define VCC_TIMEOUT 10*1000

//#define CLOSED_DIST 110
//#define HAS_HUMAN(x) (x < CLOSED_DIST - 30 && x > 20)
#define NOHUMAN_DIST 120
#define HAS_HUMAN(x) (x < NOHUMAN_DIST - 20 || x > NOHUMAN_DIST + 40)
#define LED_OFF_DELAY 5000
const int MIN_VOLTAGE_MV = 3200;
const int SLEEP_VOLTAGE_MV = 3100;

float distFilt = 0;
float rawDist = 0;
bool resetDistFilt = false;
esp8266::polledTimeout::periodicMs vccTimeout(VCC_TIMEOUT);  
esp8266::polledTimeout::periodicMs sonarTimeout(SONAR_TIMEOUT);  
esp8266::polledTimeout::periodicMs idleTimeout(10000);  // don't sleep while timer not expired
uint32_t tmr_led;
uint32_t tmr_led_off;
uint32_t tmr_print;
//uint32_t tmr_volts;
int led_val = 0;
bool dir = true;
int millivolts;
bool led_on;
int prevDoorClosed;

void platform_setup() {
#ifdef PRESCALER
    setClockPrescaler(CLOCK_PRESCALER_2);
#endif
}

void setup() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LED_OFF);
    pinMode(WAKE_UP_PIN, INPUT_PULLUP);  // polled to advance tests, interrupt for Forced Light Sleep

    platform_setup();

    TM_setup();

#ifdef USE_SERIAL
    Serial.begin(rescaleTime(115200));       // для связи
    delay(50);
#endif

#ifdef USE_WIFI
    setup_wifi();
#else
    WiFi.forceSleepBegin();
#endif

    WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 3); // Automatic Light Sleep, DTIM listen interval = 3
    // at higher DTIM intervals you'll have a hard time establishing and maintaining a connection

#if SONAR_LIB == SIMPLE
    pinMode(HC_TRIG, OUTPUT); // trig выход
    pinMode(HC_ECHO, INPUT);  // echo вход
#endif

    if (LED == LED_BUILTIN) {
        digitalWrite(LED_BUILTIN, LOW);
    }
    //pinMode(LED_BUILTIN, OUTPUT);
    // using LED_BUILTIN confilcts with NewPing timer
    prevDoorClosed = -1;
}

void wakeupCallback() {  // unlike ISRs, you can do a print() from a callback function
    //testPoint_LOW;         // testPoint tracks latency from WAKE_UP_PIN LOW to testPoint LOW
    //printMillis();         // show time difference across sleep; millis is wrong as the CPU eventually stops
    DebugPrintln(F("Woke from Light Sleep - this is the callback"));
}

// when door is closed, button must be unpressed (pressing button must cause to waking up)
bool IsDoorClosed() {
    return digitalRead(WAKE_UP_PIN) != 0;
}

void light_sleep(int delay_ms) {
    delay(200); delay_ms -= 200;
    if (delay_ms < 0)
        delay_ms = 200;

    bool isIdle = idleTimeout.expired();
    if (IsDoorClosed() && isIdle) {
        DebugPrintln("Going to sleep now");

        digitalWrite(LED, LED_ON);
        delay(100);
        digitalWrite(LED, LED_OFF);  // turn the LED off so they know the CPU isn't running
        delay(100);
        digitalWrite(LED, LED_ON);
        delay(100);
    }
    else {
        DebugPrint("idle = "); DebugPrint(isIdle); DebugPrint(", Going to sleep now for "); DebugPrint(delay_ms); DebugPrintln(" ms");
    }

    Serial.flush();
    //digitalWrite(LED, HIGH);  // turn the LED off so they know the CPU isn't running
    digitalWrite(LED, LED_OFF);
    delay(20);

    if (IsDoorClosed() && isIdle) {
        wifi_station_disconnect();
        wifi_set_opmode_current(NULL_MODE);
        wifi_fpm_set_sleep_type(LIGHT_SLEEP_T); // set sleep type, the above    posters wifi_set_sleep_type() didnt seem to work for me although it did let me compile and upload with no errors 
        wifi_fpm_open(); // Enables force sleep
        gpio_pin_wakeup_enable(GPIO_ID_PIN(WAKE_UP_PIN), GPIO_PIN_INTR_LOLEVEL); // GPIO_ID_PIN(2) corresponds to GPIO2 on ESP8266-01 , GPIO_PIN_INTR_LOLEVEL for a logic low, can also do other interrupts, see gpio.h above
        wifi_fpm_do_sleep(0xFFFFFFF); // Sleep for longest possible time
        delay(200);
    }
    else {
        WiFi.forceSleepBegin();
        WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 3); // Automatic Light Sleep, DTIM listen interval = 3
        //wifi_fpm_do_sleep(delay_ms * 1000);        // Sleep range = 10000 ~ 268,435,454 uS (0xFFFFFFE, 2^28-1)
        delay(delay_ms + 1);                // delay needs to be 1 mS longer than sleep or it only goes into Modem Sleep
    }
    DebugPrintln("Wake up");
    idleTimeout.expired(); // reset if expired to prevent sleeping immediately after wakeup

    //digitalWrite(LED, LED_OFF);
}

// функция возвращает скорректированное по CRT значение
byte getBrightCRT(byte val) {
  return (0.0003066 * pow(val, 2.46));
}
// для 8 бит ШИМ
byte getBrightCRT2(byte val) {
  return ((long)val * val * val + 130305) >> 16;
}

int _click_count = 1;

/// <summary></summary>
/// <returns>ms for delay (125-low bat, 20-blink, 1000-survey), can sleep</returns>
std::tuple<int,bool> SwitchLed() {
  uint32_t current_time = millis();
  if (millivolts < MIN_VOLTAGE_MV && millivolts > 100) {
      if (millivolts < SLEEP_VOLTAGE_MV)
          ESP.deepSleep(0);

      set_led_mode(3);
      loop_led();
      return { 125, true };
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
    }
    return { 20, false };
  }
  else {
    if (current_time - tmr_led_off > LED_OFF_DELAY) {
      //analogWrite(LED, 0);
      digitalWrite(LED, LED_OFF); // turn off
    }
  }
  return { SONAR_TIMEOUT, true };
}


void loop() {
    displayNum((int)distFilt +1+_click_count);
    _click_count = -_click_count;

#ifdef USE_WIFI
    loop_wifi();
#endif

    if (prevDoorClosed < 0)
        prevDoorClosed = IsDoorClosed();
    else if (prevDoorClosed != IsDoorClosed()) {
        if (prevDoorClosed == false)
        {
            _click_count++;
            // blink one time if door is just closed
            digitalWrite(LED, LED_ON);
            delay(100);
            digitalWrite(LED, LED_OFF);  // turn the LED off so they know the CPU isn't running
        }
        prevDoorClosed = IsDoorClosed();
    }

    auto res = SwitchLed();
    int delayMs = std::get<0>(res);
    bool can_sleep = std::get<1>(res);
    resetDistFilt = can_sleep;
#ifndef USE_WIFI
    if (/*IsDoorClosed() && */can_sleep) {
        light_sleep(delayMs);
        //sonarTimeout.resetAndSetExpired();
    }
#endif

    if (sonarTimeout) {
        float dist = readDist();

#ifdef USE_SERIAL
        uint32_t current_time = tmr_print;
        Serial.print("led=");       Serial.print(current_time - tmr_led_off > LED_OFF_DELAY ? 0 : current_time - tmr_led_off);
        Serial.print(", dist=");    Serial.print(distFilt);
        Serial.print(", raw=");     Serial.print(dist);
        delay(10);
        Serial.print(", voltage: "); Serial.print(millivolts); Serial.print(", door closed: "); Serial.println(IsDoorClosed() ? 1:0); //Serial.print(", esp: "); Serial.println((int)ESP.getVcc());
#endif
    }

    if (vccTimeout) 
        readVCC();

    //delay(50); // what for? causes led blink is not smooth
}

void readVCC() {
    //tmr_volts = millis();
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
    rawDist = getDist();       // получаем расстояние
#else 
#error("Unexpected SONAR_LIB")
#endif
    if (rawDist > 2) {
        if (rawDist > 200)
            rawDist = 200;

        if (resetDistFilt)
            distFilt = rawDist;
        else
            distFilt += (rawDist - distFilt) * 0.2;  // фильтруем
    }

    return rawDist;
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
