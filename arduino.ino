//#define USE_VOLT



#ifdef PRESCALER
//#define delayMicroseconds delay
inline unsigned long trueMicros()
{
	return micros() * getClockDivisionFactor();
}

#define millis trueMillis
#define micros trueMicros
#endif

#if SONAR_LIB == SIMPLE
#elif SONAR_LIB == NEW_PING
#include <NewPing.h>
// указываем пины и макс. расстояние в сантиметрах
NewPing sonar(HC_TRIG, HC_ECHO, 200);
float newPingDist;
#elif SONAR_LIB == ULTRASONIC 
#include <Ultrasonic.h>
Ultrasonic sonic(HC_TRIG, HC_ECHO);
#else 
#error("Unexpected SONAR_LIB")
#endif

#if 0

// this case is capricious. E.g., doesn't going to sleep if delay added
void light_sleep2() {
    Serial.println(F("\n6th test - Forced Light Sleep, wake with GPIO interrupt"));
    Serial.flush();
    WiFi.mode(WIFI_OFF);      // you must turn the modem off; using disconnect won't work

    //if (LED != LED_BUILTIN) {
    //    Serial.println(F("LED != LED_BUILTIN"));
    //    digitalWrite(LED, LOW);
    //    delay(100);
    //    digitalWrite(LED, HIGH);  // turn the LED off so they know the CPU isn't running
    //    delay(100);
    //    digitalWrite(LED, LOW);
    //    delay(100);
    //}
    digitalWrite(LED, HIGH);  // turn the LED off so they know the CPU isn't running
    int s = digitalRead(WAKE_UP_PIN);
    if (s == 0) {
        Serial.println(F("button is pressed, prevent to sleep"));
        //digitalWrite(LED, !digitalRead(LED));
        delay(500);
        return;
    }

    //digitalWrite(LED, LOW);


    //readVoltage();            // read internal VCC
    Serial.println(F("CPU going to sleep, pull WAKE_UP_PIN low to wake it (press the switch)"));
    //printMillis();   // show millis() across sleep, including Serial.flush()
    //testPoint_HIGH;  // testPoint tracks latency from WAKE_UP_PIN LOW to testPoint LOW in callback
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    gpio_pin_wakeup_enable(GPIO_ID_PIN(WAKE_UP_PIN), GPIO_PIN_INTR_LOLEVEL);
    // only LOLEVEL or HILEVEL interrupts work, no edge, that's an SDK or CPU limitation
    wifi_fpm_set_wakeup_cb(wakeupCallback);  // Set wakeup callback (optional)
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xFFFFFFF);   // only 0xFFFFFFF, any other value and it won't disconnect the RTC timer
    delay(1000);                      // it goes to sleep during this delay() and waits for an interrupt
    Serial.println(F("Woke up!"));  // the interrupt callback hits before this is executed*/

    s = digitalRead(WAKE_UP_PIN);
    Serial.print("WAKE_UP_PIN = "); Serial.println(s);

}

#endif