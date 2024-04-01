#define LED_MODE_OFF 0
#define LED_MODE_ON 1
#define LED_MODE_1_2 2
#define LED_MODE_1_8 3
#define LED_MODE_2_8 4

// Массив режимов работы светодиода
byte modes[] = {
   0B00000000, //Светодиод выключен
   0B11111111, //Горит постоянно
   0B00001111, //Мигание по 0.5 сек
   0B00000001, //Короткая вспышка раз в секунду
   0B00000101, //Две короткие вспышки раз в секунду
   0B00010101, //Три короткие вспышки раз в секунду
   0B01010101  //Частые короткие вспышки (4 раза в секунду)
};

uint32_t ms, ms1 = 0, ms2 = 0;
uint8_t  blink_loop = 0;
uint8_t  blink_mode = 0;
uint8_t  modes_count = 0;

void setup_led() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    modes_count = 1;
    blink_mode = modes[modes_count];
}

void set_led_mode(int mode) {
    if (modes_count == mode)
        return;

#ifdef USE_SERIAL
    Serial.print(F("set led mode: ")); Serial.println(mode);
#endif
    modes_count = mode;
    blink_mode = modes[mode];
    ms1 = 0;
    blink_loop = 0;
    loop_led();
}

void loop_led() {
    ms = millis();
    // Событие срабатывающее каждые 125 мс   
    if ((ms - ms1) > 125 || ms < ms1) {
        ms1 = ms;
        // Режим светодиода ищем по битовой маске       
        if (blink_mode & 1 << (blink_loop & 0x07)) digitalWrite(LED, HIGH);
        else  digitalWrite(LED, LOW);
        blink_loop++;
    }

    // Этот код служит для демонстрации переключения режимов    
    // Один раз в 5 секунд меняем эффект   
    //if ((ms - ms2) > 5000 || ms < ms2) {
    //    ms2 = ms;
    //    blink_mode = modes[modes_count++];
    //    if (modes_count >= 7)modes_count = 1;
    //}
}