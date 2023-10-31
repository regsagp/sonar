#include <Ultrasonic.h>
#include <CPUVolt.h>

//#include <NewPing.h>

// пины
#define HC_TRIG 3
#define HC_ECHO 2
//#include <NewPing.h>

// указываем пины и макс. расстояние в сантиметрах
//NewPing sonar(HC_TRIG, HC_ECHO, 200);
Ultrasonic sonic(12, 13);

#define LED 10

void setup() {
  Serial.begin(9600);       // для связи
  pinMode(HC_TRIG, OUTPUT); // trig выход
  pinMode(HC_ECHO, INPUT);  // echo вход
  pinMode(LED, OUTPUT);
}

#define CLOSED_DIST 110
#define HAS_HUMAN(x) (x < CLOSED_DIST - 30 && x > 20)

#define LED_OFF_DELAY 2000

float distFilt = 0;

uint32_t tmr_led;
uint32_t tmr_led_off;
uint32_t tmr_print;
uint32_t tmr_volts;
int val = 0;
bool dir = true;
signed long millivolts;

// функция возвращает скорректированное по CRT значение
byte getBrightCRT(byte val) {
  return (0.0003066 * pow(val, 2.46));
}
// для 8 бит ШИМ
byte getBrightCRT2(byte val) {
  return ((long)val * val * val + 130305) >> 16;
}

bool led_on;

void SwitchLed() {
  uint32_t current_time = millis();
  if (millivolts < 4500) {
    if (current_time - tmr_led >= 300) {
      int step = 255;
      tmr_led = current_time;
      if (dir) val += step; // увеличиваем яркость
      else val -= step;   // уменьшаем
      if (val >= 255 || val <= 0) dir = !dir; // разворачиваем
      int max_val = 255;
      if (val > max_val)
        val > max_val;
      else if (val < 0)
        val = 0;
      analogWrite(LED, val);
    }
    return;
  }
  
  if (distFilt < 200)
    led_on = HAS_HUMAN(distFilt);

  if (led_on || val > 30) {
    tmr_led_off = current_time;
    if (current_time - tmr_led >= 20) {
      int step = 3;
      tmr_led = current_time;
      if (dir) val += step; // увеличиваем яркость
      else val -= step;   // уменьшаем
      if (val >= 255 || val <= 0) dir = !dir; // разворачиваем
      analogWrite(LED, getBrightCRT(val) / 4);
    }
  }
  else {
    if(current_time - tmr_led_off > LED_OFF_DELAY)
      analogWrite(LED, 0);
  }
}

void loop() {
  SwitchLed();

  if (millis() - tmr_print >= 400) {
    float dist = readDist();
    tmr_print = millis();
    uint32_t current_time = tmr_print;
    Serial.print("led=");      Serial.print(current_time - tmr_led_off > LED_OFF_DELAY ? 0 : current_time - tmr_led_off);
    Serial.print(", dist=");    Serial.print(distFilt);
    Serial.print(", ");     Serial.println(dist);
  }

  if (millis() - tmr_volts >= 1 * 1000) {
    tmr_volts = millis();
    millivolts = readVcc();
    /*
      char voltstr[8];
      dtostrf(millivolts/1000.0, 5, 2, voltstr);
      Serial.print("voltage: ");
      Serial.println(voltstr);
    */
    //Serial.print("vcc=");      Serial.print(0.001 * millivolts);
  }

}

float readDist() {
  //float dist = getDist();       // получаем расстояние
  float dist = sonic.read(CM);
  if (dist > 200)
    dist = 0;
  
  distFilt += (dist - distFilt) * 0.2;  // фильтруем
  
  return dist;
}

// сделаем функцию для удобства
float getDist() {
  // импульс 10 мкс
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_TRIG, LOW);

  // измеряем время ответного импульса
  uint32_t us = pulseIn(HC_ECHO, HIGH);

  // считаем расстояние и возвращаем
  return (us / 58.3);
}
