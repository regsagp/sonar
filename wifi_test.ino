#ifdef USE_WIFI

/*
    This sketch demonstrates how to set up a simple HTTP-like server.
    The server will set a GPIO pin depending on the request
      http://server_ip/gpio/0 will set the GPIO2 low,
      http://server_ip/gpio/1 will set the GPIO2 high
    server_ip is the IP address of the ESP8266 module, will be
    printed to Serial when the module is connected.
*/

#include <ESP8266WiFi.h>
#define WIFI_  WIFI_61
#include "Credentials.h"

#ifndef STASSID
#define STASSID ssid
#define STAPSK  pass
#endif

//const char* ssid = STASSID;
const char* password = pass;

//#ifdef USE_WIFI
WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;
WiFiEventHandler stationModeGotIP;
//#endif
int counter;
// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);
unsigned long tmr_wifi_connect;
void setup_wifi() {
    set_led_mode(4);// two splash

    // Connect to WiFi network
    Serial.println();
    Serial.println();
    Serial.print(F("Connecting to "));
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);


    // Register event handlers.
    //WiFiEventHandler onStationModeConnected(std::function<void(const WiFiEventStationModeConnected&)>);
    //WiFiEventHandler onStationModeDisconnected(std::function<void(const WiFiEventStationModeDisconnected&)>);
    //WiFiEventHandler onStationModeAuthModeChanged(std::function<void(const WiFiEventStationModeAuthModeChanged&)>);
    //WiFiEventHandler onStationModeGotIP(std::function<void(const WiFiEventStationModeGotIP&)>);

    // Callback functions will be called as long as these handler objects exist.
    // Call "onStationConnected" each time a station connects
    stationConnectedHandler = WiFi.onStationModeConnected(&onStationConnected);
    // Call "onStationDisconnected" each time a station disconnects
    stationDisconnectedHandler = WiFi.onStationModeDisconnected(&onStationDisconnected);
    // Call "onProbeRequestPrint" and "onProbeRequestBlink" each time
    // a probe request is received.
    // Former will print MAC address of the station and RSSI to Serial,
    // latter will blink an LED.
    stationModeGotIP = WiFi.onStationModeGotIP(&onStationModeGotIP);

    /*
    int n_repeats = 240; // 120 sec
    while (true) {
        if (millis() - tmr_wifi_connect >= 500) {
            tmr_wifi_connect = millis();

            if (WiFi.status() == WL_CONNECTED || --n_repeats <= 0)
                break;
            delay(50);
            Serial.print(F("."));
        } 

        loop_led();
    }

    set_led_mode(0);// off

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WiFi NOT connected"));
        return;
    }
    OnConnected();
    */

}

void OnConnected() {
    Serial.println();
    Serial.println(F("WiFi connected"));

    // Start the server
    server.begin();
    Serial.println(F("Server started"));

    // Print the IP address
    Serial.println(WiFi.localIP());
}

bool blinkFlag;
bool connected;

String macToString(const unsigned char* mac) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
}

void onStationConnected(const WiFiEventStationModeConnected& evt) {
    Serial.print("Station connected on channel: ");
    Serial.println(evt.channel);
    connected = true;

    OnConnected();
}

void onStationDisconnected(const WiFiEventStationModeDisconnected& evt) {
    if (connected) {
        Serial.print("Station disconnected by reason: ");
        Serial.println(evt.reason);
    }
    connected = false;
}

void onStationModeGotIP(const WiFiEventStationModeGotIP& evt) {
    Serial.print("Station got ip: ");
    Serial.println(evt.ip);
    // We can't use "delay" or other blocking functions in the event handler.
    // Therefore we set a flag here and then check it inside "loop" function.
    //blinkFlag = true;
}

esp8266::polledTimeout::periodicMs printTimeout(1000);

void loop_wifi() {
    if (!connected) {
        if(printTimeout)
            Serial.println("wifi not connected");
        return;
    }
    counter++;
    // Check if a client has connected
    WiFiClient client = server.available();// accept();
    if (!client) {
        return;
    }

    if (client) {
        Serial.println("New client");  //  "Новый клиент"
        // создаем переменную типа «boolean»,
        // чтобы определить конец HTTP-запроса:
        boolean blank_line = true;

        char s_raw_dist[8]; dtostrf(rawDist, 3, 0, s_raw_dist);
        char s_filt_dist[8];     dtostrf(distFilt, 3, 0, s_filt_dist);

        while (client.connected()) {
            //Serial.println("client connected");  //  "Новый клиент"
                // Read the first line of the request
            //String req = client.readStringUntil('\r');
            //Serial.print(F("  request: [")); Serial.print(req); Serial.println("]");

            if (client.available()) {
                char c = client.read();
                Serial.print(c);  //  "Новый клиент"


                if (c == '\n' && blank_line) {
                    //getTemperature();
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println();
                    client.println("<!DOCTYPE HTML>");
                    client.println("<html>");
                    client.println("<head><link rel=\"icon\" href=\"data:,\"></head><body><h1>ESP8266</h1><h3>Raw dist: ");
                    client.println(s_raw_dist);
                    client.println(" cm</h3><h3>filt dist: ");
                    client.println(s_filt_dist); 
                    client.println(" cm </h3><h3>room busy: ");
                    client.println(led_on);
                    client.println(" </h3><h3>voltage: ");
                    client.println(millivolts);
                    client.println("mV</h3><h3>counter: ");
                    client.println(counter);
                    client.println("</h3></body></html>");
                    break;
                }
                if (c == '\n') {
                    // если обнаружен переход на новую строку:
                    blank_line = true;
                }
                else if (c != '\r') {
                    // если в текущей строчке найден символ:
                    blank_line = false;
                }
            }
            else {
                //Serial.println("no available data");
                //break;
            }
            yield;
        }
        // close client
        delay(1);
        client.stop();
        Serial.println("Client disconnected.");
    }


    /*

    Serial.println(F("new client {"));

    client.setTimeout(5000); // default is 1000

    // Read the first line of the request
    String req = client.readStringUntil('\r');
    Serial.print(F("  request: [")); Serial.print(req); Serial.println("]");

    // Match the request
    int val;
    if (req.indexOf(F("/gpio/0")) != -1) {
        val = 0;
    }
    else if (req.indexOf(F("/gpio/1")) != -1) {
        val = 1;
    }
    else {
        Serial.println(F("  invalid request"));
        val = digitalRead(LED_BUILTIN);
    }

    Serial.print(("  led = ")); Serial.println(val);


    // Set LED according to the request
    digitalWrite(LED_BUILTIN, val);

    // read/ignore the rest of the request
    // do not client.flush(): it is for output only, see below
    while (client.available()) {
        // byte by byte is not very efficient
        client.read();
    }

    // Send the response to the client
    // it is OK for multiple small client.print/write,
    // because nagle algorithm will group them into one single packet
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\nGPIO is now "));
    client.print((val) ? F("high") : F("low"));
    client.print(F("<br><br>Click <a href='http://"));
    client.print(WiFi.localIP());
    client.print(F("/gpio/1'>here</a> to switch LED GPIO on, or <a href='http://"));
    client.print(WiFi.localIP());
    client.print(F("/gpio/0'>here</a> to switch LED GPIO off.</html>"));

    // The client will actually be *flushed* then disconnected
    // when the function returns and 'client' object is destroyed (out-of-scope)
    // flush = ensure written data are received by the other side
    Serial.println(F("} Disconnecting from client"));
    */
}
#endif