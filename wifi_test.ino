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

    Serial.println();
    Serial.println(F("WiFi connected"));

    // Start the server
    server.begin();
    Serial.println(F("Server started"));

    // Print the IP address
    Serial.println(WiFi.localIP());
}

void loop_wifi() {
    // Check if a client has connected
    WiFiClient client = server.available();
    if (!client) {
        return;
    }

    if (client) {
        Serial.println("New client");  //  "����� ������"
        // ������� ���������� ���� �boolean�,
        // ����� ���������� ����� HTTP-�������:
        boolean blank_line = true;

        char s_volt[8];
        dtostrf(millivolts, 3, 2, s_volt);

        while (client.connected()) {
            //Serial.println("client connected");  //  "����� ������"
                // Read the first line of the request
            //String req = client.readStringUntil('\r');
            //Serial.print(F("  request: [")); Serial.print(req); Serial.println("]");

            if (client.available()) {
                char c = client.read();
                Serial.print(c);  //  "����� ������"


                if (c == '\n' && blank_line) {
                    //getTemperature();
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println();
                    client.println("<!DOCTYPE HTML>");
                    client.println("<html>");
                    client.println("<head><link rel=\"icon\" href=\"data:,\"></head><body><h1>ESP8266</h1><h3>Voltage: ");
                    client.println(s_volt);
                    client.println(" mV</h3><h3>esp voltage: ");
                    client.println(ESP.getVcc()); // works only if A0 not connected
                    client.println(" mV</h3><h3>room busy: ");
                    client.println(led_on);
                    client.println("</h3></body></html>");
                    break;
                }
                if (c == '\n') {
                    // ���� ��������� ������� �� ����� ������:
                    blank_line = true;
                }
                else if (c != '\r') {
                    // ���� � ������� ������� ������ ������:
                    blank_line = false;
                }
            }
            else {
                Serial.println("no available data");
                //break;
            }
        }
        // ��������� ���������� � ��������:
        delay(1);
        client.stop();
        Serial.println("Client disconnected.");
        //  "������ ��������."
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
