#pragma once

#define BLYNK_TEMPLATE_ID           "TMPLrV30J4jZ"
#define BLYNK_DEVICE_NAME           "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "6udSeBvnyEMFv6xlBKz7cBJeV1H6uKkQ"

#define WIFI_61 1
#define WIFI_MEGA 2
#define WIFI_P30 3
#define WIFI_IPHONE 4

// Your WiFi credentials.
// Set password to "" for open networks.
#if WIFI_ == WIFI_61 //61
char ssid[] = "Mgts kv 61";
char pass[] = "92008310";
#elif WIFI_ == 2
char ssid[] = "MegaFonMR100-3-76B0";
char pass[] = "D8EA3J3N";
#elif WIFI_ == 3
char ssid[] = "HUAWEI P30";
char pass[] = "21a62778cbba";
#elif WIFI_ == 4
char ssid[] = "iPhone a.pervov";
char pass[] = "6xaipxkke89fp";
#endif

