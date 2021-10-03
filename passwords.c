#include "passwords.h"



const uint16_t OV_THRESHOLD = 41500; // Over voltage threshold ADC Code. LSB = 0.0001    //also set in LTC681x.cpp Line: 1443
const uint16_t UV_THRESHOLD = 36500; // Under voltage threshold ADC Code. LSB = 0.0001


Battery battery;

extern const char* ssid[PASSWORDS]     = {"DESKTOP-Brad", "cookie_mansion", "Stargate"};
extern const char* password[PASSWORDS] = {"12345678", "cookies&cream", "puddlejumper"};