#include "time.h"
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 10*3600;
const int   daylightOffset_sec = 3600;


void setupTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

}