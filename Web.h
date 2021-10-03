#ifndef WEB_H
#define WEB_H

#include <Arduino.h>
#include <WebServer.h>
#include <FS.h>
#include <SPIFFS.h>

#include "passwords.h"


#include "LTC681x.h"
#include "LTC6811.h"

extern cell_asic bms_ic[TOTAL_IC];

extern WebServer server; //Server on port 80

typedef enum RUNSTATE {IDLE = 0,MONITOR =1, CHARGING = 2} ;

extern RUNSTATE runState;

extern const char* printState(RUNSTATE state);


// extern enum RUNSTATE runState;
// extern const char* printState(enum RUNSTATE);

void HandleAPIGET();
void HandleAPIPOST();

void  DeleteDataLog();
void handleRoot() ;
void handleUploadPage();
void handleCellVoltage();
const char* printState(RUNSTATE state);
void handleMONITOR() ;
void handleState() ;
void handleCharging() ;
void SaveData();
bool handleFileRead(String path);
String getContentType(String filename);

void handle_NotFound();
void handleFileUpload();

#endif