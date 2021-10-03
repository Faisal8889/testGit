#include "Web.h"

//if we format the spiff allow us to re upload all files
const char Upload_page[] PROGMEM = 
R"=====(
<!DOCTYPE html>
<html>

<head>
    <title>ESP8266 SPIFFS File Upload</title>
    <link rel="stylesheet" type="text/css" href="main.css">
</head>

<body>
    <h1>ESP8266 SPIFFS File Upload</h1>
    <p>Select a new file to upload to the ESP8266. Existing files will be replaced.</p>
    <form method="POST" enctype="multipart/form-data">
        <input type="file" name="data">
        <input class="button" type="submit" value="Upload">
    </form>
</body>

</html>
)=====" ;

RUNSTATE runState;
void HandleAPIGET(){
    if( ! server.hasArg("state")) {
    server.send(200, "text/plane", printState(runState));
    }
}

void HandleAPIPOST(){


}


//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
    if (!handleFileRead("/index.html")) {        // check if the file exists in the flash memory (SPIFFS), if so, send it
    server.send(404, "text/plain", "404: File Not Found");
    }
  }

void handleUploadPage() {
  String s = Upload_page; //Read HTML contents
  server.send(200, "text/html", s); //Send web page
}

void handleCellVoltage() {
  digitalWrite(LED, !digitalRead(LED)); //Toggle LED on data request ajax
  String CellVoltages = "";
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    int i;
    for (i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)
    {
      CellVoltages += battery.pack[current_ic].cells[i] * 0.0001;
      CellVoltages += ",";
    }
  }
  CellVoltages += battery.min.voltage * 0.0001 ;
  CellVoltages += ",";
  CellVoltages += battery.max.voltage * 0.0001;
  CellVoltages += ",";
  CellVoltages += battery.average * 0.0001;

  server.send(200, "text/plane", CellVoltages.substring(0, CellVoltages.length()));
}

const char* printState(RUNSTATE state) {

  switch (state) {
    case IDLE:
      return "Idle";
      break;
    case MONITOR: 
      return "Monitor";
      break;

    case CHARGING:
     return "Balancing";
    break; 
      
  }
}

void DeleteDataLog() {

   server.send(200, "text/plane", 
     SPIFFS.remove("/temp.csv")? "file deleted":  "delete failed" );
}

void handleMONITOR() {
  digitalWrite(LED, !digitalRead(LED)); //Toggle LED on data request ajax
  String CellVoltages = "";
  clear_discharge(TOTAL_IC, bms_ic);
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, bms_ic);

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    battery.pack[current_ic].discharge = 0;
    bool gpio[5];
    gpio[GPIO_BACKBAL] = 0;
    LTC681x_set_cfgr_gpio(current_ic,bms_ic,gpio);
  }

  runState = MONITOR;
  server.send(200, "text/plane", "MONITOR");
}



void handleCharging() {
  digitalWrite(LED, !digitalRead(LED)); //Toggle LED on data request ajax
   runState = CHARGING;

  server.send(200, "text/plane", "Balancing");
}

void handleState() {
  digitalWrite(LED, !digitalRead(LED)); //Toggle LED on data request ajax


  float fileTotalKB = (float)SPIFFS.totalBytes() / 1024.0; 
  float fileUsedKB = (float)SPIFFS.usedBytes() / 1024.0; 
  
//  float flashChipSize = (float)ESP.getFlashChipSize() / 1024.0 / 1024.0;
//  float realFlashChipSize = (float)ESP.getFlashChipRealSize() / 1024.0 / 1024.0;
//  float flashFreq = (float)ESP.getFlashChipSpeed() / 1000.0 / 1000.0;
  int index = 0;
  char responce[300];
  index = snprintf(responce,300, "%s<br>%.2f/%.2f KB" ,printState(runState), fileUsedKB ,fileTotalKB);


  for (int i = 0; i < TOTAL_IC; i++)
  {
    index += snprintf(responce+index,300-index, "<br>RAWbypass: %.2f BAL: %.2f Bypasss: %.2f" , (float)bms_ic[i].aux.a_codes[GPIO_BYPASSTEMP-1] * 0.0001,battery.pack[i].balanceTEMP ,  battery.pack[i].bypassTEMP);
 
  }
 

  server.send(200, "text/plane", responce);
}

void SaveData(){
  File cellLog = SPIFFS.open("/temp.csv", FILE_APPEND);
  Serial.println("Data Logged");
      cellLog.print(time((time_t*)0));
      cellLog.print(",");

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)  {

      for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)   {
      cellLog.print(battery.pack[current_ic].cells[i]);
      cellLog.print(",");
      }
  }
    
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)  {
      cellLog.print (battery.pack[current_ic].discharge);
      cellLog.print(",");
    }
    cellLog.println();
    cellLog.close();
}

String getContentType(String filename) { // determine the filetype of a given filename, based on the extension
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path) { // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
      path += ".gz";                                         // Use the compressed verion
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    Serial.println(String("\tSent file: ") + path);
    return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);   // If the file doesn't exist, return false
  return false;
}


void handle_NotFound() {
  if (!handleFileRead(server.uri())) {        // check if the file exists in the flash memory (SPIFFS), if so, send it
    server.send(404, "text/plain", "404: File Not Found");
  }
}

File fsUploadFile;                                    // a File variable to temporarily store the received file

void handleFileUpload() { // upload a new file to the SPIFFS
  HTTPUpload& upload = server.upload();
  String path;
  if (upload.status == UPLOAD_FILE_START) {
    path = upload.filename;
    if (!path.startsWith("/")) path = "/" + path;
    if (!path.endsWith(".gz")) {                         // The file server always prefers a compressed version of a file
      String pathWithGz = path + ".gz";                  // So if an uploaded file is not compressed, the existing compressed
      if (SPIFFS.exists(pathWithGz))                     // version of that file must be deleted (if it exists)
        SPIFFS.remove(pathWithGz);
    }
    Serial.print("handleFileUpload Name: "); Serial.println(path);
    fsUploadFile = SPIFFS.open(path, "w");               // Open the file for writing in SPIFFS (create if it doesn't exist)
    path = String();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if (upload.status == UPLOAD_FILE_END) {
    if (fsUploadFile) {                                   // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      server.sendHeader("Location", "/success.html");     // Redirect the client to the success page
      server.send(303);
    } else {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}
