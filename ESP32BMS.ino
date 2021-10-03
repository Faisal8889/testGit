/*!
  Battery management system usign a ltc6811 and esp32 master
*/

#include <Arduino.h>
#include <stdint.h>


#include "index.h"



#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include <SPI.h>


#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/


//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See LTC6811_daisy.h for Options

const uint16_t MEASUREMENT_LOOP_TIME = 5*6000;//milliseconds(mS)
const uint16_t BALANCE_LOOP_TIME = 60000;//milliseconds(mS)



//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED
/************************************
  END SETUP
*************************************/




char get_char();
void print_menu();
void read_config_data(uint8_t cfg_data[][6], uint8_t nIC);
void print_cells(uint8_t datalog_en);
void print_open();
void print_config();
void print_rxconfig();
void print_aux(uint8_t datalog_en);
void print_stat();
void check_error(int error);

/******************************************************
 *** Global Battery Variables received from 681x commands
  These variables store the results from the LTC6811
  register reads and the array lengths must be based
  on the number of ICs on the stack
 ******************************************************/


#include <WebServer.h>
WebServer  server(80);

#include "WiFi.h"
#include "ESPmDNS.h"
#include "WiFiUdp.h"
#include "ArduinoOTA.h"
#include "Web.h"
#include "TimeManager.h"



#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>

#include "passwords.h"

cell_asic bms_ic[TOTAL_IC];


//==============================================================
//                  SETUP
//==============================================================



void connectWIFI() {
  // Connect the ESP32 the the wifi AP
  WiFi.mode(WIFI_STA);

  unsigned long timeout = millis();
  int i = 0;
  do {
     i = ++i % PASSWORDS;
    Serial.print("WiFi connecting to");
  Serial.println(ssid[i]);
   
    WiFi.begin(ssid[i], password[i]);
    
    timeout = millis();
    while ((WiFi.status() != WL_CONNECTED) && ((millis() - timeout) < (5 * 1000)) ) {
      delay(500);
      Serial.print(".");
    }
  }  while (WiFi.status() != WL_CONNECTED);

  Serial.print("WiFi connected to");
  Serial.println(ssid[i]);
  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
    
}

/*
  Taken form the ESP32 OTA Example Program
*/

void setupOTA() {
  #ifdef DEVBAORD
   ArduinoOTA.setHostname("esp32-LTC6811-DEV");
#else
   ArduinoOTA.setHostname("esp32-LTC6811-BMS");
#endif


  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    //serial.println("Start updating " + type);
  })
  .onEnd([]() {
    //serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    //serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    //serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}




/*!**********************************************************************
  \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup() {

  Serial.begin(115200);
  SPIFFS.begin(true);  
  connectWIFI();
  setupOTA();
  setupTime();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  server.on("/", handleRoot);      //Which routine to handle at root location. This is display page
  server.on("/readCells", handleCellVoltage);

  server.on("/charging", handleCharging);
  server.on("/monitor", handleMONITOR);
  server.on("/state", handleState);

  server.on("/delete", DeleteDataLog);
  
  

   server.on("/upload",  HTTP_POST, []() {  // If a POST request is sent to the /edit.html address, 
    server.send(200, "text/plain", "");
  }, handleFileUpload);                       // go to 'handleFileUpload'

  server.on("/upload", HTTP_GET, [](){handleUploadPage(); } );                 // if the client requests the upload page
   

  server.onNotFound(handle_NotFound);

  server.begin();

  spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC, bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC, bms_ic);
  bms_ic->isospi_reverse = false;
  print_menu();

  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, bms_ic);  
}




int saveCount = 0;

CellData* PackToSort;

// qsort requires you to create a compare function
int sort_desc(const void *cmp1, const void *cmp2) {
  // Need to cast the void * to int *
  int a = PackToSort->cells[*((int *)cmp1)];
  int b = PackToSort->cells[*((int *)cmp2)];
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}




void BalanceCells() {
  int sortedCells[12];


  clear_discharge(TOTAL_IC, bms_ic);
  

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    battery.pack[current_ic].discharge = 0;

    for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)
    {
      sortedCells[i] = i;
    }

    PackToSort = &battery.pack[current_ic];
    qsort(sortedCells, bms_ic[0].ic_reg.cell_channels, sizeof(sortedCells[0]), sort_desc);

    Serial.println("Cells largest to smalest: ");
    
    Serial.print("Cell: "); Serial.println(sortedCells[0]+1);
    for (int i = 0; i < bms_ic[0].ic_reg.cell_channels - 1; i++)
    {
       Serial.print("Cell: "); Serial.println(sortedCells[i+1]+1);
      if(sortedCells[i] == -1) {
        continue;
      }
     
     for (int cell = i; cell < 12 - 1; cell++) {
        // if cells are next to each other on the bms remove. no adjectent cells can be balanced at a time
      if (sortedCells[i] + 1 == sortedCells[cell + 1]) {
        sortedCells[cell + 1] = -1;
      }
      
      if (sortedCells[i] - 1 == sortedCells[cell + 1]) {
        sortedCells[cell + 1] = -1;
      }
     }
     
    }

    for (int i = 0, count = 0; count < 5 && i < 12; i++) //max number of cells to ballance per IC
    {
      if (sortedCells[i] != -1 && 
            (PackToSort->cells[sortedCells[i]] > battery.pack[current_ic].average + 0.001 / 0.0001 ||
              PackToSort->cells[sortedCells[i]] > battery.average + 0.001  / 0.0001 
            )){ //0.01 V above average.
        Serial.print("Discharging: "); Serial.println(12 * current_ic + sortedCells[i]+1);
        LTC6811_set_discharge(12 * (1-current_ic) + sortedCells[i]+1 , TOTAL_IC, bms_ic);
        battery.pack[current_ic].discharge |= 1<< sortedCells[i];
        count++;
      }
    }
    

  }
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, bms_ic);
  print_config();
  SaveData();
}


void ReadBattery() {
  wakeup_sleep(TOTAL_IC);
  clear_discharge(TOTAL_IC, bms_ic);

//turn off bypass
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)  {
      bool gpio[5] = {1,1,1,1,1};
     LTC681x_set_cfgr_gpio(current_ic,bms_ic,gpio);
  }


  LTC6811_wrcfg(TOTAL_IC, bms_ic);
  delay(100); // allow the filters to settle 
  measurement_loop(DATALOG_ENABLED);

  saveCount = 0;
  battery.min.voltage =bms_ic[0].cells.c_codes[0];
  battery.min.IC = 0;
  battery.min.cell = 0;

  battery.max.voltage = bms_ic[0].cells.c_codes[0];
  battery.max.IC = 0;
  battery.max.cell = 0;
  battery.sum = 0;


  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
    battery.pack[current_ic].sum = 0;
    battery.pack[current_ic].min.voltage = bms_ic[current_ic].cells.c_codes[0];
    battery.pack[current_ic].max.voltage = bms_ic[current_ic].cells.c_codes[0];

    for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)
    {
      battery.pack[current_ic].cells[i] = bms_ic[current_ic].cells.c_codes[i];
      battery.pack[current_ic].sum += battery.pack[current_ic].cells[i];
      if (battery.pack[current_ic].cells[i] < battery.pack[current_ic].min.voltage ) {
        battery.pack[current_ic].min.voltage =  battery.pack[current_ic].cells[i];
        battery.pack[current_ic].min.IC = current_ic;
        battery.pack[current_ic].min.cell = i;
      }
      if (battery.pack[current_ic].cells[i] > battery.pack[current_ic].max.voltage ) {
        battery.pack[current_ic].max.voltage =  battery.pack[current_ic].cells[i];
        battery.pack[current_ic].max.IC = current_ic;
        battery.pack[current_ic].max.cell = i;
      }
    }
    battery.pack[current_ic].average = battery.pack[current_ic].sum / bms_ic[0].ic_reg.cell_channels;
    battery.sum += battery.pack[current_ic].sum;
    if (battery.min.voltage > battery.pack[current_ic].min.voltage ) {
      battery.min.voltage = battery.pack[current_ic].min.voltage;
      battery.min.IC = battery.pack[current_ic].min.IC;
      battery.min.cell = battery.pack[current_ic].min.cell;
    }

    if (battery.max.voltage < battery.pack[current_ic].max.voltage ) {
      battery.max.voltage = battery.pack[current_ic].max.voltage;
      battery.max.IC = battery.pack[current_ic].max.IC;
      battery.max.cell = battery.pack[current_ic].max.cell;
    }
  }
    battery.average = battery.sum/(TOTAL_IC*bms_ic[0].ic_reg.cell_channels);
}


float thermisitorTEMP(float voltage) {

  float tempature;
  const int  BCOEFFICIENT = 3590;
  
  // convert the value to resistance
  //voltage = 1023 / voltage - 1;
  //voltage = 10000 / voltage;
  
  tempature = voltage/1.5;          // (R/Ro)
  tempature = log(tempature);           // ln(R/Ro)
  tempature /= BCOEFFICIENT;            // 1/B * ln(R/Ro)
  tempature += 1.0 / (25 + 273.15);     // + (1/To)
  tempature = 1.0 / tempature;          // Invert
  tempature -= 273.15;                  // convert to C
  
  return tempature;
}

void readTempatures() {
  int error;
  wakeup_sleep(TOTAL_IC);
  LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_TO_CONVERT);
  LTC6811_pollAdc();

  delay(100);
  error = LTC6811_rdaux(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
  check_error(error);

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)  {
    battery.pack[current_ic].balanceTEMP = thermisitorTEMP(bms_ic[current_ic].aux.a_codes[GPIO_BALTEMP-1] * 0.0001);
    battery.pack[current_ic].bypassTEMP = thermisitorTEMP(bms_ic[current_ic].aux.a_codes[GPIO_BYPASSTEMP-1] * 0.0001);
  }
}

unsigned long lastBatRead;
unsigned long lastDischarge;
unsigned long keepAwake;


// void BalancePacks() {
//   // for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
// //   {
// //     bool gpio[5];
// //     gpio[GPIO_BACKBAL] = 0;
// //     LTC681x_set_cfgr_gpio(current_ic,bms_ic,gpio);    
// //   }


//       bool gpio[5];
//     if (battery.pack[current_ic].average > battery.average) {
//       gpio[GPIO_BACKBAL] = 1;
//    //   LTC681x_set_cfgr_gpio(current_ic,bms_ic,gpio);
//     } else {
//       gpio[GPIO_BACKBAL] = 0;
//       LTC681x_set_cfgr_gpio(current_ic,bms_ic,gpio);
//     }
// }

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{
  ArduinoOTA.handle();
  server.handleClient();          //Handle client requests

  if (millis() - keepAwake > 1500) { //poll every 1.5s
    keepAwake = millis();
    wakeup_sleep(TOTAL_IC);
    LTC6811_rdcfg(TOTAL_IC, bms_ic);


    if(runState == CHARGING) {
      //manage bypass here as it gets really hot. 
      readTempatures();

      for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)  {
        bool gpio[5] = {1,1,1,1,1};
        if (battery.pack[current_ic].bypassTEMP > 50.0) {
           Serial.print("Cell: ");
          Serial.print(current_ic);
          Serial.print("Bypass too hot: ");
          Serial.println(battery.pack[current_ic].bypassTEMP);
        }
        if (battery.pack[current_ic].bypassTEMP < 50.0 && battery.pack[current_ic].average > (battery.average + 0.001 / 0.0001)) {
          //death....
          gpio[GPIO_BACKBAL-1] = 0; //open drain ON
          Serial.print("Bypass balancing: ");
          Serial.println(current_ic);
        } else {
          gpio[GPIO_BACKBAL-1] = 1;
        }
        LTC681x_set_cfgr_gpio(TOTAL_IC - current_ic - 1 ,bms_ic,gpio);
        
      }
      LTC6811_wrcfg(TOTAL_IC, bms_ic);
    }
  }

  switch (runState) {
    case MONITOR:
       if (millis() - lastDischarge > MEASUREMENT_LOOP_TIME) {
        lastDischarge = millis();
        readTempatures();
        ReadBattery();
        SaveData();
      }

      break;
    case CHARGING:
      if (millis() - lastDischarge > BALANCE_LOOP_TIME) {
        lastDischarge = millis();
        readTempatures();
        ReadBattery();
        BalanceCells();
      }
      break;
  }


  if (WiFi.status() != WL_CONNECTED) {
    connectWIFI();
  }
  else {
    if (Serial.available())           // Check for user input
    {
      uint32_t user_command;
      user_command = read_int();      // Read the user command
      Serial.println(user_command);
      run_command(user_command);
    }
  }
}

/*!*****************************************
  \brief executes the user command
*******************************************/

void run_command(uint32_t cmd)
{
  int8_t error = 0;
  uint32_t conv_time = 0;
  uint32_t user_command;
  int8_t readIC = 0;
  char input = 0;
  switch (cmd)
  {

    case 1: // Write Configuration Register
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, bms_ic);
      print_config();
      break;
 
    case 2: // Read Configuration Register
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcfg(TOTAL_IC, bms_ic);
      check_error(error);
      print_rxconfig();
      break;

    case 3: // Start Cell ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
      conv_time = LTC6811_pollAdc();
      Serial.print(F("cell conversion completed in:"));
      Serial.print(((float)conv_time / 1000), 1);
      Serial.println(F("mS"));
      Serial.println();
      break;

    case 4: // Read Cell Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdcv(0, TOTAL_IC, bms_ic); // Set to read back all cell voltage registers
      check_error(error);
      print_cells(DATALOG_DISABLED);
      break;

    case 5: // Start GPIO ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_TO_CONVERT);
      LTC6811_pollAdc();
      Serial.println(F("aux conversion completed"));
      Serial.println();
      break;

    case 6: // Read AUX Voltage Registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdaux(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
      check_error(error);
      print_aux(DATALOG_DISABLED);
      break;

    case 7: // Start Status ADC Measurement
      wakeup_sleep(TOTAL_IC);
      LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
      LTC6811_pollAdc();
      Serial.println(F("stat conversion completed"));
      Serial.println();
      break;

    case 8: // Read Status registers
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_rdstat(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
      check_error(error);
      print_stat();
      break;

    case 9: // Loop Measurements
      Serial.println(F("transmit 'm' to quit"));
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, bms_ic);
      while (input != 'm')
      {
        if (Serial.available() > 0)
        {
          input = read_char();
        }

        measurement_loop(DATALOG_DISABLED);

        delay(MEASUREMENT_LOOP_TIME);
      }
      //print_menu();
      break;

    case 10: // Run open wire self test
      print_pec();

      break;

    case 11: // Read in raw configuration data
      LTC6811_reset_crc_count(TOTAL_IC, bms_ic);
      break;

    case 12:  // Run the ADC/Memory Self Test
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(CELL, ADC_CONVERSION_MODE, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and CELL Memory \n"));

      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(AUX, ADC_CONVERSION_MODE, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and AUX Memory \n"));

      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_cell_adc_st(STAT, ADC_CONVERSION_MODE, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in Digital Filter and STAT Memory \n"));
      print_menu();
      break;

    case 13: // Enable a discharge transistor
      Serial.println(F("Please enter the Spin number"));
      readIC = (int8_t)read_int();
      LTC6811_set_discharge(readIC, TOTAL_IC, bms_ic);
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, bms_ic);
      print_config();
      break;

    case 14: // Clear all discharge transistors
      clear_discharge(TOTAL_IC, bms_ic);
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, bms_ic);
      print_config();
      break;

    case 15: // Clear all ADC measurement registers
      wakeup_sleep(TOTAL_IC);
      LTC6811_clrcell();
      LTC6811_clraux();
      LTC6811_clrstat();
      Serial.println(F("All Registers Cleared"));
      break;

    case 16: // Run the Mux Decoder Self Test
      wakeup_sleep(TOTAL_IC);
      LTC6811_diagn();
      delay(5);
      error = LTC6811_rdstat(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
      check_error(error);
      error = 0;
      for (int ic = 0; ic < TOTAL_IC; ic++)
      {
        if (bms_ic[ic].stat.mux_fail[0] != 0) error++;
      }
      if (error == 0) Serial.println(F("Mux Test: PASS "));
      else Serial.println(F("Mux Test: FAIL "));

      break;

    case 17: // Run ADC Overlap self test
      wakeup_sleep(TOTAL_IC);
      error = (int8_t)LTC6811_run_adc_overlap(TOTAL_IC, bms_ic);
      if (error == 0) Serial.println(F("Overlap Test: PASS "));
      else Serial.println(F("Overlap Test: FAIL"));
      break;

    case 18: // Run ADC Redundancy self test
      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE, AUX, TOTAL_IC, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in AUX Measurement \n"));

      wakeup_sleep(TOTAL_IC);
      error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE, STAT, TOTAL_IC, bms_ic);
      Serial.print(error, DEC);
      Serial.println(F(" : errors detected in STAT Measurement \n"));
      break;

    case 19:
      LTC6811_run_openwire(TOTAL_IC, bms_ic);
      print_open();
      break;

    case 20: //Datalog print option Loop Measurements
      Serial.println(F("transmit 'm' to quit"));
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcfg(TOTAL_IC, bms_ic);
      while (input != 'm')
      {
        if (Serial.available() > 0)
        {
          input = read_char();
        }

        measurement_loop(DATALOG_ENABLED);

        delay(MEASUREMENT_LOOP_TIME);
      }
      print_menu();
      break;

    case 'm': //prints menu
      print_menu();
      break;

    case 21:
      runState = MONITOR;
      break;
    case 22:
      runState = CHARGING;
      break;

    case 30:
        Serial.printf("Deleting file: %s\r\n", "/temp.csv");
    if(SPIFFS.remove("/temp.csv")){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }

      break;
      
    default:
      Serial.println(F("Incorrect Option"));
      break;
  }
}

void measurement_loop(uint8_t datalog_en)
{
  int8_t error = 0;
  if (WRITE_CONFIG == ENABLED)
  {
    wakeup_sleep(TOTAL_IC);
    LTC6811_wrcfg(TOTAL_IC, bms_ic);
    print_config();
  }

  if (READ_CONFIG == ENABLED)
  {
    wakeup_sleep(TOTAL_IC);
    error = LTC6811_rdcfg(TOTAL_IC, bms_ic);
    check_error(error);
    print_rxconfig();
  }

  if (MEASURE_CELL == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6811_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdcv(0, TOTAL_IC, bms_ic);
    check_error(error);
    // print_cells(datalog_en);

  }

  if (MEASURE_AUX == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdaux(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
    check_error(error);
    print_aux(datalog_en);
  }

  if (MEASURE_STAT == ENABLED)
  {
    wakeup_idle(TOTAL_IC);
    LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
    LTC6811_pollAdc();
    wakeup_idle(TOTAL_IC);
    error = LTC6811_rdstat(0, TOTAL_IC, bms_ic); // Set to read back all aux registers
    check_error(error);
    print_stat();
  }

  if (PRINT_PEC == ENABLED)
  {
    print_pec();
  }

}


/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu()
{
  Serial.println(F("Please enter LTC6811 Command"));
  Serial.println(F("Write Configuration: 1            | Reset PEC Counter: 11 "));
  Serial.println(F("Read Configuration: 2             | Run ADC Self Test: 12"));
  Serial.println(F("Start Cell Voltage Conversion: 3  | Set Discharge: 13"));
  Serial.println(F("Read Cell Voltages: 4             | Clear Discharge: 14"));
  Serial.println(F("Start Aux Voltage Conversion: 5   | Clear Registers: 15"));
  Serial.println(F("Read Aux Voltages: 6              | Run Mux Self Test: 16"));
  Serial.println(F("Start Stat Voltage Conversion: 7  | Run ADC overlap Test: 17"));
  Serial.println(F("Read Stat Voltages: 8             | Run Digital Redundancy Test: 18"));
  Serial.println(F("loop Measurements: 9              | Run Open Wire Test: 19"));
  Serial.println(F("Read PEC Errors: 10               |  Loop measurements with datalog output: 20"));
  Serial.println(F("States, MONITOR: 21, Charging: 22, Delete Datastore: 30"));
  Serial.println(F("Please enter command: "));
  Serial.println();
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void print_cells(uint8_t datalog_en)
{


  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      Serial.print(", ");
      for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)
      {

        Serial.print(" C");
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.println();
    }
    else
    {
      Serial.print("Cells, ");
      for (int i = 0; i < bms_ic[0].ic_reg.cell_channels; i++)
      {
        Serial.print(bms_ic[current_ic].cells.c_codes[i] * 0.0001, 4);
        Serial.print(",");
      }

    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open()
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (bms_ic[current_ic].system_open_wire == 0)
    {
      Serial.print("No Opens Detected on IC: ");
      Serial.print(current_ic + 1, DEC);
      Serial.println();
    }
    else
    {
      for (int cell = 0; cell < bms_ic[0].ic_reg.cell_channels + 1; cell++)
      {
        if ((bms_ic[current_ic].system_open_wire & (1 << cell)) > 0)
        {
          Serial.print(F("There is an open wire on IC: "));
          Serial.print(current_ic + 1, DEC);
          Serial.print(F(" Channel: "));
          Serial.println(cell, DEC);
        }
      }
    }
  }
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      Serial.print(" IC ");
      Serial.print(current_ic + 1, DEC);
      for (int i = 0; i < 5; i++)
      {
        Serial.print(F(" GPIO-"));
        Serial.print(i + 1, DEC);
        Serial.print(":");
        Serial.print(bms_ic[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
      Serial.print(F(" Vref2"));
      Serial.print(":");
      Serial.print(bms_ic[current_ic].aux.a_codes[5] * 0.0001, 4);
      Serial.println();
    }
    else
    {
      Serial.print("AUX, ");

      for (int i = 0; i < 6; i++)
      {
        Serial.print(bms_ic[current_ic].aux.a_codes[i] * 0.0001, 4);
        Serial.print(",");
      }
    }
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat()
{

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" SOC:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[0] * 0.0001 * 20, 4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[1] * 0.0001, 4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[2] * 0.0001, 4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(bms_ic[current_ic].stat.stat_codes[3] * 0.0001, 4);
    Serial.println();
  }

  Serial.println();
}

/*!******************************************************************************
  \brief Prints the configuration data that is going to be written to the LTC6811
  to the serial port.
 ********************************************************************************/
void print_config()
{
  int cfg_pec;

  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.tx_data[5]);
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &bms_ic[current_ic].config.tx_data[0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

/*!*****************************************************************
  \brief Prints the configuration data that was read back from the
  LTC6811 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
  Serial.println(F("Received Configuration "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[0]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[1]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[2]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[3]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[4]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[6]);
    Serial.print(F(", 0x"));
    serial_print_hex(bms_ic[current_ic].config.rx_data[7]);
    Serial.println();
  }
  Serial.println();
}

void print_pec()
{
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.println("");
    Serial.print(bms_ic[current_ic].crc_count.pec_count, DEC);
    Serial.print(F(" : PEC Errors Detected on IC"));
    Serial.println(current_ic + 1, DEC);
  }
}


void serial_print_hex(uint8_t data)
{
  if (data < 16)
  {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  }
  else
    Serial.print((byte)data, HEX);
}

//Function to check error flag and print PEC error message
void check_error(int error)
{
  if (error == -1)
  {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}


// hex conversion constants
char hex_digits[16] =
{
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

// global variables

char hex_to_byte_buffer[5] =
{
  '0', 'x', '0', '0', '\0'
};               // buffer for ASCII hex to byte conversion
char byte_to_hex_buffer[3] =
{
  '\0', '\0', '\0'
};

char read_hex()
// read 2 hex characters from the serial buffer and convert
// them to a byte
{
  byte data;
  hex_to_byte_buffer[2] = get_char();
  hex_to_byte_buffer[3] = get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return (data);
}

char get_char()
{
  // read a command from the serial port
  while (Serial.available() <= 0);
  return (Serial.read());
}
