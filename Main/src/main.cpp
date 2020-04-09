#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <ST7735.h>    //Custom datahead library for ST7735 LCD display
#include "radio.h"
#include "sx126x-board.h"
#include "sx126x.h"
#include "RiggerAssist.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"
#include "SparkfunLSM303C.cpp"
#include <SoftwareSerial.h>
//******************************NOTES***************************************
/* All of the LCD related stuff in in ST7735.h and ST7735.cpp,
  this include all pin defs, animation stuff. It's wrapped in a class
  called ST7735.

  All IMU code is located in DPEnd_ICM20948 lib files.

  #define HOOKMODULE determines what setup/loop code is run

  TODO:
*/
//**************************************************************************

#define HOOKMODULE

struct rawOrient orient;

struct rxStruct {
  uint8_t data[PACKET_SIZE] = {0, 0, 0, 0};
  uint16_t size;
  int16_t rssi;
  int8_t snr;
  bool newData;
}recvBuff;

struct RA_Packet_t {
    uint8_t id;
    uint8_t ctrl;
    uint8_t data;
    uint8_t chksum;
};

union txUnion {
  RA_Packet_t raPacket;
  uint8_t txArr[4] = {RA_ID,0,0,0};
}txBuff;

volatile int interruptCounter; //need volatile so compiler doesn't remove
int totalInterruptCounter; //Used in main loop thus doesn't need volatile
hw_timer_t * timer = NULL;  //required to setup the timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //Used for syncronization with main loop

enum State {no_cal, cal_in_progress, cal_complete, rtk_state};
uint8_t state;
bool calibrateRequested = false;
bool battCharging = false;
bool timer_started = false;

static RadioEvents_t RadioEvents;
RadioStatus_t status;
bool sendReady = true;                             // Flag for completion of send
bool receiveReady = false;                          // Flag for ready to receive
bool receiving = false;

unsigned long startTime = 0;
unsigned long duration = 0;
volatile int LEDInterruptCounter = 0;
unsigned long currentTime = 0;
double horiz_dist = 0; 
//portMUX_TYPE LEDMux = portMUX_INITIALIZER_UNLOCKED; //Used for syncronization with main loop



//Used to store incoming data from the RTK module
uint8_t char_buffer[72];
char fix_type;
uint8_t fix_counter = 0;
int num_samples = 0;
double average_buffer[60];

long timeout_startTime = 0;

//Instantiate objects
ST7735 LCD; 
TaskHandle_t displayTask, TaskA;
LSM303C myIMU;
// Software Serial on pins 18 and 23
SoftwareSerial GPS_serial(18, 23); // RX, TX

//Not working for some reason...
void CalibrateIMU_ISR()
{
  Serial.println("calibrateISR");
  calibrateRequested = true;
}

void IRAM_ATTR LEDInterrupt_ISR() {
//  portENTER_CRITICAL_ISR(&LEDMux);
//  LEDInterruptCounter++;
//  portEXIT_CRITICAL_ISR(&LEDMux);
currentTime = micros();
duration = currentTime - startTime;
}
// Need PCB alteration for this ISR to work 
// void BattCharging_ISR() {
//   digitalWrite(LED_P, HIGH); //Turn LED on when charging.
//   attachInterrupt(BATT_CHG, BattChargingFin_ISR, LOW);  //Change interrupt to low so know when batt charge finished
//   battCharging = true;
//    Serial.println("Battery charging");
// }

// //Once the chrg pin goes from high to low we need to toggle flag
// void BattChargingFin_ISR() {
//   digitalWrite(LED_P, HIGH); //Turn LED on when charging.
//   attachInterrupt(BATT_CHG, BattCharging_ISR, HIGH);  //Change interrupt to low so know when batt charge finished
//   battCharging = false;
// }

//IRAM_ATTR is used to place ISR in IRAM
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {

  //GPS SERIAL
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for Native USB only
  // }
  Serial.println("All your base are belong to us");

  // set the data rate for the SoftwareSerial port
  GPS_serial.begin(57600);
  // seb comment out
  // Serial.begin(115200);
  // while (!Serial);                                // Wait for serial to start
  // delay(200);                                    //Add a delay for good measure
  
  Serial.print("Starting Rigger Assist as ");
  #ifdef HOOKMODULE
    Serial.print("hook module");
  #else
    Serial.print("datahead");
  #endif
  Serial.print(", version ");
  Serial.println(VERSION);

  InitGPIO();

  Serial.println("Starting LORA...");
  SX126xIoInit();       //Setup GPIO and SPI 
  CalibrateSX1272();
  Serial.println("LORA started");

  #ifdef HOOKMODULE
    int timerAlarmPeriod_ms = 300000;   //300ms
    state = cal_complete;
    //attachInterrupt(CALIBRATE, CalibrateIMU_ISR, LOW); Interrupt not working, possibly no interupts connected to pin SENSOR_CAPP
    // attachInterrupt(BATT_CHG, BattCharging_ISR, LOW); 
    digitalWrite(LED_P, HIGH); //LED on when setting up, flashing when running

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );    // Was 3000000
    Serial.println("LORA Tx config set");

    //Setup IMU sampling on core 0 
    // xTaskCreatePinnedToCore(
    //   Task1,                  /* pvTaskCode */
    //   "Workload",            /* pcName */
    //   4096,                   /* usStackDepth */
    //   NULL,                   /* pvParameters */
    //   1,                      /* uxPriority */
    //   &TaskA,                 /* pxCreatedTask */
    //   0);                     /* xCoreID */

  #else
    int timerAlarmPeriod_ms = 1000000;  //5 seconds
    pinMode(TFT_CS, OUTPUT);    
    pinMode(TFT_LED, OUTPUT);  
//
//    Wire.begin(21,22);//set up I2C bus, comment out if using SPI mode
//       
//    if (myIMU.begin() != IMU_SUCCESS)
//    {
//      Serial.println("Failed setup.");
//      while (1);
//    }

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     LORA_PAYLOAD_LENGTH, true, 0, 0, LORA_IQ_INVERSION_ON, true );                  
    Serial.println("LORA Rx config set");
    Radio.Rx(0); // Continuous Rx
    Serial.println("Receiving...");
    
    //Start the LCD, this will setup LCD, display splash, and run initial animation
    LCD.begin();
    Serial.println("LCD started");

    receiveReady = true;
  #endif

  //Setup timer with diff periods for Hookmodule/Datahead
  timer = timerBegin(0, 80, true);  //Apply prescalar so we can work in uS
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timerAlarmPeriod_ms, true); //Trigger ISR every 5 seconds = 15000000uS (used for batt voltage check)
  timerAlarmEnable(timer);  //Start the timer 
}

void readAllSentences(){
  char new_sentence[150];
  memset(new_sentence,0,150);
//  for (int i = 0; i < 100; i++){
//    ESP_BT.println("Oh hi there");
//    delay(500);
//  }
  
//  while(!GPS_serial.available());
  char incomingByte = GPS_serial.read();
  if (incomingByte == '$'){
    new_sentence[0] = incomingByte;
    int counter = 1;
    bool end_of_sentence = false;
    while(!end_of_sentence){
      while(!GPS_serial.available());
      incomingByte = GPS_serial.read();
      new_sentence[counter] = incomingByte;
      counter++;
      // end of sentence marked with checksum, starts with * then is two more bytes:
      if (incomingByte == '*'){
        end_of_sentence = true;
        while(!GPS_serial.available());
        new_sentence[counter] = GPS_serial.read();
        while(!GPS_serial.available());
        new_sentence[counter+1] = GPS_serial.read();
      }
    }
    
    // send the whole sentence

    // Serial.println(new_sentence);

// check if is GGA
    if (new_sentence[3] == 'G' && new_sentence[4] == 'G' && new_sentence[5] == 'A'){
      // need to filter past a certain number of , to find the fix type, 6 of them
      int comma = 0;
      for (int i = 0; i < 100; i++){
        if(new_sentence[i] == ','){
          comma++;
          if (comma == 6){
            fix_type = (char)new_sentence[i+1];
            i = 100;
            Serial.println("0 = No GPS, 1 = GPS, 2 = RTK");
            Serial.print("Fix Type: ");
            Serial.println(fix_type);
          }
        }
      }
    }
  }
}


void loop() {
  


  // int chrgState = digitalRead(BATT_CHG);
  // Serial.println(chrgState);
  // delay(500);
  //NEED PCB MOD AS OUPUT V FROM CHRG CHIP NOT HIGH ENOUGH 
  // if (battCharging) {
  //   digitalWrite(LED_P, !digitalRead(LED_P));
  //   delay(1000); //Flash led slowly when charging
  //   Serial.println(20);
  //   Serial.println(battCharging);
  //   battCharging = !digitalRead(BATT_CHG); //Check if the battery is charging, low means it is charging
  // }

  // else {
    #ifdef HOOKMODULE
      CheckCalButtonState(!digitalRead(CALIBRATE)); //At some point we should be able to replace with ISR

        // BEGIN GPS SERIAL
    // Serial.write("test");
    readAllSentences();
    /*
    while(!GPS_serial.available()); // wait for the serial port to send data
    uint8_t incomingByte1 = GPS_serial.read();
//    Serial.println(incomingByte1);
    // for the GGA, we want this as the start: 24 2d 2d 47 47 41
        if(incomingByte1 == 0x24){
      //  Serial.print("1");
        while(!GPS_serial.available());
        incomingByte1 = GPS_serial.read();
//        Serial.write(incomingByte1);
        if (incomingByte1 == 0x47){
        //  Serial.print("2");
          while(!GPS_serial.available());
          incomingByte1 = GPS_serial.read();
          if (incomingByte1 == 0x4E){
          //  Serial.print("3");
            while(!GPS_serial.available());
            incomingByte1 = GPS_serial.read();
            if (incomingByte1 == 0x47){
            //  Serial.print("4");
              while(!GPS_serial.available());
              incomingByte1 = GPS_serial.read();
              if (incomingByte1 == 0x47){
              //  Serial.print("5");
                while(!GPS_serial.available());
                incomingByte1 = GPS_serial.read();
                if (incomingByte1 == 0x41){
                //  Serial.print("6");
                  // then we want to wait after 5 "," pass, or 0x2c, then read the next byte.
                  // It will contain the fix type. 4: RTK FIXED, 5: RTK FLOAT
                  uint8_t comma_counter = 0;
                  while (comma_counter < 6){
                    while(!GPS_serial.available());
                    incomingByte1 = GPS_serial.read();
                    if (incomingByte1 == 0x2C){
                      comma_counter++;
                    }
                  }
                  // 0 = invalid
                  // 1 = GPS fix (SPS)
                  // 2 = DGPS fix
                  // 3 = PPS fix
                  // 4 = Real Time Kinematic
                  // 5 = Float RTK
                  // 6 = estimated (dead reckoning) (2.3 feature)
                  // 7 = Manual input mode
                  // 8 = Simulation mode
                  // read the next byte, which should be the fix type:
                  while(!GPS_serial.available());
                  incomingByte1 = GPS_serial.read();
                  Serial.print("Fix Type: ");
                  if(incomingByte1 == 0x34){
                    Serial.println("Fixed RTK");
                    fix_type = 4;
                  } else if(incomingByte1 == 0x35){
                    Serial.println("Float RTK");
                    fix_type = 5;
                  } else if(incomingByte1 == 0x31){
                    Serial.println("GPS only - No RTK");
                    fix_type = 1;
                  } else if(incomingByte1 == 0x30){
                    Serial.println("No GPS Fix");
                    fix_type = 0;
                  } else if(incomingByte1 == 0x32){
                    Serial.println("DPGS Fix");
                    fix_type = 2;
                  } else if(incomingByte1 == 0x33){
                    Serial.println("PPS Fix");
                    fix_type = 3;
                  } else if(incomingByte1 == 0x36){
                    Serial.println("Est Dead Reckoning");
                    fix_type = 6;
                  } else if(incomingByte1 == 0x37){
                    Serial.println("Manual Input");
                    fix_type = 7;
                  } else if(incomingByte1 == 0x38){
                    Serial.println("Simulation");
                    fix_type = 8;
                  }
                  
                  else {
                    Serial.println("Error");
                  }
//                  Serial.println((char) incomingByte1);
                  
                }
              } 
            }
          }
        }
      }*/
      /*
      
    if (incomingByte1 == 0xB5){
        while(!GPS_serial.available());
        uint8_t incomingByte2 = GPS_serial.read();
        if (incomingByte2 == 0x62){
          while(!GPS_serial.available());
          uint8_t incomingByte3 = GPS_serial.read();
          if (incomingByte3 == 0x01){
            while(!GPS_serial.available());
            uint8_t incomingByte4 = GPS_serial.read();
            if (incomingByte4 == 0x3C){
        
        while(!GPS_serial.available());
            char_buffer[0] = incomingByte1;
            char_buffer[1] = incomingByte2;
            char_buffer[2] = incomingByte3;
            char_buffer[3] = incomingByte4;
            for (int i = 4; i < 72; i++){
              while(!GPS_serial.available());
              char_buffer[i] = GPS_serial.read();
            }

            Serial.println("");

            //8-11 is north in, 12-15 is east, 16-19 is down, in cm
            //32 is north, 33 is east, 34 is down, scaled by 0.1 mm
            //6 is the byte offset from the start to the payload
            
            int32_t north_cm = ((uint32_t) char_buffer[11+6] << 24 ) | ((uint32_t) char_buffer[10+6]) << 16 | ((uint32_t) char_buffer[9+6]) << 8 | ((uint32_t) char_buffer[8+6]);
            int8_t north_01_mm = (int32_t) char_buffer[32+6];
            double north_m_1 = north_cm / 100.0;
            double north_m_2 = north_01_mm / 10000.0;
            double north_m = north_m_1 + north_m_2;
          
            // EAST
            int32_t east_cm = ((uint32_t) char_buffer[15+6] << 24 ) | ((uint32_t) char_buffer[14+6]) << 16 | ((uint32_t) char_buffer[13+6]) << 8 | ((uint32_t) char_buffer[12+6]);
            int8_t east_01_mm = (int32_t) char_buffer[33+6];
            double east_m_1 = east_cm / 100.0;
            double east_m_2 = east_01_mm / 10000.0;
            double east_m = east_m_1 + east_m_2;
            
            // DOWN
            int32_t down_cm = ((uint32_t) char_buffer[19+6] << 24 ) | ((uint32_t) char_buffer[18+6]) << 16 | ((uint32_t) char_buffer[17+6]) << 8 | ((uint32_t) char_buffer[16+6]);
            int8_t down_01_mm = (int32_t) char_buffer[34+6];
            double down_m_1 = down_cm / 100.0;
            double down_m_2 = down_01_mm / 10000.0;
            double down_m = down_m_1 + down_m_2;

            // HORIZONTAL DISPLACEMENT
            horiz_dist = sqrt(pow(north_m,2)+pow(east_m,2));
            Serial.print("Horizontal Displacement: ");
            Serial.print(horiz_dist,4);
            Serial.println(" m");

            // ONLY DO AVERAGE IF HAS FLOAT OR FIX LOCK
            if (fix_type == 4 || fix_type == 5){

              // AVERAGE SAMPLE 60
              if (num_samples < 60){
                num_samples++;
              }
              for (int k = 59; k > 0; k--){        
                average_buffer[k] = average_buffer[k-1];
              }
              average_buffer[0] = horiz_dist;
              double sum_average = 0;
              for (int i = 0; i < 60; i++){
                sum_average += average_buffer[i];
              }
              double average_dist = sum_average / (double) num_samples;
              Serial.print("Average: ");
              Serial.println(average_dist,4);

              // MIN MAX AVERAGE
              double min = average_buffer[0];
              double max = average_buffer[0];
              for (int z = 1; z < num_samples; z++){
                if (average_buffer[z] > max){
                  max = average_buffer[z];
                }
                if (average_buffer[z] < min){
                  min = average_buffer[z];
                }
              }
              double minmaxmean = (min + max) / 2.0;
              Serial.print("MinMaxMean: ");
              Serial.println(minmaxmean);
            }

            // reset the character buffer
            for (int k = 0; k < 72; k++){
              char_buffer[k] = 0;
            }
            
        }
        }
        }

        //8-11 is north in, 12-15 is east, 16-19 is down, in cm
        //32 is north, 33 is east, 34 is down, scaled by 0.1 mm

        
      
    }
    */
    // END GPS SERIAL

      //Send IMU data if timer interrupt has occured - which is every 300ms
      if (interruptCounter > 0) {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux); 
        SendDataPacket();
        totalInterruptCounter++;
        // battCharging = !digitalRead(BATT_CHG); //Check if the battery is charging
      }

    #else
      if (recvBuff.newData) {
        recvBuff.newData = false;     // Potential clash with interrupt???
        receiving = true;
        timer_started = false;
        Serial.println(recvBuff.data[1]);
        Serial.println(recvBuff.data[2]);
        if(recvBuff.data[1] == RTK_STATE){
          fix_type = recvBuff.data[2];
        }

        UpdateDisplay();
      
        #ifdef DEBUG_RADIO 
          Serial.print("Data[2]: "); //How is the data output?? Does ISR get called when newData is in???
          Serial.print(" RSSI: ");
          Serial.print(recvBuff.rssi);
          Serial.print(" SNR: ");
          Serial.println(recvBuff.snr);
        #endif

      }
      else { 
        receiving = false;
        UpdateDispOnRxTimeout();
      }

      //Update battery voltage every 3 seconds
      if (interruptCounter > 0) {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux); 
        totalInterruptCounter++;
        updateBattLevelIndicator(ReadBattVoltage());
        AmbientLightAdjust();
      }

    #endif
  // }
}

void InitGPIO() {
pinMode(BATT_CHG, INPUT);
  //attachInterrupt(BATT_CHG, BattCharging_ISR, HIGH); 
  pinMode(LED_P, OUTPUT);               //Init LED pins and turn on LED
  pinMode(LED_N, OUTPUT);
  digitalWrite(LED_N, LOW);
  digitalWrite(LED_P, HIGH);                  // Turn LED on initially
  pinMode(BATT_CHG, INPUT_PULLUP);           // Pullup needed for charging pin on battery charger IC
  pinMode(BATT_VOLT, INPUT);
  pinMode(CALIBRATE, INPUT);
}

void CalibrateSX1272() {
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetPublicNetwork(false);              // Not using LoRaWAN
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Serial.println("Radio events initialised");
}

uint8_t PackSendBits(float inputData) {
  uint8_t packedBits = 0;
  float roundedVal = 0;

  //Need flag to let RX know max angle has been reached, need to set LCD accordinly
  if (inputData > 12.7) {
    packedBits = 12;
  }  
  else if (inputData < -12.7) {
    packedBits = 12;
  }
  else {
    roundedVal = (uint8_t)(inputData * 100 + 0.5);  //Convert float to uint8_t and add 2nd decimal place rounding
    packedBits = roundedVal;
  }

  return packedBits;   
}

void AssignTxBufferContents() {

  // FIELD TEST
  uint8_t packedIMUData;
  fix_counter++;
  if (fix_counter > 3){
    fix_counter = 0;
    // state = rtk_state;
    txBuff.raPacket.ctrl = RTK_STATE;
    // packedIMUData = PackSendBits(fix_type);
    packedIMUData = fix_type;
    Serial.print("Packed IMU Data: ");
    Serial.println(packedIMUData);
    // END FIELD TEST
  }
  else {
    state = cal_complete;
    switch (state) {
    case rtk_state:
      txBuff.raPacket.ctrl = RTK_STATE;
      break;
    case cal_in_progress:
      txBuff.raPacket.ctrl = CAL_IN_PROG; 
      break;
    case cal_complete:
      txBuff.raPacket.ctrl = CAL_COMPLETE;
      break;
    case no_cal:
      txBuff.raPacket.ctrl = NOT_CAL;
      break;
      

    default:
      break;
    }
  
    //Should we put this in the if statement so data is only assigned when radio ready to send?
    packedIMUData = PackSendBits(horiz_dist);
  }
  uint8_t checkSum = chksum8(packedIMUData, 8);
  txBuff.raPacket.data = packedIMUData;
  txBuff.raPacket.chksum = checkSum; //send an additional cehcksum 
  
}

void SendDataPacket() {
  AssignTxBufferContents();
  if (sendReady) {
    //start = millis(); //Used to time lora send 
    sendReady = false;
    Radio.Send(txBuff.txArr, PACKET_SIZE); 
    // Serial.println("TX");     
  }
  flashLED(state);
}

// Receive interrupt, copy over data received. Are we using this properly??
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) {
  if (size == PACKET_SIZE) {
    digitalWrite(LED_P, HIGH);

    for (int i = 0; i < size; ++i) {
      recvBuff.data[i] = payload[i];
      #ifdef DEBUG_IMU
        Serial.print(payload[i]);
        Serial.println("");
      #endif
    }

    recvBuff.rssi = rssi;
    recvBuff.size = size;
    recvBuff.snr = snr;
    recvBuff.newData = true;
    digitalWrite(LED_P, LOW);

  } 
  else {
    Serial.println("Receive error - incorrect packet size received");
  }
  
  return;
}

void OnTxDone(void) {
  sendReady = true;
  //float stop = millis() - start;
  //start = stop;
  //Serial.println(stop);
}

void OnTxTimeout() {
  Serial.println("tx timeout");
}

//Simple 8-bi checksum 
uint8_t chksum8(uint8_t number, size_t len) {
  int digit = 0;
  int sumOfDigits = 0;
  //Add digits of input number
  while (number != 0)
  {
    digit = number % 10;
    //Serial.println(digit);
    sumOfDigits = sumOfDigits + digit;
    number = number / 10;
  }

  return (uint8_t)sumOfDigits;
}

void UpdateDisplay(){
  int8_t rxData = recvBuff.data[2];
  float data = (float)rxData / 100;  //convert the data back to float

  switch (recvBuff.data[1]) {
  case RTK_STATE:
    state = cal_complete;
    LCD.capacity(rxData);
    break;
  case NOT_CAL:
    state = no_cal;
    LCD.NotCalibrated();
    break;
  case CAL_IN_PROG:
    state = cal_in_progress;
    LCD.CalibrationInProgress();
    break;
  case CAL_COMPLETE:
    state = cal_complete;
    if (receiveReady) {
      receiveReady = false;
      // Serial.println("inside if");
      LCD.moveMarker(data);
      receiveReady = true;
    }   
    break;

  default:
    break;
  }

  SetSigStrengthIndicator();
}

void SetSigStrengthIndicator() {
  int8_t SNR = recvBuff.snr;
  int8_t strongSNR = 0;
  int8_t mediumSNR = -5;
  int8_t lowSNR = -15;
  //Need to use SNR as RSSI isn't working well
  if (receiving) {
    if (SNR >= strongSNR) {
      LCD.radioStrength(8); 
    }
    else if (SNR >= mediumSNR) {
      LCD.radioStrength(5); 
    }
    else if  (SNR >= lowSNR) {
      LCD.radioStrength(3); 
    }
  } 
  else {
    LCD.radioStrength(-1); 
  } 

  return;
}

void UpdateDispOnRxTimeout(){
  // static unsigned long timeout_startTime = 0;
  static float timeoutDuration = 3000; //3000ms = 3 seconds

  if (timer_started == false) {
    timer_started = true;
    timeout_startTime = millis();  
  }
  if (millis() - timeout_startTime > timeoutDuration && state != cal_in_progress) {
    timer_started = false;
    LCD.NoSignal();
    
    SetSigStrengthIndicator();
  } 

  return;
}

//Toggle LED flashing period depending on state - calibrated, calibrating, startup state (not calibrated.)
void flashLED(uint8_t state) {
  uint32_t ledFlashPeriod = 0;
  static float lastTime_LEDflash = 0; 

  switch (state) {
  case 0:
    ledFlashPeriod = 10000000; //Stay on for ages
    break;
  case 1:
    ledFlashPeriod = 1000; //when calibrating set led to slow flash
    break;
  case 2:
    ledFlashPeriod = 50;
    break;

  default:
    break;
  }

  if ((millis() - lastTime_LEDflash > ledFlashPeriod)) {
    digitalWrite(LED_P, !digitalRead(LED_P));
    lastTime_LEDflash = millis();
  }

  return;
}

void CheckCalButtonState(uint8_t reading) {
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
  static int lastButtonState = LOW;   // the previous reading from the input pin
  static int buttonState = 0;             // the current reading from the input pin

    if (reading != lastButtonState) {
    // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;

        // only toggle the cal state if the new button state is HIGH
        if (buttonState == HIGH) {
          calibrateRequested = true;
          // state = cal_in_progress;
          #ifdef DEBUG_IMU_ANGLES
            Serial.println("Calibration requested");
          #endif 
          delay(5);
        }

      }
    }
  lastButtonState = reading;
  state = cal_complete;
  return;
}

float ReadBattVoltage() {
  float battVoltage = (analogRead(BATT_VOLT) * 3.3 ) / 4095; //Convert to voltage
  return battVoltage;
}

void updateBattLevelIndicator(float battVoltage) {
  int battLevel = 0;
//  Serial.print("Batt voltage: ");
//  Serial.println(battVoltage);
  LCD.capacity(fix_type);


  if (battVoltage > 1.5) {
    battLevel = 3;
  }
  else if(battVoltage > 1.2) {
    battLevel = 2;
  }
  else {
    battLevel = 1;
  }
  LCD.battBars(battLevel);
  return;
}




//************************************************CORE 0 TASKs*****************************************************
//Task running on core 0
// void Task1( void * parameter ) {
//   uint32_t imuStabilseTime_ms = 40000; //update at 200Hz

//   for (;;) { 
//     if((millis() > imuStabilseTime_ms) && calibrateRequested) {
//       calibrateRequested = false; 
//       CalibrateIMU();  
//       state = cal_complete;
//     }
//     else {
//       updateOrientation(true);
//       delay(5);
//     } 
//   }
//Smooths the angle data using a moving average filter
//Smooths the angle data using a moving average filter
//Smooths the angle data using a moving average filter

//   return;
// }

void AmbientLightAdjust() {
  digitalWrite(LED_P, LOW);
  digitalWrite(LED_N, HIGH);
  unsigned long adjAmb = 0;
  startTime = micros();
  
  pinMode(LED_N, INPUT);
  attachInterrupt(digitalPinToInterrupt(LED_N), LEDInterrupt_ISR, HIGH );
  delay(500);

  if (duration < HIGH_LIGHT_LEVEL)
  {
    adjAmb = TFT_MAX_BRIGHT;
  }
  if (duration > LOW_LIGHT_LEVEL)
  {
    adjAmb = TFT_MID_BRIGHT;
  }
  ledcWrite(PWM_CHANNEL, adjAmb); // LED brightness 0 to 255

  Serial.print(duration);
  Serial.print(" ");
  Serial.println(adjAmb);
  pinMode(LED_N, OUTPUT);
}
