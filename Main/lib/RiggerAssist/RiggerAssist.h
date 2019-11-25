/*Header file for rigger assist system which
Includes the hook module used for collecting
data via an IMU and the datahead for displaying the
data to the user. Both devices communicate via
LORA. */

#include "radio.h"
#include "sx126x-board.h"
#include "sx126x.h"

#define VERSION         1.0               // Software version
#define PACKET_SIZE     4
#define RA_ID           67                // Rigger Assist unique ID

//#define HOOKMODULE                        // Select hook module or datahead for compilation
//#define DEBUG_IMU_RAW                //Produce  different output sets depending on what you need
//#define DEBUG_IMU_ANGLES
#define DEBUG_RADIO



//**********************LORA DEFINITIONS*************************

#define USE_MODEM_LORA
#define RF_FREQUENCY                                921500000 // Hz (don't use 915Mhz as other non hoppers might as well)
#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       11        // [SF5..SF12]
#define LORA_CODINGRATE                             3         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_FIX_LENGTH_PAYLOAD_ON                  true
#define LORA_PAYLOAD_LENGTH                         4         // Size of data
#define LORA_IQ_INVERSION_ON                        false
#define TX_OUTPUT_POWER                             22        // Max 22

// LoRa setup - see board.h and ensure pins are updated there as well
#define LORA_NSS    15
#define LORA_SCK    14    //NEW BOARD
// #define LORA_SCK    4       // Old board
#define LORA_MOSI   13
#define LORA_MISO   12
//#define LORA_RST    
#define LORA_BUSY   19
#define LORA_DIO1   4     // New board
// #define LORA_DIO1   14      // Old board
//#define LORA_FREQ   915E6           // 915Mhz
#define LORA_ADDRESS  0xA6          // Rigger Assist network address

/* ***********************OTHER STUFF************************************/
#define CALIBRATE           37                                // Calibrate button
#define BATT_VOLT           39                      // Battery voltage * 0.728
#define BATT_CHG            9                          // Battery charging GPIO
#define LED_N               27                        // Blue LED -ve
#define LED_P               26                        // Blue LED +ve

//These are the capacitove discharge times for high and low light levels, may need tweaking.
#define HIGH_LIGHT_LEVEL    20000
#define LOW_LIGHT_LEVEL     40000

//Used in control byte of radio packet
#define NOT_CAL             7
#define CAL_IN_PROG         6
#define BATTSTATUS          5
#define BINDING             4
#define CAL_COMPLETE        3
#define RTK_STATE          2

// uint8_t dischargeCurve(uint8_t pec)
// {
// static const uint8_t lookup[20] =
// {
// 4.25, 3.93,	3.81, 3.77,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75,
// 3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75,
// 3.75, 3.75,	3.75, 3.75,	3.75, 3.75,	3.75, 3.75, 3.74, 3.73,	3.71, 3.68,	3.62, 3.49,	3.24,
// };
// pec = lookup[pec];
// return pec;
// }



struct rawOrient {
  float yaw;
  float roll;
  float pitch;
  float compPitch;
  float compRoll;
  float avgAngle;
};
extern struct rawOrient orient;

void CalibrateIMU_ISR();
void BattCharging_ISR();
void BattChargingFin_ISR();
void onTimer();
void Task1( void * parameter );
void CheckCalButtonState(uint8_t reading);

float ReadBattVoltage();
void updateBattLevelIndicator(float battVoltage);
void UpdateDisplay();
void SetSigStrengthIndicator();
void UpdateDispOnRxTimeout();
void AmbientLightAdjust();

void CalibrateIMU();
void updateOrientation(bool calib);
void SendDataPacket();
void SmoothData(); 

void InitGPIO();
void CalibrateSX1272();
void InitRadioEvents();
void flashLED(uint8_t states);
void Send();
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxDone(void);
void OnTxTimeout(void);
uint8_t PackSendBits(float inputData);
void AssignTxBufferContents();
float rxDataToFloat(uint8_t rxData);
uint8_t chksum8(uint8_t input, size_t len);
void OutputIMUData(bool outputMode);
