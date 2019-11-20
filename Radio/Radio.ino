#include "radio.h"
#include "sx126x-board.h"
#include "sx126x.h"
#include <SPI.h>

#define Transmit                               // Comment out to act as sender COM16
//#define Receive                               // Comment out to act as receiver COM13
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

#define CALIBRATE 37                                // Calibrate button

// LoRa setup - see board.h and ensure pins are updated there as well
#define LORA_NSS    15
#define LORA_SCK    14    //NEW BOARD
//#define LORA_SCK    4
#define LORA_MOSI   13
#define LORA_MISO   12
//#define LORA_RST    
#define LORA_BUSY   19
#define LORA_DIO1   4     // New board
//#define LORA_DIO1   14
#define LORA_FREQ   915E6           // 915Mhz
#define LORA_ADDRESS  0xA6          // Rigger Assist newtwork address

struct transfer {
  float pitch;
  float roll;
  float yaw;
} myData;

// write struct as binary Serial.write((const uint8_t *)&myData,sizeof(struct transfer));
// Read struct (could also use a fixed size so no need to send size)
//char *ptr;
//struct myStruct myStructVariable;
//int count;
//int size;
//size = sizeof(struct myStruct);
//ptr = (char *)&myStructVariable;
//for(count=0; count<size; count++)
//{
//  *(ptr+count) = Serial.read();
//}

SPIClass * hspi = NULL;     // Default for Arduino if SPI used
static RadioEvents_t RadioEvents;
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnTxDone(void);
void OnTxTimeout(void);
uint8_t packet[] = {'h','e','l','l','o'};
RadioStatus_t status;
uint16_t Interrupt;
//RadioError_t Rerror;

void setup() {
  // put your setup code here, to run once:
  hspi = new SPIClass(HSPI);
   //hspi->begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS); //SCLK, MISO, MOSI, SS
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  SX126xIoInit();

  Serial.begin(115200);
  Serial.println("Begin");

  

  pinMode(CALIBRATE, INPUT);                      // ON when LOW
  pinMode(26, OUTPUT);        // LED CATHODE
  pinMode(27, OUTPUT);        // LED ANODE
  
  SX126xCalibrateImage(RF_FREQUENCY);
  CalibrationParams_t calibParam;
  calibParam.Value = 0x7F;
  SX126xCalibrate(calibParam);

  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init( &RadioEvents );

  Serial.println("Init events");
  Radio.SetChannel( RF_FREQUENCY );
  Serial.println("Set rf frequency");
  Radio.SetPublicNetwork(true);

  //TODO: Add network address
  //TODO: Consider using implicit header mode as it will save transmission time (header / setup is manual on both ends)
#ifdef Receive
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     LORA_PAYLOAD_LENGTH, true, 0, 0, LORA_IQ_INVERSION_ON, true );
  Serial.println("Set Rx config");
#else
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );    // Was 3000000
  Serial.println("Set Tx config");
#endif
#ifdef Receive
  Radio.Rx( 0 ); // Continuous Rx
  Serial.println("RX");
#else
  Radio.Send(packet,5);
  Serial.println("Sent");
#endif
}

void loop() {
  //if (digitalRead(CALIBRATE)) {
    digitalWrite(27, HIGH);
  //} else {
    //digitalWrite(26, LOW);    
  //}
  digitalWrite(26, LOW);
  delay(1000);
  Radio.Send(packet,5);
  Serial.println("xfin");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
  //Serial.println("rx finish");
  //Serial.print("packet : ");
  for (int i = 0; i < size; ++i)
    Serial.print(payload[i]);
    Serial.println();
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" SNR: ");
    Serial.println(snr);
  //status = SX126xGetStatus();
  //Serial.print("Status: ");
  //Serial.println(status.Fields.ChipMode);
}

void OnTxDone(void)
{
  //Serial.println("fin");
  //Radio.Send(packet,5);
}

void OnTxTimeout()
{
  Serial.println("tx timeout");
}
