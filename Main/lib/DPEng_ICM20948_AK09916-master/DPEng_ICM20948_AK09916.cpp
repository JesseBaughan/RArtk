/*!
 * @file DPEng_ICM20948_AK09916.cpp
 *
 * @mainpage DPEng ICM20948 accel/gyro/mag sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for DPEng's ICM20948 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * DPEng ICM20948 breakout which can be purchased from eBay here:
 * https://www.ebay.co.uk/itm/323724746939
 * or Amazon here:
 * https://www.amazon.co.uk/DP-Eng-ICM-20948-Breakout-obsolete/dp/B07PDTKK3Y
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout. SPI is also possible with <2 MBIT/s speeds.
 *
 * DPEng invests time and resources providing this open source code,
 * please support DPEng and open-source hardware by purchasing
 * products from DPEng!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_Sensor">
 * Adafruit_Sensor</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by David Pattison for DPEng.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>
#include <limits.h>

#include "DPEng_ICM20948_AK09916.h"

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in the Arduino wire library
			for any address
	@param address The i2c address of the device
    @param reg The register address to write to
    @param value The value to write to the specified register
*/
/**************************************************************************/
void DPEng_ICM20948::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in the Arduino wire library
    @param reg The register address to write to
    @param value The value to write to the specified register
*/
/**************************************************************************/
void DPEng_ICM20948::write8(byte reg, byte value)
{
  Wire.beginTransmission(ICM20948_ACCELGYRO_ADDRESS);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in the Arduino wire library
			for any address
    @param address The i2c address of the device
	@param reg The register address to read from
*/
/**************************************************************************/
byte DPEng_ICM20948::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission((byte)address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((byte)address, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif

  return value;
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in the Arduino wire library
    @param reg The register address to read from
*/
/**************************************************************************/
byte DPEng_ICM20948::read8(byte reg)
{
  byte value;

  Wire.beginTransmission((byte)ICM20948_ACCELGYRO_ADDRESS);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((byte)ICM20948_ACCELGYRO_ADDRESS, (byte)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif

  return value;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

 /**************************************************************************/
 /*!
     @brief  Instantiates a new DPEng_ICM20948_AK09916 class, including assigning
             a unique ID to the accel and gyroscope for logging purposes.

     @param accelSensorID The unique ID to associate with the accelerometer.
     @param gyroSensorID The unique ID to associate with the gyroscope.
	 @param magSensorID The unique ID to associate with the magnetometer.
 */
 /**************************************************************************/
DPEng_ICM20948::DPEng_ICM20948(int32_t accelSensorID, int32_t gyroSensorID, int32_t magSensorID)
{
  _accelSensorID = accelSensorID;
  _gyroSensorID = gyroSensorID;
  _magSensorID = magSensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

 /**************************************************************************/
 /*!
     @brief  Initializes the hardware, including setting the accelerometer
             range based on icm20948AccelRange_t and gyroscope range based
			 on icm20948GyroRange_t

     @param  rngAccel
             The range to set for the accelerometer, based on icm20948AccelRange_t
	 @param  rngGyro
             The range to set for the gyroscope, based on icm20948GyroRange_t
	 @param  lowpassAccel
             The lowpass configuration of the accelerometer, based on icm20948AccelLowpass_t

     @return True if the device was successfully initialized, otherwise false.
 */
 /**************************************************************************/
bool DPEng_ICM20948::begin(icm20948AccelRange_t rngAccel, icm20948GyroRange_t rngGyro, icm20948AccelLowpass_t lowpassAccel)
{
  /* Enable I2C */
  Wire.begin(SDA, SCL);

  /* Set the range the an appropriate value */
  _rangeAccel = rngAccel;
  _rangeGyro = rngGyro;

  /* Clear the raw sensor data */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;
  gyro_raw.x = 0;
  gyro_raw.y = 0;
  gyro_raw.z = 0;
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  // delay(100);
  // write8(PWR_MGMT_1, 0x80);           // Software reset
  // delay(150);

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = read8(WHO_AM_I_ICM20948);
  if (id != ICM20948_ACCELGYRO_ID)
  {
    Serial.print("Incorrect Accel/Gyro WHOAMI, should be 0xEA returned ");
    Serial.println(id, HEX);
    return false;
  }
  Serial.println("Accel/Gyro OK");

  /* Set to standby mode (required to make changes to this register) */
  write8(PWR_MGMT_1, 0x01);

  delay(100);

  write8(INT_PIN_CFG, 0x22);

  // Switch to user bank 2
  write8(REG_BANK_SEL, 0x20);

  uint8_t reg1 = read8(ACCEL_CONFIG);

  // reg1 = reg1 & ~0xE0; // Clear self-test bits [7:5]
  reg1 = reg1 & ~0x06;  // Clear AFS bits [2:1]

  /* Configure the accelerometer */
  /* Set ACCEL_CONFIG (0x14)
   ====================================================================
   BIT  Symbol    		Description                                   Default
   ---  ------    		--------------------------------------------- -------
   7:6  -	 		Reserved                                   		  -
   5:3  ACCEL_DLFPFCFG[2:0] 	Accelerometer low pass filter                     000
		  		000 = 246 Hz / 1209 Hz if FCHOICE is 0
                  		001 = 246 Hz
                  		010 = 111.4 Hz
                  		011 = 50.4 Hz
                  		100 = 23.9 Hz
                  		101 = 11.5 Hz
                  		110 = 5.7 Hz
                  		111 = 473 Hz
   2:1  ACCEL_FS_SEL[1:0]   	Accelerometer Full Scale Select                    00
     0  ACCEL_FCHOICE           0 :Bypass accel DLPF, 1 :Enable accel DLPF          1
  */
  switch (_rangeAccel) {
      case (ICM20948_ACCELRANGE_2G):
        reg1 = reg1 | ICM20948_ACCELRANGE_2G; // Set full scale range for the accelerometer
      break;
      case (ICM20948_ACCELRANGE_4G):
        reg1 = reg1 | ICM20948_ACCELRANGE_4G; // Set full scale range for the accelerometer
      break;
      case (ICM20948_ACCELRANGE_8G):
        reg1 = reg1 | ICM20948_ACCELRANGE_8G; // Set full scale range for the accelerometer
      break;
	  case (ICM20948_ACCELRANGE_16G):
        reg1 = reg1 | ICM20948_ACCELRANGE_16G; // Set full scale range for the accelerometer
      break;
  }
  reg1 = reg1 | 0x01; // Set enable accel DLPF for the accelerometer
  reg1 = reg1 | lowpassAccel; // and set DLFPFCFG to 50.4 hz

  // Write new ACCEL_CONFIG register value
  write8(ACCEL_CONFIG, reg1);

  write8(ACCEL_SMPLRT_DIV_2, 0x14);

  /* Set GYRO_CONFIG_1 to selected DPS Range - Default value 0x01 (250 dps)
  =====================================================================
  BIT  Symbol     		Description                                   Default
  7:6  -          		Reserved				           00
  5:3  GYRO_DLPFCFG[2:0]	Gyro low pass filter configuration                000
  2:1  GYRO_FS_SEL[1:0]        	Gyro Full Scale Select:				   00
		 		00 = +-250 dps
                  		01 = +-500 dps
                  		10 = +-1000 dps
                  		11 = +-2000 dps
    0  GYRO_FCHOICE    		0 :Bypass gyro DLPF, 1 :Enable gyro DLPF            1

  */

  /* Instantiate ctrlReg0 */
  uint8_t ctrlReg0 = 0x01;
  /* Configure the gyroscope */
  switch(_rangeGyro)
  {
    case GYRO_RANGE_250DPS:
      ctrlReg0 = 0x01;
      break;
    case GYRO_RANGE_500DPS:
      ctrlReg0 = 0x03;
      break;
    case GYRO_RANGE_1000DPS:
      ctrlReg0 = 0x05;
      break;
    case GYRO_RANGE_2000DPS:
      ctrlReg0 = 0x07;
      break;
  }

  write8(GYRO_CONFIG_1, ctrlReg0); // Set sensitivity
  delay(100); // 60 ms + 1/ODR
  write8(GYRO_SMPLRT_DIV, 0x0A); // Set gyro sample rate divider

  // Switch to user bank 0
  write8(REG_BANK_SEL, 0x00);

   /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  id = read8(ICM20948_MAG_ADDRESS, WHO_AM_I_AK09916);
  //while(1) { Serial.print("TEST ");Serial.println(id); }
  // Serial.print("WHO AM I? 0x"); Serial.println(id, HEX);
  if (id != ICM20948_MAG_ID)
  {
    Serial.print("Incorrect Mag WHOAMI, should be 0xEA returned ");
    Serial.println(id, HEX);
    return false;
  }

  Serial.println("Mag OK");

  /* Configure the magnetometer */
  /* Hybrid Mode, Over Sampling Rate = 16 */
<<<<<<< HEAD
  write8(ICM20948_MAG_ADDRESS, AK09916_CNTL, 0x08);
=======
  write8(ICM20948_MAG_ADDRESS, AK09916_CNTL, 0x08); //running in continouos mode 4 = 100Hz
>>>>>>> develop

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor events.

            This function reads from both the accelerometer, gyroscope and
            magnetometer in one call, and is a deviation from the standard
            Adafruit_Sensor API, but is provided as a convenience since most
            AHRS algorithms require sensor samples to be as close in time as
            possible.

    @param    accelEvent
              A reference to the sensors_event_t instances where the
              accelerometer data should be written.
    @param    gyroEvent
              A reference to the sensors_event_t instances where the
              gyroscope data should be written.
	@param	  magEvent
			  A reference to the sensors_event_t instances where the
			  magnetometer data should be written.

    @return True if the event read was successful, otherwise false.
*/
/**************************************************************************/
bool DPEng_ICM20948::getEvent(sensors_event_t* accelEvent, sensors_event_t* gyroEvent, sensors_event_t* magEvent)
{
  /* Clear the event */
  memset(accelEvent, 0, sizeof(sensors_event_t));
  memset(gyroEvent, 0, sizeof(sensors_event_t));
  memset(magEvent, 0, sizeof(sensors_event_t));

  /* Clear the raw data placeholder */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;
  gyro_raw.x = 0;
  gyro_raw.y = 0;
  gyro_raw.z = 0;
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  /* Set the static metadata */
  accelEvent->version   = sizeof(sensors_event_t);
  accelEvent->sensor_id = _accelSensorID;
  accelEvent->type      = SENSOR_TYPE_ACCELEROMETER;

  gyroEvent->version   = sizeof(sensors_event_t);
  gyroEvent->sensor_id = _gyroSensorID;
  gyroEvent->type      = SENSOR_TYPE_GYROSCOPE;

  magEvent->version   = sizeof(sensors_event_t);
  magEvent->sensor_id = _magSensorID;
  magEvent->type      = SENSOR_TYPE_MAGNETIC_FIELD;

  /* Read 12 bytes from the icm20948 sensor */
  Wire.beginTransmission((byte)ICM20948_ACCELGYRO_ADDRESS);
  #if ARDUINO >= 100
    Wire.write(ACCEL_XOUT_H);
  #else
    Wire.send(ACCEL_XOUT_H);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)ICM20948_ACCELGYRO_ADDRESS, (byte)12);

  /* ToDo: Check status first! */
  #if ARDUINO >= 100
    uint8_t axhi = Wire.read();
    uint8_t axlo = Wire.read();
    uint8_t ayhi = Wire.read();
    uint8_t aylo = Wire.read();
    uint8_t azhi = Wire.read();
    uint8_t azlo = Wire.read();
    uint8_t gxhi = Wire.read();
    uint8_t gxlo = Wire.read();
    uint8_t gyhi = Wire.read();
    uint8_t gylo = Wire.read();
    uint8_t gzhi = Wire.read();
    uint8_t gzlo = Wire.read();
  #else
    uint8_t axhi = Wire.receive();
    uint8_t axlo = Wire.receive();
    uint8_t ayhi = Wire.receive();
    uint8_t aylo = Wire.receive();
    uint8_t azhi = Wire.receive();
    uint8_t azlo = Wire.receive();
    uint8_t gxhi = Wire.receive();
    uint8_t gxlo = Wire.receive();
    uint8_t gyhi = Wire.receive();
    uint8_t gylo = Wire.receive();
    uint8_t gzhi = Wire.receive();
    uint8_t gzlo = Wire.receive();
  #endif

  /* Read 7 bytes from the AK09916 sensor */
  Wire.beginTransmission((byte)ICM20948_MAG_ADDRESS);
  #if ARDUINO >= 100
    Wire.write(AK09916_XOUT_L);
  #else
    Wire.send(AK09916_XOUT_L);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)ICM20948_MAG_ADDRESS, (byte)8);

  #if ARDUINO >= 100
    uint8_t mxlo = Wire.read();
    uint8_t mxhi = Wire.read();
    uint8_t mylo = Wire.read();
    uint8_t myhi = Wire.read();
    uint8_t mzlo = Wire.read();
    uint8_t mzhi = Wire.read();
	  uint8_t reset = Wire.read();
	  uint8_t status = Wire.read();
  #else
    uint8_t mxlo = Wire.receive();
    uint8_t mxhi = Wire.receive();
    uint8_t mylo = Wire.receive();
    uint8_t myhi = Wire.receive();
    uint8_t mzlo = Wire.receive();
    uint8_t mzhi = Wire.receive();
	uint8_t reset = Wire.receive();
	uint8_t status = Wire.receive();
  #endif

  /* Set the timestamps */
  accelEvent->timestamp = millis();
  gyroEvent->timestamp = accelEvent->timestamp;
  magEvent->timestamp = accelEvent->timestamp;

  /* Shift values to create properly formed integers */
  /* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
  accelEvent->acceleration.x = (int16_t)((axhi << 8) | axlo);
  accelEvent->acceleration.y = (int16_t)((ayhi << 8) | aylo);
  accelEvent->acceleration.z = (int16_t)((azhi << 8) | azlo);
  gyroEvent->gyro.x = (int16_t)((gxhi << 8) | gxlo);
  gyroEvent->gyro.y = (int16_t)((gyhi << 8) | gylo);
  gyroEvent->gyro.z = (int16_t)((gzhi << 8) | gzlo);
  magEvent->magnetic.x = (int16_t)((mxhi << 8) | mxlo);
  magEvent->magnetic.y = (int16_t)((myhi << 8) | mylo);
  magEvent->magnetic.z = (int16_t)((mzhi << 8) | mzlo);

  /* Assign raw values in case someone needs them */
  accel_raw.x = accelEvent->acceleration.x;
  accel_raw.y = accelEvent->acceleration.y;
  accel_raw.z = accelEvent->acceleration.z;
  gyro_raw.x = gyroEvent->gyro.x;
  gyro_raw.y = gyroEvent->gyro.y;
  gyro_raw.z = gyroEvent->gyro.z;
  mag_raw.x = magEvent->magnetic.x;
  mag_raw.y = magEvent->magnetic.y;
  mag_raw.z = magEvent->magnetic.z;

  /* Convert accel values to m/s^2 */
  switch (_rangeAccel) {
      case (ICM20948_ACCELRANGE_2G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ICM20948_ACCELRANGE_4G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ICM20948_ACCELRANGE_8G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
	  case (ICM20948_ACCELRANGE_16G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
      break;
  }

  /* Compensate values depending on the resolution */
  switch(_rangeGyro)
  {
    case GYRO_RANGE_250DPS:
      gyroEvent->gyro.x *= GYRO_SENSITIVITY_250DPS;
      gyroEvent->gyro.y *= GYRO_SENSITIVITY_250DPS;
      gyroEvent->gyro.z *= GYRO_SENSITIVITY_250DPS;
      break;
    case GYRO_RANGE_500DPS:
      gyroEvent->gyro.x *= GYRO_SENSITIVITY_500DPS;
      gyroEvent->gyro.y *= GYRO_SENSITIVITY_500DPS;
      gyroEvent->gyro.z *= GYRO_SENSITIVITY_500DPS;
      break;
    case GYRO_RANGE_1000DPS:
      gyroEvent->gyro.x *= GYRO_SENSITIVITY_1000DPS;
      gyroEvent->gyro.y *= GYRO_SENSITIVITY_1000DPS;
      gyroEvent->gyro.z *= GYRO_SENSITIVITY_1000DPS;
      break;
    case GYRO_RANGE_2000DPS:
      gyroEvent->gyro.x *= GYRO_SENSITIVITY_2000DPS;
      gyroEvent->gyro.y *= GYRO_SENSITIVITY_2000DPS;
      gyroEvent->gyro.z *= GYRO_SENSITIVITY_2000DPS;
      break;
  }

  /* Convert mag values to uTesla */
  magEvent->magnetic.x *= MAG_UT_LSB;
  magEvent->magnetic.y *= MAG_UT_LSB;
  magEvent->magnetic.z *= MAG_UT_LSB;

  // Serial.print(accel_raw.x);
  // Serial.print("\t");
  // Serial.print(accel_raw.y);
  // Serial.print("\t");
  // Serial.print(accel_raw.z);
  // Serial.print("\t");
  // Serial.print(gyro_raw.x);
  // Serial.print("\t");
  // Serial.print(gyro_raw.y);
  // Serial.print("\t");
  // Serial.print(gyro_raw.z);
  // Serial.print("\t");
  // Serial.print(mag_raw.x);
  // Serial.print("\t");
  // Serial.print(mag_raw.y);
  // Serial.print("\t");
  // Serial.println(mag_raw.z);
  //Serial.print("\t");

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most accelerometer recent sensor events.

            This function reads only the accelerometer. The function has been
            reduced in complexity to speed up the execution.

    @param    accelEvent
              A reference to the sensors_event_t instances where the
              accelerometer data should be written.

    @return True if the event read was successful, otherwise false.
*/
/**************************************************************************/
bool DPEng_ICM20948::getEventAcc(sensors_event_t* accelEvent)
{
  /* Clear the event */
  memset(accelEvent, 0, sizeof(sensors_event_t));

  /* Clear the raw data placeholder */
  accel_raw.x = 0;
  accel_raw.y = 0;
  accel_raw.z = 0;

  /* Set the static metadata */
  accelEvent->version   = sizeof(sensors_event_t);
  accelEvent->sensor_id = _accelSensorID;
  accelEvent->type      = SENSOR_TYPE_ACCELEROMETER;

  /* Read 6 bytes from the icm20948 sensor */
  Wire.beginTransmission((byte)ICM20948_ACCELGYRO_ADDRESS);
  #if ARDUINO >= 100
    Wire.write(ACCEL_XOUT_H);
  #else
    Wire.send(ACCEL_XOUT_H);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)ICM20948_ACCELGYRO_ADDRESS, (byte)6);

  /* ToDo: Check status first! */
  #if ARDUINO >= 100
    uint8_t axhi = Wire.read();
    uint8_t axlo = Wire.read();
    uint8_t ayhi = Wire.read();
    uint8_t aylo = Wire.read();
    uint8_t azhi = Wire.read();
    uint8_t azlo = Wire.read();
  #else
    uint8_t axhi = Wire.receive();
    uint8_t axlo = Wire.receive();
    uint8_t ayhi = Wire.receive();
    uint8_t aylo = Wire.receive();
    uint8_t azhi = Wire.receive();
    uint8_t azlo = Wire.receive();
  #endif

  /* Set the timestamps */
  accelEvent->timestamp = millis();

  /* Shift values to create properly formed integers */
  /* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
  accelEvent->acceleration.x = (int16_t)((axhi << 8) | axlo);
  accelEvent->acceleration.y = (int16_t)((ayhi << 8) | aylo);
  accelEvent->acceleration.z = (int16_t)((azhi << 8) | azlo);

  /* Assign raw values in case someone needs them */
  accel_raw.x = accelEvent->acceleration.x;
  accel_raw.y = accelEvent->acceleration.y;
  accel_raw.z = accelEvent->acceleration.z;

  /* Convert accel values to m/s^2 */
  switch (_rangeAccel) {
      case (ICM20948_ACCELRANGE_2G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ICM20948_ACCELRANGE_4G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ICM20948_ACCELRANGE_8G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
	  case (ICM20948_ACCELRANGE_16G):
          accelEvent->acceleration.x *= ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.y *= ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
          accelEvent->acceleration.z *= ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
      break;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor events.

            This function reads only the magnetometer. The function has been
            reduced in complexity to speed up the execution.

	@param	  magEvent
			  A reference to the sensors_event_t instances where the
			  magnetometer data should be written.

    @return True if the event read was successful, otherwise false.
*/
/**************************************************************************/
bool DPEng_ICM20948::getEventMag(sensors_event_t* magEvent)
{
  /* Clear the event */
  memset(magEvent, 0, sizeof(sensors_event_t));

  /* Clear the raw data placeholder */
  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  /* Set the static metadata */
  magEvent->version   = sizeof(sensors_event_t);
  magEvent->sensor_id = _magSensorID;
  magEvent->type      = SENSOR_TYPE_MAGNETIC_FIELD;

  /* Read 7 bytes from the AK09916 sensor */
  Wire.beginTransmission((byte)ICM20948_MAG_ADDRESS);
  #if ARDUINO >= 100
    Wire.write(AK09916_XOUT_L);
  #else
    Wire.send(AK09916_XOUT_L);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)ICM20948_MAG_ADDRESS, (byte)8);

  #if ARDUINO >= 100
    uint8_t mxlo = Wire.read();
    uint8_t mxhi = Wire.read();
    uint8_t mylo = Wire.read();
    uint8_t myhi = Wire.read();
    uint8_t mzlo = Wire.read();
    uint8_t mzhi = Wire.read();
	  uint8_t reset = Wire.read();
	  uint8_t status = Wire.read();
  #else
    uint8_t mxlo = Wire.receive();
    uint8_t mxhi = Wire.receive();
    uint8_t mylo = Wire.receive();
    uint8_t myhi = Wire.receive();
    uint8_t mzlo = Wire.receive();
    uint8_t mzhi = Wire.receive();
	uint8_t reset = Wire.receive();
	uint8_t status = Wire.receive();
  #endif

  /* Set the timestamps */
  magEvent->timestamp = millis();

  /* Shift values to create properly formed integers */
  /* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
  magEvent->magnetic.x = (int16_t)((mxhi << 8) | mxlo);
  magEvent->magnetic.y = (int16_t)((myhi << 8) | mylo);
  magEvent->magnetic.z = (int16_t)((mzhi << 8) | mzlo);

  /* Assign raw values in case someone needs them */
  mag_raw.x = magEvent->magnetic.x;
  mag_raw.y = magEvent->magnetic.y;
  mag_raw.z = magEvent->magnetic.z;

  /* Convert mag values to uTesla */
  magEvent->magnetic.x *= MAG_UT_LSB;
  magEvent->magnetic.y *= MAG_UT_LSB;
  magEvent->magnetic.z *= MAG_UT_LSB;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets sensor_t data for both the accel, gyro and mag in one operation.

    @param    accelSensor
              A reference to the sensor_t instances where the
              accelerometer sensor info should be written.
    @param    gyroSensor
              A reference to the sensor_t instances where the
              gyroscope sensor info should be written.
	@param	  magSensor
			  A reference to the sensor_t instances where the
			  magnetometer sensor info should be written.
*/
/**************************************************************************/
void  DPEng_ICM20948::getSensor(sensor_t* accelSensor, sensor_t* gyroSensor, sensor_t* magSensor)
{
  /* Clear the sensor_t object */
  memset(accelSensor, 0, sizeof(sensor_t));
  memset(gyroSensor, 0, sizeof(sensor_t));
  memset(magSensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (accelSensor->name, "ICM20948", sizeof(accelSensor->name) - 1);
  accelSensor->name[sizeof(accelSensor->name) - 1] = 0;
  accelSensor->version     = 1;
  accelSensor->sensor_id   = _accelSensorID;
  accelSensor->type        = SENSOR_TYPE_ACCELEROMETER;
  accelSensor->min_delay   = 0.01F; // 100Hz
  switch (_rangeAccel) {
      case (ICM20948_ACCELRANGE_2G):
          accelSensor->max_value   = 2.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -1.999F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ICM20948_ACCELRANGE_4G):
          accelSensor->max_value   = 4.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -3.998F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_4G * SENSORS_GRAVITY_STANDARD;
      break;
      case (ICM20948_ACCELRANGE_8G):
          accelSensor->max_value   = 8.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -7.996F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_8G * SENSORS_GRAVITY_STANDARD;
      break;
	  case (ICM20948_ACCELRANGE_16G):
          accelSensor->max_value   = 16.0F * SENSORS_GRAVITY_STANDARD;
          accelSensor->min_value   = -15.996F * SENSORS_GRAVITY_STANDARD;
          accelSensor->resolution  = ACCEL_MG_LSB_16G * SENSORS_GRAVITY_STANDARD;
      break;
  }

  strncpy (gyroSensor->name, "ICM20948", sizeof(gyroSensor->name) - 1);
  gyroSensor->name[sizeof(gyroSensor->name) - 1] = 0;
  gyroSensor->version     = 1;
  gyroSensor->sensor_id   = _gyroSensorID;
  gyroSensor->type        = SENSOR_TYPE_GYROSCOPE;
  gyroSensor->min_delay   = 0;
  gyroSensor->max_value   = (float)this->_rangeGyro;
  gyroSensor->min_value   = (this->_rangeGyro * -1.0);
  gyroSensor->resolution  = 0.0F; // TBD

  strncpy (magSensor->name, "AK09916", sizeof(magSensor->name) - 1);
  magSensor->name[sizeof(magSensor->name)- 1] = 0;
  magSensor->version     = 1;
  magSensor->sensor_id   = _magSensorID;
  magSensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  magSensor->min_delay   = 0.01F;
  magSensor->max_value   = 1200.0F;
  magSensor->min_value   = -1200.0F;
  magSensor->resolution  = 0.1F; // TBD
}

/**************************************************************************/
/*!
    @brief    Reads a single sensor event from the accelerometer.

    @attention

    This function exists to keep Adafruit_Sensor happy since we
    need a single sensor in the canonical .getEvent call. The
    non-standard .getEvent call with two parameters should
    generally be used with this sensor.

    @param    accelEvent
              A reference to the sensors_event_t instances where the
              accelerometer data should be written.

    @return True if the event read was successful, otherwise false.
*/
/**************************************************************************/
bool DPEng_ICM20948::getEvent(sensors_event_t* accelEvent)
{
    sensors_event_t accel;

    return getEvent(accelEvent, &accel, &accel);
}

/**************************************************************************/
/*!
    @brief   Gets sensor details about the accelerometer.

    @attention

    This function exists to keep Adafruit_Sensor happy since we
    need a single sensor in the canonical .getSensor call. The
    non-standard .getSensor call with two parameters should
    generally be used with this sensor.

    @param   accelSensor
             A reference to the sensor_t instances where the
             accelerometer sensor info should be written.
*/
/**************************************************************************/
void  DPEng_ICM20948::getSensor(sensor_t* accelSensor)
{
    sensor_t accel;

    return getSensor(accelSensor, &accel, &accel);
}


/**************************************************************************/
/*!
    @brief  Puts device into/out of standby mode

    @param standby Set this to a non-zero value to enter standy mode.
*/
/**************************************************************************/
void DPEng_ICM20948::standby(boolean standby)
{
  uint8_t reg1 = read8(PWR_MGMT_1);
  if (standby) {
    reg1 &= ~(0x01);
  } else {
    reg1 |= (0x01);
  }
  write8(PWR_MGMT_1, reg1);

  if (! standby) {
    delay(100);
  }
}

<<<<<<< HEAD
void DPEng_ICM20948::calibrateIMU(float * gyroBias, float * accelBias) {

  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
=======
void DPEng_ICM20948::getAccelGyroOffsets(float * gyroBias, float * accelBias) {
>>>>>>> develop
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec

<<<<<<< HEAD
  // // reset device
  // // Write a one to bit 7 reset bit; toggle reset device
  // writeByte(ICM20948_ACCELGYRO_ADDRESS, PWR_MGMT_1, READ_FLAG);
  // delay(200);

  // // get stable time source; Auto select clock source to be PLL gyroscope
  // // reference if ready else use the internal oscillator, bits 2:0 = 001
  // writeByte(ICM20948_ACCELGYRO_ADDRESS, PWR_MGMT_1, 0x01);
  // delay(200);
  // // How many sets of full gyro and accelerometer data for averaging

  packet_count = 150;
=======
  packet_count = 1000;
>>>>>>> develop

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};

    /* Read 12 bytes from the icm20948 sensor */
    Wire.beginTransmission((byte)ICM20948_ACCELGYRO_ADDRESS);
    #if ARDUINO >= 100
      Wire.write(ACCEL_XOUT_H);
    #else
      Wire.send(ACCEL_XOUT_H);
    #endif
    Wire.endTransmission();
    Wire.requestFrom((byte)ICM20948_ACCELGYRO_ADDRESS, (byte)12);

    uint8_t axhi = Wire.read();
    uint8_t axlo = Wire.read();
    uint8_t ayhi = Wire.read();
    uint8_t aylo = Wire.read();
    uint8_t azhi = Wire.read();
    uint8_t azlo = Wire.read();
    uint8_t gxhi = Wire.read();
    uint8_t gxlo = Wire.read();
    uint8_t gyhi = Wire.read();
    uint8_t gylo = Wire.read();
    uint8_t gzhi = Wire.read();
    uint8_t gzlo = Wire.read();
 
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t)((axhi << 8) | axlo);
    accel_temp[1] = (int16_t)((ayhi << 8) | aylo);
    accel_temp[2] = (int16_t)((azhi << 8) | azlo);
    gyro_temp[0]  = (int16_t)((gxhi << 8) | gxlo);
    gyro_temp[1]  = (int16_t)((gyhi << 8) | gylo);
    gyro_temp[2]  = (int16_t)((gzhi << 8) | gzlo);

<<<<<<< HEAD
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
=======
>>>>>>> develop
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

<<<<<<< HEAD
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
=======
>>>>>>> develop
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

<<<<<<< HEAD
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }
  // Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) -gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) -gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) -gyro_bias[2]/(float) gyrosensitivity;

  // Output scaled accelerometer biases for display in the main program
  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;

}

=======
  if (accel_bias[1] > 0L)
  {
    accel_bias[1] -= (int32_t) accelsensitivity; // Remove gravity from the y-axis accelerometer bias calculation as this is our orientation when on hook
  }
  else
  {
    accel_bias[1] += (int32_t) accelsensitivity;
  }
  // Output scaled gyro and accel biases  for display in the main program
  gyroBias[0] = (float) -gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) -gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) -gyro_bias[2]/(float) gyrosensitivity;
  accelBias[0] = (float)accel_bias[0] * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
  accelBias[1] = (float)accel_bias[1] * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
  accelBias[2] = (float)accel_bias[2] * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STANDARD;
}


void DPEng_ICM20948::getMagBaseValues(float * destMag) 
 {
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_temp[3] = {0, 0, 0};

  // shoot for ~fifteen seconds of mag data
  sample_count = 500;  // at 100 Hz ODR, new mag data is available every 10 ms

  Serial.println("Getting mag offsets!!!!!!");

  for(ii = 0; ii < sample_count; ii++) {
  /* Read 7 bytes from the AK09916 sensor */
    Wire.beginTransmission((byte)ICM20948_MAG_ADDRESS);
    #if ARDUINO >= 100
      Wire.write(AK09916_XOUT_L);
    #else
      Wire.send(AK09916_XOUT_L);
    #endif
    Wire.endTransmission();
    Wire.requestFrom((byte)ICM20948_MAG_ADDRESS, (byte)6);

    #if ARDUINO >= 100
      uint8_t mxlo = Wire.read();
      uint8_t mxhi = Wire.read();
      uint8_t mylo = Wire.read();
      uint8_t myhi = Wire.read();
      uint8_t mzlo = Wire.read();
      uint8_t mzhi = Wire.read();
    #endif

    mag_temp[0] = (int16_t)((mxhi << 8) | mxlo);
    mag_temp[1] = (int16_t)((myhi << 8) | mylo);
    mag_temp[2] = (int16_t)((mzhi << 8) | mzlo);
    
    mag_bias[0] += (int32_t) mag_temp[0];
    mag_bias[1] += (int32_t) mag_temp[1];
    mag_bias[2] += (int32_t) mag_temp[2];

    delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
  }

    mag_bias[0] /= (int32_t) sample_count;
    mag_bias[1] /= (int32_t) sample_count;
    mag_bias[2] /= (int32_t) sample_count;

    destMag[0] = (float) mag_bias[0]*MAG_UT_LSB;  // save mag biases in G for main program
    destMag[1] = (float) mag_bias[1]*MAG_UT_LSB;   
    destMag[2] = (float) mag_bias[2]*MAG_UT_LSB;  

    Serial.print(destMag[0]);
    Serial.print(' ');
    Serial.print(destMag[1]);
    Serial.print(' ');
    Serial.print(destMag[2]);
    Serial.println();

    Serial.println("We have mag offsets!!!!!!");
 }


void DPEng_ICM20948::magCal(float * dest1, float * dest2) 
 {
 uint16_t ii = 0, sample_count = 0;
 int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
 int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

 Serial.println("Mag Calibration: Wave device in a figure eight until done!");
 delay(4000);

// shoot for ~fifteen seconds of mag data
sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

for(ii = 0; ii < sample_count; ii++) {
/* Read 7 bytes from the AK09916 sensor */
  Wire.beginTransmission((byte)ICM20948_MAG_ADDRESS);
  #if ARDUINO >= 100
    Wire.write(AK09916_XOUT_L);
  #else
    Wire.send(AK09916_XOUT_L);
  #endif
  Wire.endTransmission();
  Wire.requestFrom((byte)ICM20948_MAG_ADDRESS, (byte)6);

  #if ARDUINO >= 100
    uint8_t mxlo = Wire.read();
    uint8_t mxhi = Wire.read();
    uint8_t mylo = Wire.read();
    uint8_t myhi = Wire.read();
    uint8_t mzlo = Wire.read();
    uint8_t mzhi = Wire.read();
  #endif

    mag_temp[0] = (int16_t)((mxhi << 8) | mxlo);
    mag_temp[1] = (int16_t)((myhi << 8) | mylo);
    mag_temp[2] = (int16_t)((mzhi << 8) | mzlo);
    
  for (int jj = 0; jj < 3; jj++) {
    if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
    if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
  }

  delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
}


// Get hard iron correction
 mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
 mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
 mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

 dest1[0] = (float) mag_bias[0]*MAG_UT_LSB;  // save mag biases in G for main program
 dest1[1] = (float) mag_bias[1]*MAG_UT_LSB;   
 dest1[2] = (float) mag_bias[2]*MAG_UT_LSB;  
   
// Get soft iron correction estimate
 mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
 mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
 mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

 float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
 avg_rad /= 3.0;

 dest2[0] = avg_rad/((float)mag_scale[0]);
 dest2[1] = avg_rad/((float)mag_scale[1]);
 dest2[2] = avg_rad/((float)mag_scale[2]);

 Serial.println("Mag Calibration done!");
 }

>>>>>>> develop
// Wire.h read and write protocols
uint8_t DPEng_ICM20948::writeByte(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t data)
{
  return writeByteWire(deviceAddress,registerAddress, data);
}

uint8_t DPEng_ICM20948::writeByteWire(uint8_t deviceAddress, uint8_t registerAddress,
                            uint8_t data)
{
  Wire.beginTransmission(deviceAddress);  // Initialize the Tx buffer
  Wire.write(registerAddress);      // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
  // TODO: Fix this to return something meaningful
  return NULL;
}

uint8_t DPEng_ICM20948::readBytes(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
    return readBytesWire(deviceAddress, registerAddress, count, dest);
}

uint8_t DPEng_ICM20948::readBytesWire(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
  // Initialize the Tx buffer
  Wire.beginTransmission(deviceAddress);
  // Put slave register address in Tx buffer
  Wire.write(registerAddress);
  // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);

  uint8_t i = 0;
  // Read bytes from slave register address
  Wire.requestFrom(deviceAddress, count);
  while (Wire.available())
  {
    // Put read results in the Rx buffer
    dest[i++] = Wire.read();
  }

  return i; // Return number of bytes written
}