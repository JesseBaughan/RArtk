/*!
 * \file      board.h
 *
 * \brief     Target board general functions implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdint.h>

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

  #define RADIO_RESET                                 10
  #define RADIO_MOSI                                  13
  #define RADIO_MISO                                  12
  #define RADIO_SCLK                                  4  //OLD BOARD
  // #define RADIO_SCLK                                  14  //NEW 
  #define RADIO_NSS                                   15
  #define RADIO_BUSY                                  19                                
  #define RADIO_DIO_1                                 14  //OLD
  // #define RADIO_DIO_1                                 4   //NEW
  #define RADIO_DIO_2                                 
  #define RADIO_DIO_3                                                             

/*!
 * \brief Disable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardDisableIrq( void );

/*!
 * \brief Enable interrupts
 *
 * \remark IRQ nesting is managed
 */
void BoardEnableIrq( void );

#endif // __BOARD_H__
