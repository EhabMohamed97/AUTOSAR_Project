 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM 
 *              Microcontroller - Port AUTOSAR Driver.
 * Author: Ehab Mohamed
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* The total number of pins in TM4C123GH6PM MCU */   
#define MCU_TOTAL_PINS_NUM 43

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)
   
/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/* Pre-compile option for presence of Port_SetPinDirection API */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

#define PIN_DIO_MODE      (0U)   /* Degital Input/Output mode number in TM4C123GH6PM */
#define PIN_AIN_MODE      (10U)   /* Analog Input mode number in TM4C123GH6PM */

#endif                                        