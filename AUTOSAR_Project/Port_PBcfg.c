 /******************************************************************************
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM 
 *              Microcontroller - Port AUTOSAR Driver
 *
 * Author: Ehab Mohamed
 ******************************************************************************/



/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#include "Port.h"
   
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_PBcfg.c does not match the expected version"
#endif

   /* Symbolic names for major(most used) configurations */
typedef enum
{
  PIN_DEFAULT_CONFIG, PIN_OUT_HIGH, PIN_IN_PULLUP_HIGH, PIN_IN_PULL_DOWN, PIN_OUT_LOW
}MajorPinsConfig;

/* Array of structures contians most used  pin configurations*/
STATIC const Port_configChannel pinsConfigSettings[] = { {PORT_PIN_IN, STD_LOW, OFF, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_OUT, STD_HIGH, OFF, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_IN, STD_HIGH, PULL_UP, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_IN, STD_HIGH, PULL_DOWN, PIN_DIO_MODE, STD_ON, STD_ON},
                                                         {PORT_PIN_OUT, STD_LOW, OFF, PIN_DIO_MODE, STD_ON, STD_ON}
                                                       };

/* 
 * PB configuration structure used by the Port_Init() fucntion to initialze all
 *   pins.
 * It is an array of pointers to structures.Each index is set with the address of
 *  the configuration structure of the corresponding pin.
 */
const Port_ConfigType Port_Configuration = {  &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PA7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PB7 */
                                              NULL_PTR,                                         /* PC0 */
                                              NULL_PTR,                                         /* PC1 */
                                              NULL_PTR,                                          /* PC2 */
                                              NULL_PTR,                                          /* PC3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC6 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PC7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD4 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD5 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PD6 */
                                              NULL_PTR,                                              /* PD7 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE0 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE3 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE4 */                                                    
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PE5 */
                                              &pinsConfigSettings[PIN_IN_PULLUP_HIGH],                 /* PF0 */
                                              &pinsConfigSettings[PIN_OUT_LOW],                 /* PF1 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PF2 */
                                              &pinsConfigSettings[PIN_DEFAULT_CONFIG],                 /* PF3 */
                                              &pinsConfigSettings[PIN_IN_PULLUP_HIGH]                  /* PF4 */
                                             };