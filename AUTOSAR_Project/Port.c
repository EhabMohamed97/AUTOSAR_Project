 /******************************************************************************
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port AUTOSAR Driver.
 *
 * Author: Ehab Mohamed
 ******************************************************************************/

#include "Port.h"
#include "Port_regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)
   #include "Det.h"
      /* AUTOSAR Version checking between Det and Dio Modules */
      #if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
         || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
         || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
         #error "The AR version of Det.h does not match the expected version"
      #endif

#endif
   
/*******************************************************************************
 *                      Static Functions Prototypes                            *
 *******************************************************************************/

/* 
 *  A function to get the pin index in the port.It takes the Pin number in the 
 *  MCU for example getPinNumber(PA4) returns 4.
 */
STATIC uint8 getPinNumber(Port_PinType);

/* 
 *  A function to get the port base address of the pin.It takes the pin number in
 *  the MCU and a pointer ,for example getPortBaseAddress(PA4, &PortGpio_Ptr) it
 *   retuns the Port number for the input pin and save the corresponding port
 *    (Port A)in the passed pointer to use it in another function.
 */
STATIC uint8 getPortBaseAddress(Port_PinType, volatile uint32**);

/*******************************************************************************
 *                      Static variables                                       *
 *******************************************************************************/

/* 
 * A variable to store the status of the module To indicate that the Port_Init
 *  function is called and the module has already been initialized. 
 */
volatile STATIC boolean Port_Status = PORT_NOT_INITIALIZED;

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType *ConfigPtr)
{
  Port_PinType pin_num;
  uint8 port_num;
  volatile uint32 * PortGpio_Ptr = NULL_PTR;   /* point to the required Port Registers base address */
  volatile uint32 delay;   /* used to make a 3 clock cycles delay after setting the RCGC2 Register */
  
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
      
      /* check if the input configuration pointer is not a NULL_PTR */
      if (NULL_PTR == ConfigPtr)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                         PORT_E_PARAM_CONFIG);
         return;
      }
      else
      {
         /* No action required */
      }
      
  #endif
 
  for(pin_num = PA0; pin_num < MCU_TOTAL_PINS_NUM; pin_num++)
  {
    /* 
     * Get the corresponding port number for the current pin and save
     *  save its register base address in PortGpio_Ptr 
     */
    port_num = getPortBaseAddress(pin_num, &PortGpio_Ptr);
    
     /* Enable clock for PORT and allow time for clock to start*/
    SYSCTL_REGCGC2_REG |= (1 << port_num);
    delay = SYSCTL_REGCGC2_REG;
    
    if( (pin_num == PD7) || (pin_num == PF0) ) /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;   /* Unlock the GPIOCR register */  
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET), getPinNumber(pin_num));   /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }   
    else if( (pin_num == PC0) || (pin_num == PC1) || (pin_num == PC2) || (pin_num == PC3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
        continue;
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
        
     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (getPinNumber(pin_num) * 4));   /* Clear the PMCx bits for this pin */
     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ( (ConfigPtr->MCU_Channels[pin_num]->pin_mode) << (getPinNumber(pin_num) * 4) );  /* Set the PMCx bits for this pin with the mode number */
     
    if(ConfigPtr->MCU_Channels[pin_num]->pin_mode == PIN_DIO_MODE)
    {
         CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), getPinNumber(pin_num));             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
         CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), getPinNumber(pin_num));      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), getPinNumber(pin_num));      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */      
    }
    else if(ConfigPtr->MCU_Channels[pin_num]->pin_mode == PIN_AIN_MODE)
    {
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), getPinNumber(pin_num));             /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), getPinNumber(pin_num));      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
         CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), getPinNumber(pin_num));      /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */           
    }
    else
    {
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), getPinNumber(pin_num));             /* Enable Alternative function for this pin by setting the corresponding bit in GPIOAFSEL register */
         CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), getPinNumber(pin_num));    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), getPinNumber(pin_num));       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    }
    
    if(ConfigPtr->MCU_Channels[pin_num]->direction == PORT_PIN_OUT)
    {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), getPinNumber(pin_num) );                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(ConfigPtr->MCU_Channels[pin_num]->initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), getPinNumber(pin_num) );          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), getPinNumber(pin_num) );        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
    }
    else if(ConfigPtr->MCU_Channels[pin_num]->direction == PORT_PIN_IN)
    {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , getPinNumber(pin_num) );             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(ConfigPtr->MCU_Channels[pin_num]->resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , getPinNumber(pin_num) );       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(ConfigPtr->MCU_Channels[pin_num]->resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , getPinNumber(pin_num) );     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , getPinNumber(pin_num) );     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , getPinNumber(pin_num) );   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
    }
    else
    {
        /* Do Nothing */
    }

  }   /* End for(pin_num=PA0;pin_num<MCU_TOTAL_PINS_NUM;pin_num++) */
  
  /* Set the module state to initialized */
  Port_Status = PORT_INITIALIZED;
  
}   /* End Port_Init() function */

/*******************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin->Port Pin ID number, Direction->Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction.
********************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{
  volatile uint32*PortGpio_Ptr = NULL_PTR;
  
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
     
      /* Check if the Driver is initialized before using this function */
      if (PORT_NOT_INITIALIZED == Port_Status)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
         return;
      }
      else
      {
         /* No Action Required */
      }
      
      /* Check if the used pin is within the valid range */
      if(Pin >= MCU_TOTAL_PINS_NUM)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
         return;
      }
      else
      {
         /* No Action Required */
      }
      
      /* Check if the direction of the used pin is changeable */
      if(FALSE == Port_Configuration.MCU_Channels[Pin]->pin_direction_changeable)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
         return;
      }
      else
      {
		/* No Action Required */
      }           
   #endif
 
  /* 
   * Get the corresponding port number for the current pin and
   *  save its register base address in PortGpio_Ptr 
   */
  getPortBaseAddress(Pin, &PortGpio_Ptr); 
  
  switch(Direction)
  {
      case PORT_PIN_IN:  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , getPinNumber(Pin) );   /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */   
                         break; 
                           
      case PORT_PIN_OUT: SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), getPinNumber(Pin) );   /* Set the corresponding bit in the GPIODIR register to configure it as output pin */ 
                         break;                       
  }   /* End switch(Direction) */   
   
}   /* End Port_SetPinDirection() function */
#endif

/*******************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction..
********************************************************************************/
void Port_RefreshPortDirection(void)
{
  Port_PinType Pin;
  volatile uint32 *PortGpio_Ptr = NULL_PTR;
  
  for(Pin = PA0; Pin < MCU_TOTAL_PINS_NUM; Pin++)
  {
    /* 
     * Get the corresponding port number for the current pin and
     *  save its register base address in PortGpio_Ptr 
     */
    getPortBaseAddress(Pin, &PortGpio_Ptr);
    
    if(Port_Configuration.MCU_Channels[Pin]->pin_direction_changeable == TRUE)
    {
      if(BIT_IS_CLEAR(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), getPinNumber(Pin) ) )   /* Set the corresponding bit in the GPIODIR register to configure it as output pin */ 
      {
         CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , getPinNumber(Pin) );             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */ 
      }
      else
      {
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , getPinNumber(Pin) );             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */ 
      }
    }   /* End if(Port_Configuration->MCU_Channels[pin_num]->pin_direction_changeable==FALSE) */
    
  } /* End for(pin_num=PA0;pin_num<MCU_TOTAL_PINS_NUM;pin_num++) */
  
}   /* End Port_RefreshPortDirection() function */

/*******************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo
* Return value: None
* Description: Returns the version information of this module..
********************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
   #if (PORT_DEV_ERROR_DETECT == STD_ON)
      /* Check if input pointer is not Null pointer */
      if(NULL_PTR == versioninfo)
      {
         /* Report to DET  */
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
         return;
      }
      else
      {
         /* No action required */
      }   /* End else*/
   
      /* Check if the Driver is initialized before using this function */
      if (PORT_NOT_INITIALIZED == Port_Status)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_UNINIT);
         return;
      }
      else
      {
         /* No Action Required */
      }
   
   #endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */
	
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
}   /* End Port_GetVersionInfo() function */

#endif  /* End #endif(PORT_VERSION_INFO_API==STD_ON) */

/*******************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin->Port Pin ID number, Mode->New Port Pin mode to be set on
                   port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
********************************************************************************/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
  volatile uint32*PortGpio_Ptr = NULL_PTR;
  
  #if (PORT_DEV_ERROR_DETECT == STD_ON)
      
      /* Check if the used pin is within the valid range */
      if(Pin >= MCU_TOTAL_PINS_NUM)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
         return;
      }
      else
      {
         /* No Action Required */
      }
      
       /* Check if the used pin is within the valid range */
      if( (Port_Configuration.MCU_Channels[Pin]->pin_mode > 10) && (Port_Configuration.MCU_Channels[Pin]->pin_mode != 14) )
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
         return;
      }
      else
      {
         /* No Action Required */
      }
      
      if(Port_Configuration.MCU_Channels[Pin]->pin_mode_changeable == FALSE)
      {
         /* Report to DET  */
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, 
                         PORT_E_MODE_UNCHANGEABLE);
         return;    
      }   /* End if(Port_Configuration->MCU_Channels[Pin]->pin_mode_changeable==FALSE) */
      
      /* Check if the Driver is initialized before using this function */
      if (PORT_NOT_INITIALIZED == Port_Status)
      {
         Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
         return;
      }
      else
      {
         /* No Action Required */
      }
      
  #endif
      
     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (getPinNumber(Pin) * 4));     /* Clear the PMCx bits for this pin */
     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ( (Port_Configuration.MCU_Channels[Pin]->pin_mode) << (getPinNumber(Pin) * 4) );
     
     if(Port_Configuration.MCU_Channels[Pin]->pin_mode == PIN_DIO_MODE)
     {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), getPinNumber(Pin));             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), getPinNumber(Pin));      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), getPinNumber(Pin));      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */      
     }
     else if(Port_Configuration.MCU_Channels[Pin]->pin_mode == PIN_AIN_MODE)
     {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), getPinNumber(Pin));             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), getPinNumber(Pin));      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), getPinNumber(Pin));      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */           
     }
     else
     {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), getPinNumber(Pin));             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), getPinNumber(Pin));      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), getPinNumber(Pin));      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
     }
      
}   /* End Port_SetPinMode() function */

/* 
 *  A function to get the pin index in the port.It takes the Pin number in the 
 *  MCU for example getPinNumber(PA4) returns 4.
 */
STATIC uint8 getPinNumber(Port_PinType MCU_pinNum)
{
  
  if(MCU_pinNum >= PF0)return MCU_pinNum % PF0;
  else return MCU_pinNum % 8;  
}   /* End getPinNumber() function */

/* 
 *  A function to get the port base address of the pin.It takes the pin number in
 *  the MCU and a pointer ,for example getPortBaseAddress(PA4, &PortGpio_Ptr) it
 *   retuns the Port number for the input pin and save the corresponding port
 *    (Port A)in the passed pointer to use it in another function.
 */
STATIC uint8 getPortBaseAddress(Port_PinType pinNumber, volatile uint32 **PortGpio_Ptr)
{
   uint8 port_num;
  
   if(pinNumber > PE5)port_num = 5;
   else  port_num = pinNumber / 8;
    
    
    switch(port_num)
    {
        case  0: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                 break;
                 
        case  1: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                 break;
                 
        case  2: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                 break;

        case  3: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                 break;
                 
        case  4: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                 break;
                 
        case  5: *PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                 break;
    }   /* End switch(port_num) */
    
    return port_num;
  
}   /* End getPortBaseAddress() function */