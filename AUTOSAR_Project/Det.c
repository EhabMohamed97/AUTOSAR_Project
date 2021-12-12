 /******************************************************************************
 *
 * Module: Det
 *
 * File Name: Det.c
 *
 * Description:  Det stores the development errors reported by other modules.
 *
 * Author: Ehab Mohamed
 ******************************************************************************/

#include "Det.h"

Std_ReturnType Det_ReportError( uint16 ModuleId,
                      uint8 InstanceId,
                      uint8 ApiId,
                      uint8 ErrorId )
{
  
  uint8 c = 20;
    
  while(1)
    {
      
      if(10 == c)return E_OK;   /* To solve the warning  :'D */
    }

}

