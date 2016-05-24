/**************************************************************************************************
  Filename:       hal_rtc.c
  Revised:        $Date: 2016-04-07 15:20:16 +0800 (Tues, 7 Apr 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to the RTC Service.


  Copyright 2016 Bupt. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact kylinnevercry@gami.com.
  使用MSP430本身的RTC
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_rtc.h"

/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/

/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

/**************************************************************************************************
 *                                        INNER GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
  
/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/
  
/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalRTCInit
 *
 * @brief   Initialize RTC Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalRTCInit(void)
{
  RTCCTL01 = RTCHOLD + RTCMODE; // RTCmode,hexadecimal mode,RTC Source Select RT1PS
  
  
  /* Setting DS1302 default time */
  /* 2016-4-26 00:00:00 周二 */
  RTCStruct_t RTCStruct;
  HalRTCStructInit(&RTCStruct,0,0,0,1,5,2,16);
  HalRTCGetOrSetFull(RTC_DS1302_SET,&RTCStruct);
  
  /* Setting RTC clock */
  RTCPS1CTL |= RT0PSDIV_7;
  /* Start RTC work */
  RTCCTL01 &= ~RTCHOLD;
}
  

/**************************************************************************************************
 * @fn      HalRTCGetOrSetFull
 *
 * @brief   Set or Get RTC ALL register data.
 *
 * @param   GetOrSetFlag -- RTC_DS1302_GET、RTC_DS1302_SET
 *          RTCStructTemp -- store the calendar to set or store the calendar get from DS1302.
 * @return  
 **************************************************************************************************/
void HalRTCGetOrSetFull(uint8 getOrSetFlag, RTCStruct_t *RTCStruct)
{ 
  if(getOrSetFlag == RTC_DS1302_SET) // Write data
  {
    RTCCTL01 |= RTCHOLD;
    RTCSEC = RTCStruct->sec;
    RTCMIN = RTCStruct->min;
    RTCHOUR = RTCStruct->hour;
    RTCDOW = (RTCStruct->week - 1);
    RTCDAY = RTCStruct->date;
    RTCMON = RTCStruct->month;
    RTCYEAR = RTCStruct->year;
    RTCCTL01 &= ~RTCHOLD;
  }
  else if(getOrSetFlag == RTC_DS1302_GET) // Read data
  {
    while(!(RTCCTL01 & RTCRDY));
    // Get useful value and change data
    RTCStruct->sec = RTCSEC;
    RTCStruct->min = RTCMIN;
    RTCStruct->hour = RTCHOUR;
    RTCStruct->week = RTCDOW + 1;
    RTCStruct->date = RTCDAY;
    RTCStruct->month = RTCMON;
    RTCStruct->year = RTCYEAR;
  }
  else
  {}
  
  return;
  
}

/**************************************************************************************************
 * @fn      HalRTCGetOrSet
 *
 * @brief   Set or Get RTC one register data.
 *
 * @param   GetOrSetFlag -- RTC_DS1302_GET、RTC_DS1302_SET
 *          RegisterName -- which register want to operation
 *          value -- get return value . set return 0xFF.All use decimal.
 * @return  
 **************************************************************************************************/
void HalRTCGetOrSet(uint8 getOrSetFlag,uint8 registerName,uint8 *value)
{
  if(getOrSetFlag == RTC_DS1302_SET)  //设置值
  {
    RTCCTL01 |= RTCHOLD;
    switch(registerName)  //Set useful value
    {
      case RTC_REGISTER_SEC   : RTCSEC = *value ; break;  //bit:0-6
      case RTC_REGISTER_MIN   : RTCMIN = *value ; break;  //bit:0-6
      case RTC_REGISTER_HOUR  : RTCHOUR = *value ; break;  //bit:0-5
      case RTC_REGISTER_DATE  : RTCDOW = *value ; break;  //bit:0-5
      case RTC_REGISTER_MONTH : RTCDAY = *value ; break;  //bit:0-4
      case RTC_REGISTER_WEEK  : RTCMON = (*value - 1) ; break;  //bit:0-2
      case RTC_REGISTER_YEAR  : RTCYEAR = *value ; break;  //bit:0-7
    }
    RTCCTL01 &= ~RTCHOLD;
  }
  else if(getOrSetFlag == RTC_DS1302_GET) // 获取值
  {
    while(!(RTCCTL01 & RTCRDY));
    switch(registerName)  //Get useful value
    {
      case RTC_REGISTER_SEC   : *value = RTCSEC ; break;  //bit:0-6
      case RTC_REGISTER_MIN   : *value = RTCMIN ; break;  //bit:0-6
      case RTC_REGISTER_HOUR  : *value = RTCHOUR ; break;  //bit:0-5
      case RTC_REGISTER_DATE  : *value = RTCDAY ; break;  //bit:0-5
      case RTC_REGISTER_MONTH : *value = RTCMON ; break;  //bit:0-4
      case RTC_REGISTER_WEEK  : *value = (RTCDOW + 1) ; break;  //bit:0-2
      case RTC_REGISTER_YEAR  : *value = RTCYEAR ; break;  //bit:0-7
    }    
  }
  else
  {
  }
  return;
}

/**************************************************************************************************
 * @fn      HalRTCStructInit
 *
 * @brief   Init RTC struct
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void HalRTCStructInit(RTCStruct_t *RTCStruct,uint8 sec,uint8 min,uint8 hour,uint8 date,
                             uint8 month,uint8 week,uint8 year)
{
  RTCStruct->sec = sec;
  RTCStruct->min = min;
  RTCStruct->hour = hour;
  RTCStruct->date = date;
  RTCStruct->month = month;
  RTCStruct->week = week;
  RTCStruct->year = year;
  RTCStruct->WP = 0;
}
