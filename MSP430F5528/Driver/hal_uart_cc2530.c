/**************************************************************************************************
  Filename:       hal_uart_cc2530.h
  Revised:        $Date: 2016-05-19 13:44:16 +0800 (Thus, 19 May 2016) $
  Revision:       $Revision: 1 $

  Description:    This file contains the interface to uart Service.


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
  UART 驱动
**************************************************************************************************/

/***************************************************************************************************
 *                                             INCLUDES
 ***************************************************************************************************/
#include "hal_uart_cc2530.h"
#include "msp430f5528.h"
/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/


/***************************************************************************************************
 *                                              TYPEDEFS
 ***************************************************************************************************/

  
/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/


/**************************************************************************************************
 *                                        INNER GLOBAL VARIABLES
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      UART1_Config_Init
 *
 * @brief   Initialize UART1
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void UART1_Config_Init(void)
{
  P4SEL |= BIT4 + BIT5; // P4.4 P4.5 设置成复用功能
  P4DIR |= BIT4;        // P4.4 设置为输出 UCA1TXD
  P4DIR &= ~BIT5;       // P4.5 设置成输入 UCA1RXD
  
  UCA1CTL1 |= UCSWRST;               		// Enable SW reset
  UCA1CTL0  = 0;	                        // [b0]   0 -  ASynchronous mode UART mode
                                                // [b2-1] 00- UART mode
                                                // [b3]   0 - One stop mode
                                                // [b4]   0 - 8-bit data
                                                // [b5]   0 - LSB first
                                                // [b6]   0 - b7disable 该位无效
                                                // [b7]   0 - 无奇偶校验位.
  
  UCA1CTL1 |= UCSSEL_2;               	// select SMCLK as clock source 16Mhz
//  UCA1BR0 = 0xA0;                       // 16Mhz 115200
//  UCA1BR1 = 0x01;
//  UCA1MCTL = UCBRS_6;
  UCA1BR0 = 0x8A;                       // 16Mhz 115200
  UCA1BR1 = 0x00;
  UCA1MCTL = UCBRS_7;
  UCA1CTL1 &= ~UCSWRST;                 // Clear SW reset, resume operation  
  
  UCA1IFG &= ~UCRXIFG;                  // 清空接收中断标志
  UCA1IE |= UCRXIE;                     // 打开接收中断 
}


/**************************************************************************************************
 * @fn      UART1_Send_Buffer
 *
 * @brief   Send buffer
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void UART1_Send_Buffer(uint8 *buffer,uint16 len)
{
  uint16 i;
  for(i = 0; i < len ; ++i)
  {
    UCA1TXBUF = buffer[i];
    while(!(UCA1IFG & UCTXIFG)); //等待发送结束 
  }
}


/**************************************************************************************************
 * @fn      UUART1_Send_Byte
 *
 * @brief   Send Byte
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void UART1_Send_Byte(uint8 data)
{
  UCA1TXBUF = data;
  while(!(UCA1IFG & UCTXIFG)); //等待发送结束 
}




