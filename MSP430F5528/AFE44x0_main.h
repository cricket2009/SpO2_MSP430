//******************************************************************************
//  Praveen Aroul
//  HealthTech, MHR
//  (C) Texas Instruments Inc., 2013
//  All Rights Reserved.
//  Built with IAR Workbench 5.50.2
//
//-------------------------------------------------------------------------------
// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
//--------------------------------------------------------------------------------

#ifndef AFE44x0_MAIN_H_
#define AFE44x0_MAIN_H_
#include "hal_type.h"               //Basic Type declarations
#define SOT     0x02
#define EOT     0x03
#define CR      0x0D

#define WRITE_REG_CMD                   0x02
#define READ_REG_CMD                    0x03
#define START_READ_ADC_REG_CMD          0x01
#define STOP_READ_ADC_REG_CMD           0x06
#define DEV_ID_CMD                      0x04
#define FW_UPGRADE_CMD                  0x05
#define FW_VERSION_CMD                  0x07

#define __AFE4400__
//#define __AFE4490__

typedef enum
{
  SpO2_ONLINE_IDLE,             // ÔÚÏßµÈ´ý×´Ì¬
  SpO2_ONLINE_MEASURE,          // ÔÚÏß²âÁ¿×´Ì¬
  SpO2_OFFLINE_IDLE,            // ÀëÏßµÈ´ý×´Ì¬
  SpO2_OFFLINE_MEASURE,         // ÀëÏß²âÁ¿×´Ì¬
  SpO2_FIND_NETWORK,            // ÕÒÍø×´Ì¬
  SpO2_SYNC_DATA,               // Í¬²½Êý¾Ý×´Ì¬
  SpO2_CLOSING,                 // ¹Ø±ÕÍøÂç×´Ì¬
  SpO2_ON_SLEEP,                // ÔÚÏßË¯Ãß×´Ì¬
  SpO2_OFF_SLEEP,               // ÀëÏßË¯Ãß×´Ì¬ 
} SpO2SystemStatus_t;

extern SpO2SystemStatus_t SpO2SystemStatus;

#define SPO2_Wait_Symbol_Start_X 11
#define SPO2_Wait_Symbol_Start_Y 40

#define HR_Wait_Symbol_Start_X   76
#define HR_Wait_Symbol_Start_Y   40

#define SPO2_Show3Num_Start_X   11
#define SPO2_Show3Num_Start_Y   30

#define HR_Show3Num_Start_X     76   
#define HR_Show3Num_Start_Y     30

#define SPO2_Show2Num_Start_X   17
#define SPO2_Show2Num_Start_Y   30

#define HR_Show2Num_Start_X     83   
#define HR_Show2Num_Start_Y     30

#define SPO2_Symbol_Start_X     8
#define SPO2_Symbol_Start_Y     16

#define PR_Symbol_Start_X     88
#define PR_Symbol_Start_Y     16

#define Heart_Sympol_Start_X    62
#define Heart_Sympol_Start_Y    0
//Function declarations
void Init_Ports (void);
void Init_Clock (void);
void Show_Wait_Symbol(const UCHAR *p);
void Init_TimerA1 (void);
void Cal_spo2_and_HR(void);
void Init_KEY_Interrupt (void);
//VOID Init_TimerA1 (VOID);
void Init_ADG1608();
void ADG1608_select(unsigned char S_number);

void UART_send(unsigned char* byt_string, int length);
void Init_UART();
void SendRedAndIRToCC2530(uint32 REDdata,uint32 IRdata,uint16 SpO2_temp,uint16 HR_temp);
//volatile unsigned char RxBuffer[6]; 
//unsigned char MMA8451_senddata[8];

void delay(unsigned long num);

void Init_MPY(void);

void GenericApp_GetWriteName();



#endif /*AFE44x0_MAIN_H_*/




























