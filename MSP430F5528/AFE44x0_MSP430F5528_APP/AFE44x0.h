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

#ifndef AFE44x0_H_
#define AFE44x0_H_



#define AFE_RESETZ      BIT1
#define AFE_PDNZ        BIT7
#define AFE_ADC_DRDY    BIT2
#define AFE_PD_ALM      BIT3
#define AFE_LED_ALM     BIT4
#define AFE_DIAG_END    BIT6

struct AFE44xx_state{
	unsigned char state;
	unsigned char SamplingRate;
	unsigned char command;
};

typedef enum stECG_RECORDER_STATE {
	
	IDLE_STATE =0,
	DATA_STREAMING_STATE,
	ACQUIRE_DATA_STATE,
	ECG_DOWNLOAD_STATE,
	ECG_RECORDING_STATE
}ECG_RECORDER_STATE;

/****************************************************************/
/* Global functions*/
/****************************************************************/
void Init_AFE44xx_Resource(void);
void AFE44xx_Default_Reg_Init(void);
void AFE44xx_Reg_Write(unsigned char Reg_address, unsigned long Reg_data);
unsigned long AFE44xx_Reg_Read(unsigned char Reg_address);

void Init_AFE44xx_DRDY_Interrupt (void);
void Enable_AFE44xx_DRDY_Interrupt (void);
void Disable_AFE44xx_DRDY_Interrupt (void);
void Set_GPIO(void);
void Set_UCB1_SPI(void);
void AFE44xx_Read_All_Regs(unsigned long AFE44xxeg_buf[]);
void AFE44xx_PowerOn_Init(void);
void AFE44xx_Parse_data_packet(void);
void ADS1292x_Parse_data_packet(void);
void Set_Device_out_bytes(void);

void AFE44xx_PowerOn(void);
void AFE44xx_PowerOff(void);

#endif /*AFE44x0_H_*/
