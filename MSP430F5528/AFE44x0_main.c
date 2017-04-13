/*  
 * ======== AFE44x0_main.c ========
 * AFE44x0 Pulse Oximeter
 *
 * 
 *
 * ---------------------------------------------------------------------------*/

//--------------------------------------------------------------------------------

#include <intrinsics.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "msp430.h"
//#include "descriptors.h"

//#include "device.h"
#include "types.h"               //Basic Type declarations

#include "HAL_UCS.h"
#include "HAL_PMM.h"

#include "AFE44x0.h"
//#include "AFE44x0_FWVersion.h"
#include "AFE44x0_main.h"

#include "msp430f5528.h"

#include "AFE44x0.h"


#include "Spo2_Functions.h"   

#include "hal_battery_monitor.h"
#include "hal_SDcard.h"
#include "exfuns.h"
#include "ff.h"
#include "hal_uart_cc2530.h"
#include "PingPongBuf.h"
#include "hal_rtc.h"
#include "hal_oled.h"
//==============================================================================

#define SYS_CLK 16000000 //15597800    // 16Mhz


unsigned long AFE44xx_SPO2_Data_buf[2];

char sendDataFlag = 0;
char readDataFlag = 0;

int Fs= 80;
int Fs_OLED = 5;
//#define Fs 80     // ������

#define MA_N  20   //��һ�λ���ƽ���˲����� Fs/10


unsigned long MA_buf_Led1[MA_N] = {0}; // �����˲����ݻ�����
unsigned long MA_buf_Led2[MA_N] = {0};


unsigned long MAF_data_Led1 = 0;       // �����˲�����
unsigned long MAF_data_Led2 = 0;
unsigned long MAF_data_Led1_new =0;
unsigned long MAF_data_Led2_new =0;

unsigned long Led1_DC = 0;
unsigned long Led1_DC_new = 0;
unsigned long Led2_DC = 0;
unsigned long Led2_DC_new = 0;

//unsigned long Led1_AC = 0;
//unsigned long Led1_AC_new = 0;
//unsigned long Led2_AC = 0;
//unsigned long Led2_AC_new = 0;

float Led1_AC = 0;
float Led1_AC_new = 0;
float Led2_AC = 0;
float Led2_AC_new = 0;



signed int MAF_data_Led1_diff = 0;
signed int MAF_data_Led2_diff = 0;

unsigned int MAF_data_Led2_diff_abs = 0;


float R = 0.0;
float R1 = 0.0;
float R2 = 0.0;

unsigned int R1_temp = 0;
unsigned int R2_temp = 0;


float spo2_average = 0.0;
float spo2_average_prev = 0.0;
float spo2_out = 0.0;
float spo2_sum = 0.0;
//#define SPO2_BUF_L 100 
//float spo2_buf[SPO2_BUF_L] = {0.0};
int spo2_buf_len = 0;
int spo2_flag = 0;
int spo2_count = 0;
float spo2_temp = 0.0;
float spo2_instant = 0.0;
float spo2_instant1 = 0.0;
int  spo2 = 0;
int  spo2_prev = 0;
float spo2_instant1_sum = 0.0;

#define SPO2_N 160    // Ѫ���������ݴ��ڣ�ȡ3sƽ��

unsigned long dc_led1_buf[SPO2_N] = {0};
unsigned long dc_led2_buf[SPO2_N] = {0};
//unsigned int ac_led1_buf[SPO2_N] = {0};
//unsigned int ac_led2_buf[SPO2_N] = {0};
float ac_led1_buf[SPO2_N] = {0};
float ac_led2_buf[SPO2_N] = {0};




unsigned long led_up_val = 1200000;  // ��Χ
unsigned long led_very_up_val = 1300000;  // ��Χ
unsigned long led_low_val = 600000;
unsigned long led_very_low_val = 500000;

UCHAR small_change = 1;  // ��ǿ�ı䲽��
UCHAR fast_change = 5;
UCHAR led1_current = 0x14;  // led1������ʼֵ
UCHAR led2_current = 0x14;
unsigned long current_reg_val =  0x011414;    // Reg34:LEDCNTRL: LED CONTROL REGISTER ��ʼֵ����ʼ���� Red��3.90625mA, IR:3.90625mA

#define NRCOEFF 0.985 
float temp1 = 0.0;
float temp2 = 0.0;
signed int Led1_HR_AC_prev = 0;
signed int Led1_HR_AC =0;
signed int Led2_HR_AC_prev = 0;
signed int Led2_HR_AC =0;

signed int MAF_data_Led1_HR_ac = 0;

//#define MA_NAC  20  //��AC����ƽ���˲�����

signed int MAF_buf_Led1_diff_ac[] = {0}; // ΢�ֺ����ݻ����� 



int window_move_count = 0; //�����ƶ���������
int sum_count =0;
unsigned long sum = 0;

int window_move_count_old = 0;
#define HR_window_L  30  //��ʼ���ڳ���ΪС�ڲ�����һ��
//
signed int HR_window[HR_window_L] = {0}; // ��ʼ����������
int peak_count = 0; // ��ֵ��������
signed int local_max = 0;
int max_location = 0;
unsigned long LED_count = 0;
//float HR_interval_average = 0.0;
float HR_instant =0;
unsigned long HR_instant_temp =0;
float HR_average = 0;
float HR_average_prev = 0;

int HR_count = 0;

#define HR_BUF_L 10
float HR_buf[HR_BUF_L] = {0.0};

float HR_diff_average = 0;
float HR_diff_average_prev = 0;
float HR_diff_buf[HR_BUF_L] = {0.0};



int count = 0;
#define SUM_DIFF_AC_BUF_L 5
unsigned int sum_diff_ac_buf[SUM_DIFF_AC_BUF_L] = {0};
unsigned int sum_diff_ac = 0;
unsigned long sum_diff_ac_old =0;
unsigned long threshold_diff_ac =0;
unsigned long threshold_diff_ac_sum = 0;

signed int  MAF_data_Led2_diff_old = 0;

int HR_instant_buf[30] ={0};


int num =0;
int cycle_num = 0;
int cycle_num_OLED = 0;
#define HR_diff_window_L  30

signed int HR_diff_window[HR_diff_window_L] ={0};

int diff_window_move_count =0;
int local_diff_max = 0;
//int local_diff_max_old1 = 0;
//int local_diff_max_old2 = 0;

int local_diff_max_buf[3] = {0};


int max_diff_location = 0;
unsigned long sum_diff = 0;
int sum_diff_count = 0;
float HR_diff_instant = 0.0;

int HR_diff_instant_buf[30]={0};

int HR = 75;
int HR_prev = 0;
int HR_diff = 0;
int HR_diff_old = 0;
int HR_raw = 0;
int HR_raw_old = 0;
int HR_old = 75;
int HR_update = 0;
unsigned char txString[4] = "";
int send_num =0;

int Finger_out = 0;

extern void write_oled_command(unsigned char ucCmd);
extern void write_oled_data(unsigned char ucData);

SpO2SystemStatus_t SpO2SystemStatus;
int Finger_out_num = 0;   //��ָ�뿪״̬������


int HR_diff_count = 0; // ��ͨ��AC����źż����������ֵ�Ĵ������м���
int HR_OLEDflag = 0;  // ��ʼ����ʼ����OLED��HR�ı�־����Ϊ1��ʼ����OLED

//��ʱ����
int Device_waite_time1;

// pingPong buffer
PingPongBuf_t *pingPongBuf_ForSD;
uint8 bufferSendToCC2530[68];
uint8 writeNum;
uint8 bufferFullFlag  = 0;
char fileName[30]; // store file name
char pathname[30]; // read file name
// ��ʼ��֮ǰ�͹رտ��Ź�����Ȼ��ʼ��ʱ������������ϵͳ���ϸ�λ
int __low_level_init(void)
{
  // stop WDT
  WDTCTL = WDTPW + WDTHOLD;
 
  // Perform data segment initialization
  return 1;
}

/*  
 * ======== main ========
 */
void main (void)
{
    //�رտ��Ź� 
    WDTCTL = WDTPW + WDTHOLD;
    uint32 *dataTemp;
    uint8 loopNum = 10;
    //Stop watchdog timer
    //ϵͳ��ʼ��
    SpO2SystemStatus = SpO2_OFFLINE_IDLE;    //Ĭ����ͨ��ʾ״̬
    Init_Ports();                                               //Init ports (do first ports because clocks do change ports)
    SetVCore(3);
    Init_Clock();                                               //Init clocks
    //Init_MPY();
    AFE44xx_PowerOn_Init();
    AFE44xx_PowerOff();       //�رղ���
    Init_KEY_Interrupt();
    HalBattMonInit();   //Initialize Battery monitor
    
    // Init UART to CC2530
    UART1_Config_Init();
        
    // RTC��ʼ��
    HalRTCInit();
    // Initialize OLED
    HalOledInit();               
    // SD����ʼ��
//    while(SD_Initialize());
    while(loopNum--)
      SD_Initialize();
    exfuns_init();      // �����ļ�ϵͳ�ڴ�
    f_mount(0,fs);      // �����ļ�ϵͳ  
    f_mkdir("0:S");     // �����ļ���
    pingPongBuf_ForSD = NULL;
    
    GenericApp_OpenDir();
     //��ҳ����ʾ
    HalOledShowString(SPO2_Symbol_Start_X,SPO2_Symbol_Start_Y,16,"SpO2%");
    HalOledShowString(PR_Symbol_Start_X,PR_Symbol_Start_Y,16,"PR");   
    Show_Wait_Symbol("Off_IDLE");
    delay(500000);  
    //Init_TimerA1();
    //����ȫ�����ж�
    __enable_interrupt();            //Enable interrupts globally

 
    while (1)
    {
      if(SpO2SystemStatus == SpO2_OFFLINE_IDLE || SpO2SystemStatus == SpO2_ONLINE_IDLE)
      {
        // �ͷſռ�
        PingPongBufFree(pingPongBuf_ForSD);
        pingPongBuf_ForSD = NULL;
        bufferFullFlag  = 0;
        // �ر��ļ�
        f_close(file);
        //����ʾ��
        HalOledOnOff(HAL_OLED_MODE_ON);     
        if(Device_waite_time1 <5000)
        {
          delay(3000);
          Device_waite_time1++;
        }
        else
        {
           if(SpO2SystemStatus == SpO2_OFFLINE_IDLE)
             SpO2SystemStatus = SpO2_OFF_SLEEP;
           else if(SpO2SystemStatus == SpO2_ONLINE_IDLE)
             SpO2SystemStatus = SpO2_ON_SLEEP;
        }     
      }
      else if(SpO2SystemStatus == SpO2_OFF_SLEEP || SpO2SystemStatus == SpO2_ON_SLEEP)    //��������˯��״̬
      {
        // �ͷſռ�
        PingPongBufFree(pingPongBuf_ForSD);
        pingPongBuf_ForSD = NULL;
        bufferFullFlag  = 0;
        // �ر��ļ�
        f_close(file);
        
        //�ر���ʾ��
        HalOledOnOff(HAL_OLED_MODE_OFF);
        //�ر�AFE4400
        AFE44xx_PowerOff();
        //�ر�AFE4400���ж�
        Disable_AFE44xx_DRDY_Interrupt();
        //����͹���
        _BIS_SR(LPM3_bits + GIE);
      }
      else if(SpO2SystemStatus == SpO2_OFFLINE_MEASURE || SpO2SystemStatus == SpO2_ONLINE_MEASURE)  //���߻����߲���״̬
      { 
        if(bufferFullFlag  == 1) // д��SD��
        {
          PingPongBufRead(pingPongBuf_ForSD,&dataTemp);
          f_write(file,dataTemp,512,&bw);
          bufferFullFlag  = 0;
        }
        if (readDataFlag)        // ����Ƶ��80Hz��ÿ��readDataFlag��80�ε���1
        {
          readDataFlag = 0;
          cycle_num++;  // ��������
          //����OLE
          if(cycle_num == Fs)      // ÿ1s����һ����ʾ
          {
            cycle_num = 0;
            if(Finger_out)
            {
               if(Finger_out_num == 0)
               {
                 HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
                 HalOledShowString(SPO2_Symbol_Start_X,40,16,"Finger Lose");
                 OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //�������ͼ��
                }
               if(Finger_out_num > 5) //5s��û����ָ����5�β�����û����ָ,ֹͣ����
               {
                 //�ر�AFE4400���ж�
                 Disable_AFE44xx_DRDY_Interrupt();
                 //�ر�AFE4400
                 AFE44xx_PowerOff();
                 
                 HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
                 if(SpO2SystemStatus == SpO2_OFFLINE_MEASURE)   // ���߲���״̬�л������߿���
                 {
                   SpO2SystemStatus = SpO2_OFFLINE_IDLE;
                   Show_Wait_Symbol("Off_IDLE");
                 }
                 else if(SpO2SystemStatus == SpO2_ONLINE_MEASURE) // ���߲���״̬�л������߿���
                 {
                   SpO2SystemStatus = SpO2_ONLINE_IDLE;
                   Show_Wait_Symbol("On_IDLE ");
                 }
                 continue;
               }
              /*��ʾFinger Lose����*/
              Finger_out_num++;
//              HR_diff_count = 0; // ����ֵ�Ĵ�����������
              continue;
            }
            else
            {
              Finger_out_num = 0;
               //����HR(PR)             
               if(abs(HR_diff-HR_raw)<=2 && (HR_diff != HR_diff_old || HR_raw != HR_raw_old))
               {
                 HR = (HR_diff +HR_raw)/2;        // ����ֵ HR
                 if ((HR-HR_old)>3)               // ÿ�θ������Ϊ 3 BPM
                 { 
                   HR = HR_old+3;
                 }
                 else if((HR_old-HR)>3)
                 {
                   HR = HR_old -3;
                 }
                 HR_old = HR;
                 HR_diff_old = HR_diff;
                 HR_raw_old = HR_raw;
               }
               //����SPO2
               spo2 = (int)(spo2_instant1+0.5)+1;     // Ѫ��ֵ spo2, ��1��ΪУ����     
               if(spo2>100)
                 spo2 = 100;
              //���ԭ�е���ʾ
              HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
              /*��ʾspo2 ��HRֵ*/
              if(spo2<100)
                HalOledShowNum(SPO2_Show2Num_Start_X,SPO2_Show2Num_Start_Y,spo2,2,32);              
              else
                HalOledShowNum(SPO2_Show3Num_Start_X,SPO2_Show3Num_Start_Y,spo2,3,32);


//////////////////////////////////////////////////////////              
              if(HR_OLEDflag)
              { 
////////////////////////////////////////////////////////             
                if(HR < 100)
                  HalOledShowNum(HR_Show2Num_Start_X,HR_Show2Num_Start_Y,HR,2,32);              
                else
                  HalOledShowNum(HR_Show3Num_Start_X ,HR_Show3Num_Start_Y,HR,3,32);
/////////////////////////////////////////////////////////
              }//if(HR_//OLEDflag)
              else
              {
                 HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X,HR_Wait_Symbol_Start_Y,1);
                 HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X+16,HR_Wait_Symbol_Start_Y,1);
                 HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X+32,HR_Wait_Symbol_Start_Y,1);
              }
////////////////////////////////////////////////////////
              if(cycle_num_OLED == 0)
              {
                OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,1); //С��
                cycle_num_OLED = 1;
              }
              else
              {
                OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //����
                cycle_num_OLED = 0;                
              }
              continue;
            }
          }
        } //if(readDataFlag) 
      }
      else if(SpO2SystemStatus == SpO2_SYNC) // ����ͬ��״̬
      {
        SyncData();
      }
   }  //while(1)
} //main()



/*  
 * ======== Init_Clock ========
 */
void Init_Clock (void)
{
    //Initialization of clock module
    P1SEL |= BIT0;  // P1.0 as ACLK, ACLK (8Mhz)for AFE44X0
    P1DIR |= BIT0;  // output
//    P7SEL |= BIT7;  // P7.7 as MCLK, for test
//    P7DIR |= BIT7;  // output
    UCSCTL5 |= DIVPA__4; // ACLK/2 at external pin, (�� 16M/2)
    UCSCTL3 = SELREF__REFOCLK;  // use REFO to source the FLL,  FLLREFDIV (n): 000-1��Ƶ
    UCSCTL4 = (UCSCTL4 & ~(SELA_7|SELM_7)) | (SELA__DCOCLK) | (SELM__DCOCLK); //DCOCLK for ACLK and MCLK
    
    Init_FLL_Settle(SYS_CLK / 1000, SYS_CLK / 32768);   //Start the FLL,  DCOCLK = 24Mhz
//   Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);   //set FLL (DCOCLK)
}


void Show_Wait_Symbol(const UCHAR *p)
{
    HalOledShowString(SPO2_Symbol_Start_X,0,12,p);
    HalOledShowWaitSymbol(SPO2_Wait_Symbol_Start_X,SPO2_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(SPO2_Wait_Symbol_Start_X+16,SPO2_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(SPO2_Wait_Symbol_Start_X+32,SPO2_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X,HR_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X+16,HR_Wait_Symbol_Start_Y,1);
    HalOledShowWaitSymbol(HR_Wait_Symbol_Start_X+32,HR_Wait_Symbol_Start_Y,1);
    
    // ��ʾ����
    HalShowBattVol(BATTERY_MEASURE_SHOW);
}
/*  
 * ======== Init_Ports ========
 */
void Init_Ports (void)
{
    //Initialization of ports (all unused pins as outputs with low-level
    P1OUT = 0xFF;
    P1DIR = 0xFF;
    P2OUT = 0xFF;
    P2DIR = 0xFF;
    P3OUT = 0x00;
    P3DIR = 0xFF;
    P4OUT = 0x00;
    P4DIR = 0xFF;
    P5OUT = 0x00;
    P5DIR = 0xFF;
    P6OUT = 0x00;
    P6DIR = 0xFF;
//    P7OUT = 0x00;
//    P7DIR = 0xFF;
//    P8OUT = 0x00;
//    P8DIR = 0xFF;
    #if defined (__MSP430F563x_F663x)
		P9OUT = 0x00;
		P9DIR = 0xFF;
    #endif
}


// UART1--CC2530 �����жϺ���
#pragma vector=USCI_A1_VECTOR
__interrupt void UART1_CC2530RX_ISR(void)
{
  static uint8 receiveFlag = 0;
  static uint8 controlMessage = 0;
  Device_waite_time1 = 0;
  uint8 receiveMessage = UCA1RXBUF;
  switch(receiveFlag)
  {
    case 0:
      if(receiveMessage == DATA_START)
        receiveFlag = 1;
      else
        receiveFlag = 0;
      break;
    case 1:
        controlMessage = receiveMessage;
        receiveFlag = 2;
      break;
    case 2:
      if(receiveMessage == DATA_END)
      {
        Device_waite_time1 = 0;  //�ȴ�״̬����ֵ��0
        Finger_out_num = 0;      //������״̬��0
 
        // ����controlMessage��ֵ���в�ͬ�Ĵ���
        switch(controlMessage)
        {
          case START_MEASURE: //������������
            if(SpO2SystemStatus == SpO2_ON_SLEEP || SpO2SystemStatus == SpO2_ONLINE_IDLE) // ����˯��״̬�����߿���״̬
            {
              if(SpO2SystemStatus == SpO2_ON_SLEEP) // ����˯��״̬���˳��͹���
                LPM3_EXIT; // �˳��͹���
              
              SpO2SystemStatus = SpO2_ONLINE_MEASURE;
              
              HalOledShowString(SPO2_Symbol_Start_X,0,12,"On_Go   ");
              //����ʾ��
              HalOledOnOff(HAL_OLED_MODE_ON);
              // ��ʼ��д�����
              writeNum = 0;
              //��AFE4400
              AFE44xx_PowerOn();
              //��AFE4400���ж�
              Enable_AFE44xx_DRDY_Interrupt();

            }
            break;
          
          case STOP_MEASURE: // ֹͣ��������
            if(SpO2SystemStatus == SpO2_ONLINE_MEASURE)
            {
               SpO2SystemStatus = SpO2_ONLINE_IDLE;
              //�ر�AFE4400���ж�
              Disable_AFE44xx_DRDY_Interrupt();          
              //�ر�AFE4400
              AFE44xx_PowerOff();
              HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
              Show_Wait_Symbol("On_IDLE ");
              OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //�������ͼ��       
            }
            break;
            
          case SYNC_MEASURE: // ͬ����Ϣ
            if(SpO2SystemStatus == SpO2_ON_SLEEP || SpO2SystemStatus == SpO2_ONLINE_IDLE) // ����˯��״̬�����߿���״̬
            {
              if(SpO2SystemStatus == SpO2_ON_SLEEP) // ����˯��״̬���˳��͹���
                LPM3_EXIT; // �˳��͹���
              
              SpO2SystemStatus = SpO2_SYNC;
              
              HalOledShowString(SPO2_Symbol_Start_X,0,12,"On_Sync ");
              //����ʾ��
              HalOledOnOff(HAL_OLED_MODE_ON);

            }            
            break;
            
          case FIND_NWK:  // ����������Ϣ
            if(SpO2SystemStatus == SpO2_ONLINE_MEASURE || SpO2SystemStatus == SpO2_OFFLINE_MEASURE) // ������������߻����߲���
            {
              // ֹͣ����
              Disable_AFE44xx_DRDY_Interrupt();          
              //�ر�AFE4400
              AFE44xx_PowerOff();
           
            }
            if(SpO2SystemStatus == SpO2_OFF_SLEEP || SpO2SystemStatus == SpO2_ON_SLEEP) // �����˯��
            {
               LPM3_EXIT;
               //����ʾ��
               HalOledOnOff(HAL_OLED_MODE_ON);
            }
            SpO2SystemStatus = SpO2_FIND_NETWORK;
            HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
            OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //�������ͼ��   
            Show_Wait_Symbol("FIND_NWK");
            break;
            
          case END_DEVICE:  // �ҵ�������Ϣ ֻ���ܴ�FIND_NWK�����״̬
            SpO2SystemStatus = SpO2_ONLINE_IDLE;
            Show_Wait_Symbol("On_IDLE ");
            break;
             
          case CLOSEING:   // ���ڹر�������Ϣ
            if(SpO2SystemStatus == SpO2_ONLINE_MEASURE) // ���߲���״̬�ر�����
            {
              // �رղ���
              Disable_AFE44xx_DRDY_Interrupt();          
              //�ر�AFE4400
              AFE44xx_PowerOff();
              OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //�������ͼ��             
            }
            // ���߿���״̬��Ѱ������״̬���¹ر�����
            if(SpO2SystemStatus == SpO2_ON_SLEEP) // ˯��״̬�¹ر�����
            {
              HalOledOnOff(HAL_OLED_MODE_ON);
              LPM3_EXIT;    
            }
            SpO2SystemStatus = SpO2_CLOSING;
            HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
            Show_Wait_Symbol("CLOSING ");
            break;
            
          case CLOSE_NWK:
              SpO2SystemStatus = SpO2_OFFLINE_IDLE;
              Show_Wait_Symbol("Off_IDLE");
            break;
          default:break;

        }
        
        receiveFlag = 0;
      }
      break;
  }
}

// Port 2 interrupt service routine ����Ƶ��Ϊ80Hz��Ҳ����ÿ1/80s����һ���ж�
#pragma vector=PORT2_VECTOR  //DRDY interrupt
__interrupt void Port_2(void)
{
  
  switch (P2IV)
  {
  case P2IV_P2IFG2:   
    P2IFG &= ~BIT2;                                   // Clear P2.2 IFG i.e Data RDY interrupt status
    //P5OUT |= BIT0;                                  //Turn on LED P5.0 (Green)
    readDataFlag = 1;                                 // Set Flag to read AFE44x0 ADC REG data
    //����SPO2��HR��ֵ
    Cal_spo2_and_HR();                                // ÿ��һ���жϣ�����һ��
    break;
    
  case P2IV_NONE:
    break;
  }
}

// Port 1 �����жϷ����� ,P1.6
#pragma vector=PORT1_VECTOR  //DRDY interrupt
__interrupt void Port_1(void)
{
  P2IE &= ~BIT2;                        
  switch (P1IV)
  {
  case P1IV_P1IFG6:
    P1IE &= ~BIT6;               // P1.7 interrupt DISABLE
    Device_waite_time1 = 0;  // �ȴ�״̬����ֵ��0
    Finger_out_num = 0;      // ������״̬��0
    writeNum = 0;            // ��ʼ��д�����
    if(SpO2SystemStatus == SpO2_ON_SLEEP || SpO2SystemStatus == SpO2_OFF_SLEEP)//����˯��̬�������̰����ǽ���ȴ�̬��ֻ������״̬���ܽ���ȴ�״̬
    {
      if(SpO2SystemStatus == SpO2_OFF_SLEEP)   // ����˯��״̬�л������߿���
        SpO2SystemStatus = SpO2_OFFLINE_IDLE;
      else if(SpO2SystemStatus == SpO2_ON_SLEEP) // ����˯��״̬�л������߿���
        SpO2SystemStatus = SpO2_ONLINE_IDLE;
      LPM3_EXIT;
    }
    else    //���ڵȴ�״̬���ǲ���״̬�а�������
    {
      int Press_type = 0;       //0��ʾ�̰���1��ʾ����
      //�ж��ǳ������Ƕ̰�
      for(Press_type = 0;Press_type < 1000; Press_type++)//16Mhz�� Լ1.22s
      {
        if(P1IN&BIT6)    //P1.6�Ǹߵ�ƽ����ʾ�ɿ�����
          break;
        delay(2000);
      }
      if(Press_type != 1000)    //�̰�
        Press_type = 0;
      else
        Press_type = 1;         //����
      
      
      switch(SpO2SystemStatus)
      {
          case SpO2_OFFLINE_IDLE: // ���ߵȴ�״̬
            if(Press_type == 0)//�̰�����ʼ����
            {      
              SpO2SystemStatus = SpO2_OFFLINE_MEASURE;
              // ����ռ�
              pingPongBuf_ForSD = PingPongBufInit(BUFFER_WRITE_SIZE);
              // ���ļ�
              GenericApp_GetWriteName();
              f_open(file,fileName,FA_CREATE_ALWAYS | FA_WRITE);
              HalOledShowString(SPO2_Symbol_Start_X,0,12,"Off_Go  ");
              //��AFE4400
              AFE44xx_PowerOn();
              //��AFE4400���ж�
              Enable_AFE44xx_DRDY_Interrupt();
            }
            else//����������
              SpO2SystemStatus = SpO2_OFF_SLEEP;
            break;
          
          case SpO2_ONLINE_IDLE: // �������ߵȴ�״̬
            if(Press_type == 0)//�̰�����ʼ����
            {          
              SpO2SystemStatus = SpO2_ONLINE_MEASURE;
              HalOledShowString(SPO2_Symbol_Start_X,0,12,"On_Go   ");
              //��AFE4400
              AFE44xx_PowerOn();
              //��AFE4400���ж�
              Enable_AFE44xx_DRDY_Interrupt();
            }
            else//����������
              SpO2SystemStatus = SpO2_ON_SLEEP;
            break;
            
          case SpO2_OFFLINE_MEASURE: //�������߲���״̬
            if(Press_type == 0)//�̰���ֹͣ����
            {
              SpO2SystemStatus = SpO2_OFFLINE_IDLE;
              //�ر�AFE4400���ж�
              Disable_AFE44xx_DRDY_Interrupt();          
              //�ر�AFE4400
              AFE44xx_PowerOff();
              HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
              Show_Wait_Symbol("Off_IDLE");
              OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //�������ͼ��
            }
            else//����������
              SpO2SystemStatus = SpO2_OFF_SLEEP;
            break;
            
          case SpO2_ONLINE_MEASURE: // �������߲���״̬
            if(Press_type == 0)//�̰���ֹͣ����
            {
              SpO2SystemStatus = SpO2_ONLINE_IDLE;
              //�ر�AFE4400���ж�
              Disable_AFE44xx_DRDY_Interrupt();          
              //�ر�AFE4400
              AFE44xx_PowerOff();
              HalOledShowString(0,32,32,"        ");  //8���ո���ȫ���
              Show_Wait_Symbol("On_IDLE ");
              OLED_ShowHeartSymbol(Heart_Sympol_Start_X,Heart_Sympol_Start_Y,1,0); //�������ͼ��
            }
            else//����������
              SpO2SystemStatus = SpO2_ON_SLEEP;
            break;

           default:break; // ����״ֱ̬���˳�������
      }
    }
    P1IFG &= ~BIT6;             // P1.6 IFG cleared
    P1IE |= BIT6;               // P1.7 interrupt enabled
    break;
    
  case P1IV_NONE:
    break;
  }
  P2IE |= BIT2;  
}
unsigned char ascii2uint8 (unsigned char asciiVal)
{
	unsigned char uint8Val;
	
	if (asciiVal > 0x60 && asciiVal < 0x67)			// 'a' to 'f'
		uint8Val = ((asciiVal - 0x61) + 0x0A);
  	else if (asciiVal > 0x40 && asciiVal < 0x47)		// 'A' to 'F'
  		uint8Val = ((asciiVal - 0x41) + 0x0A);
  	else if (asciiVal > 0x2F && asciiVal < 0x3A)		// '0' to '9'
  		uint8Val = (asciiVal - 0x30);
  	else												// others
  		uint8Val = 0;
  		
  	return uint8Val;
}




/*  
 * ======== init_UART  ========
 */
void Init_UART()
{
  P4SEL |= BIT4+BIT5;                        // P4.4,5 = USCI_A1 TXD/RXD
  //P4DIR |= BIT4;
  UCA1CTL1 |= UCSWRST;                       // **Put state machine in reset**
  UCA1CTL1 |= UCSSEL_1;                      // ACLK -- 16MHz
  UCA1MCTL |= UCOS16;                        // UCOS16 =1, oversampling mode
  
//  UCA1BR0 = 52;                            // 16MHz 19200 Buard rate (see User's Guide), 
//  UCA1BR1 = 0;                             // 16MHz 
//  UCA1MCTL |=  UCBRF_1;                    // Modulation UCBRSx=0, UCBRFx=1
    
  UCA1BR0 = 104;                             // 16MHz 9600 Buard rate (see User's Guide), 
  UCA1BR1 = 0; 
  
  UCA1MCTL |= UCBRS_3;
 
  UCA1CTL1 &= ~UCSWRST;                      // **Initialize USCI state machine**
}


/*  
 * ======== UART_send  ========
 */
void UART_send(unsigned char* byt_string, int length)
{
  int ii;
  for(ii=0; ii<length;ii++)
  {
    while(UCA1STAT & UCBUSY);
    UCA1TXBUF = *(byt_string+ii);
  }
}

void Cal_spo2_and_HR(void)
{
    BufOpStatus_t OpStatus;
    AFE44xx_SPO2_Data_buf[0] = AFE44xx_Reg_Read(46);  //read RED - Ambient Data
    AFE44xx_SPO2_Data_buf[1] = AFE44xx_Reg_Read(47);  //read IR - Ambient Data
    
    if(SpO2SystemStatus == SpO2_ONLINE_MEASURE) // �������߲�����������
    {
      SendRedAndIRToCC2530(AFE44xx_SPO2_Data_buf[0],AFE44xx_SPO2_Data_buf[1],spo2,HR);
    }
    else // �������߲���״̬ д64�������0.8s
    {
      OpStatus = PingPongBufWrite(pingPongBuf_ForSD,AFE44xx_SPO2_Data_buf[0]); // ��дRED
      OpStatus = PingPongBufWrite(pingPongBuf_ForSD,AFE44xx_SPO2_Data_buf[1]); // ��дIR
      //�������ִ�в�ͬ���¼�
      if (OpStatus == PingPongBuf_WRITE_SWITCH) // Success and switch buff
      {
        bufferFullFlag = 1;
      }
      else if(OpStatus == PingPongBuf_WRITE_FAIL )// fail
      {
        PingPongBufReset(pingPongBuf_ForSD);
      }
      else  //Success
      { //do nothing
      }      
    }
    
    if (AFE44xx_SPO2_Data_buf[0] > 2000000 ||AFE44xx_SPO2_Data_buf[1] > 2000000 )//�ж�Finger OUt״̬
    {
      Finger_out = 1;  // finger out ��־��1����ʾ��ָ����ָ��
    }
    else
    {
      Finger_out = 0;  // finger out ��־����
      //����ƽ���˲��˳���Ƶ���� 
      MAF_data_Led1_new = Moving_Average_unsig(AFE44xx_SPO2_Data_buf[0],MAF_data_Led1,MA_buf_Led1,MA_N);  // �������е�LED1��Ӧdatasheet��LED2��
      MAF_data_Led2_new = Moving_Average_unsig(AFE44xx_SPO2_Data_buf[1],MAF_data_Led2,MA_buf_Led2,MA_N);  //�������е�LED2��Ӧdatasheet��LED1��
      
      //DC      
      Led1_DC_new = Moving_Average_unsig(MAF_data_Led1_new,Led1_DC,dc_led1_buf,SPO2_N);  // �Ե�ͨ�˲���������ڴ�������SPO2_NҪ����һ��PPG���ڣ�ȡ5~10�����ڣ�����ƽ���õ�DC
      Led2_DC_new = Moving_Average_unsig(MAF_data_Led2_new,Led2_DC,dc_led2_buf,SPO2_N);
      //AC      
      MAF_data_Led1_diff = MAF_data_Led1_new - MAF_data_Led1;   // ΢�ֺ�ȡ����ֵ�� ��Ϊabs��AC��
      MAF_data_Led2_diff = MAF_data_Led2_new - MAF_data_Led2;   // ΢�ֺ�ȡ����ֵ  
          
      Led1_AC_new = Moving_Average_float(abs(MAF_data_Led1_diff),Led1_AC,ac_led1_buf,SPO2_N);  
      Led2_AC_new = Moving_Average_float(abs(MAF_data_Led2_diff),Led2_AC,ac_led2_buf,SPO2_N);
          
      //��������        
      Led1_DC = Led1_DC_new;
      Led2_DC = Led2_DC_new; 
          
      Led1_AC = Led1_AC_new;
      Led2_AC = Led2_AC_new;   
      //Ѫ��ֵ����        
      R1 = (float)Led1_AC/Led2_AC;   // Ѫ������
      R2 = (float)Led2_DC/Led1_DC;
          
      R = R2*R1;
      spo2_instant1 = 110-25*R;                    //˲ʱѪ��ֵ 
          
      //HR calculation===================================================*/     
      /* HR ������� ԭʼ����AC��ֵ�б���һ��΢�����ݷ�ֵ�б����ϵķ��������ַ����ֱ�õ��Ľ����һ������Ϊ�˴η�ֵ������Ч */
      /* AFE4400 �����Ϊ��ǿ�仯�źţ�������PPG�ź��෴�������ǵ�PPG���䲨Ӱ�죬�������η�ֵЧ���Ϻ�*/  
        
      //=====================================����ԭʼ����AC�ɷ���HR================================================//          
       MAF_data_Led1 = MAF_data_Led1_new;	//��������(SPO2����������)
       temp2 = (NRCOEFF * Led2_HR_AC_prev);
       Led2_HR_AC = (signed int)(MAF_data_Led2_new - MAF_data_Led2) + (signed int) temp2;
       MAF_data_Led2 = MAF_data_Led2_new;        
       Led2_HR_AC_prev = Led2_HR_AC;                       // ��ȡIR led AC�ɷ�
     
       fifo_move(Led2_HR_AC,HR_window,HR_window_L);        // �ƶ�����
       window_move_count = window_move_count +1;           // �����ƶ��������� 
          
       local_max = HR_window[0];                           // Ѱ��HR_window�ڵ����ֵ
       for (int ii = 1; ii < HR_window_L; ii++)
       {
         if (HR_window[ii] > local_max)
         {
           local_max = HR_window[ii];                    // �������������ֵ
           max_location = ii;                            // ���ֵ�ڴ���������
         }
       }
       if (max_location == HR_window_L/2-1 || max_location == HR_window_L/2)     // �����ֵλ����HR_window�����м䣬����Ϊ��ֵ��������HR_window ����ΪHR_window_L
       {
         if(window_move_count >= Fs/3 && window_move_count < 2*Fs)               //��������ֵ�����жϣ�����Fs/3��180BPM��С��2Fs��30BPM������Ϊ��Ч
         {
           HR_instant = (float)Fs*60/window_move_count;                         // HR ����
          
           HR_average = Moving_Average_float(HR_instant,HR_average_prev,HR_buf,HR_BUF_L);  // ��HR_instant����MA�˲���HR_average Ϊfloat����       
           
           HR_raw = (int)(HR_average + 0.5);             // HR_raw ��������ԭʼAC���ݵõ���HR
           
           HR_average_prev = HR_average;
           
           HR_update = 1;
           
         } 
         window_move_count = 0;      // ���ѵ���ֵʱ��window_move_count��ʼ��Ϊ0�����¼���                        
       }        
       
       //================================= ����һ��΢�ֺ�������HR================================================//          
       if (MAF_data_Led2_diff >0)
       {
         MAF_data_Led2_diff = 0;   // ΢�ִ�����ֵ�������㣨��Ϊ����ΪPPGȡ��
       }
       MAF_data_Led2_diff_abs = abs(MAF_data_Led2_diff);  //С����ı������

       fifo_move(MAF_data_Led2_diff_abs,HR_diff_window,HR_diff_window_L);  // �����ƶ�
       diff_window_move_count = diff_window_move_count +1;                 // �����ƶ��������� 
          
       local_diff_max = HR_diff_window[0];                                 // Ѱ��HR_diff_window�ڵ����ֵ�����ֵ��������
       for (int ii = 1; ii < HR_diff_window_L; ii++)
       {
         if (HR_diff_window[ii] > local_diff_max)
         {
           local_diff_max = HR_diff_window[ii];
           max_diff_location = ii;                                        //���ֵ��������
         }
       }
       
       if (max_diff_location == HR_diff_window_L/2-1 || max_diff_location == HR_diff_window_L/2 )     // ��ֵλ���Ƿ���HR_window�����м䣬�����м䣬���ʾ����һ����ֵ������������Ĵ���
       {
         // �������������������ֵ��С����ֵ�� ���û��������¼�����ֵ��Ϊ��ֹ��ֵ̫����ɵ�HR�޷�����
         if ( local_diff_max_buf[0] < 0.7*threshold_diff_ac && local_diff_max_buf[1] < 0.7*threshold_diff_ac && local_diff_max_buf[2] < 0.7*threshold_diff_ac )
         {
           for (int jj = 0; jj<SUM_DIFF_AC_BUF_L;jj++)
           {
             sum_diff_ac_buf[jj] = 0;
           }                    
           threshold_diff_ac_sum = 0;
           threshold_diff_ac =0;
         }
         local_diff_max_buf[2] = local_diff_max_buf[1];   // �ֲ����ֵ������£��������εģ�
         local_diff_max_buf[1] =  local_diff_max_buf[0];  
         local_diff_max_buf[0] = local_diff_max; 
         
         if(diff_window_move_count >= Fs/3 && diff_window_move_count < 2*Fs && local_diff_max> 0.7*threshold_diff_ac )       //��������ֵ�����жϣ�����Fs/3��С��2Fs,���Ҽ���ֵҪ������ֵ������Ϊ��Ч
         {
           threshold_diff_ac_sum =  sum_AC(local_diff_max,threshold_diff_ac_sum,sum_diff_ac_buf,SUM_DIFF_AC_BUF_L);   //������ֵ��ΪǰSUM_DIFF_AC_BUF_L����Ч����ֵȡƽ��
           threshold_diff_ac = threshold_diff_ac_sum/SUM_DIFF_AC_BUF_L;
              

           HR_diff_instant = (float)Fs*60/diff_window_move_count;   // ����HR
           HR_diff_average = Moving_Average_float(HR_diff_instant,HR_diff_average_prev,HR_diff_buf,HR_BUF_L);  // ��HR_instant����MA�˲���HR_average Ϊfloat����

           if ((HR_diff_average-HR_diff_average_prev)<2 &&(HR_diff_average-HR_diff_average_prev)>-2 )
           {
             HR_diff = (int)(HR_diff_average + 0.5);
/////////////////////////////////////////////////////////////////////////////////             
             if(HR_diff_count < HR_BUF_L)   // �������ЧHR_diff�Ĵ������м���������Ϊ��ʼ���ʱ����OLED�������ݣ�
             {
               HR_diff_count++;
             }
             else if(HR_diff_count == HR_BUF_L)  //��HR_diff_count����HR_BUF_L��ʼ����OLED��HR
             {
               HR_OLEDflag = 1;
             }
////////////////////////////////////////////////////////////////////////////////             
           }
           HR_diff_average_prev = HR_diff_average;
         }
         diff_window_move_count = 0;  // ���ѵ���ֵʱ��diff_window_move_count��ʼ��Ϊ0�����¼���   
       }
    }
  return;  
}

/**********************************************************************************************************
* Init_KEY_Interrupt
* P1_6--KEY
**********************************************************************************************************/
void Init_KEY_Interrupt (void)
{
    P1DIR &= ~BIT6;
    P1REN |= BIT6;              // Enable P1.6 internal resistance
    P1OUT |= BIT6;              // Set P1.6 as pull-Up resistance
    P1IES |= BIT6;              // P1.6 Hi/Lo edge �½����ж�
    P1IFG &= ~BIT6;             // P1.6 IFG cleared
    P1IE |= BIT6;               // P1.6 interrupt enabled
}


void SendRedAndIRToCC2530(uint32 REDdata,uint32 IRdata,uint16 SpO2_temp,uint16 HR_temp)
{
  uint8 temp = writeNum*8;
  bufferSendToCC2530[temp++] = (REDdata & 0xFF);
  bufferSendToCC2530[temp++] = (REDdata & 0xFF00) >> 8;
  bufferSendToCC2530[temp++] = (REDdata & 0xFF0000) >> 16;
  bufferSendToCC2530[temp++] = (REDdata & 0xFF000000) >> 24;
  bufferSendToCC2530[temp++] = (IRdata & 0xFF);
  bufferSendToCC2530[temp++] = (IRdata & 0xFF00) >> 8;
  bufferSendToCC2530[temp++] = (IRdata & 0xFF0000) >> 16;
  bufferSendToCC2530[temp++] = (IRdata & 0xFF000000) >> 24;
  writeNum++;
  if(writeNum == 8)
  {
    bufferSendToCC2530[temp++] = (SpO2_temp & 0xFF);
    bufferSendToCC2530[temp++] = (SpO2_temp & 0xFF00) >> 8;
    bufferSendToCC2530[temp++] = (HR_temp & 0xFF);
    bufferSendToCC2530[temp++] = (HR_temp & 0xFF00) >> 8;
    UART1_Send_Buffer(bufferSendToCC2530,68);
    writeNum = 0;
  }
}


/*********************************************************************
 * @fn      GenericApp_GetWriteName
 *
 * @brief   Get the RTC time and make file name.
 *
 * @param   char *
 *          0:D/20xx-xx-xx xx-xx-xx x.txt + \0 = 30Byte
 *
 * @return  
 *
 */
void GenericApp_GetWriteName(void)
{
  RTCStruct_t RTCStruct;
  HalRTCGetOrSetFull(RTC_DS1302_GET,&RTCStruct);

  // Make file name
  fileName[0] = '0';
  fileName[1] = ':';
  fileName[2] = 'S';
  fileName[3] = '/';
  fileName[4] = '2';
  fileName[5] = '0';
  fileName[6] = RTCStruct.year/10 + '0';
  fileName[7] = RTCStruct.year%10 + '0';
  fileName[8] = '-';
  fileName[9] = RTCStruct.month/10 + '0';
  fileName[10] = RTCStruct.month%10 + '0';
  fileName[11] = '-';
  fileName[12] = RTCStruct.date/10 + '0';
  fileName[13] = RTCStruct.date%10 + '0';
  fileName[14] = ' ';
  fileName[15] = RTCStruct.hour/10 + '0';
  fileName[16] = RTCStruct.hour%10 + '0';
  fileName[17] = '-';
  fileName[18] = RTCStruct.min/10 + '0';
  fileName[19] = RTCStruct.min%10 + '0';
  fileName[20] = '-';
  fileName[21] = RTCStruct.sec/10 + '0';
  fileName[22] = RTCStruct.sec%10 + '0';
  fileName[23] = ' ';  
  fileName[24] = RTCStruct.week + '0';
  fileName[25] = '.';
  fileName[26] = 't';
  fileName[27] = 'x';
  fileName[28] = 't';
  fileName[29] = '\0';
}


void delay(unsigned long num)
{
  unsigned long i;
  for(i=0;i<num;i++){ _NOP();}
}

/*********************************************************************
 * @fn      GenericApp_OpenDir
 *
 * @brief   �ҵ�ָ��Ŀ¼���ļ�
 *
 * @param  
 *
 * @return  true is over,flase is not over
 *
 */
bool GenericApp_OpenDir(void)
{
  uint8 res = 0;
  DIR *fddir = 0;         // Ŀ¼
  FILINFO *finfo = 0;     // �ļ���Ϣ
  uint8 *fn = 0;          // ���ļ���
  
  // initilize pathname
  strcpy(pathname,"0:S/");
  
  // �����ڴ�
  fddir = (DIR *)malloc(sizeof(DIR));
  finfo = (FILINFO *)malloc(sizeof(FILINFO));
  if(fddir == NULL || finfo == NULL)
  {
    if(fddir != NULL)
      free(fddir);
    if(finfo != NULL)
      free(finfo);
    return FALSE;
  }
  
  finfo->lfsize = 28 + 1;
  finfo->lfname = malloc(finfo->lfsize);
  if(finfo->lfname == NULL)
  {
    free(finfo->lfname);
    free(fddir);
    free(finfo);
    return FALSE;
  }
  
  // ��ԴĿ¼
  res = f_opendir(fddir,"0:S");

  if(res == 0)  // ��Ŀ¼�ɹ�
  {
    while(res == 0)
    {
      res = f_readdir(fddir,finfo);   //��ȡĿ¼�µ�һ���ļ�
     
      if(res != FR_OK || finfo->fname[0] == 0) // ������߶����˽�β
      {
        free(finfo->lfname);
        free(fddir);
        free(finfo);
        return FALSE;
      }
      
      if(finfo->fname[0] == '.') // �����ϲ��ļ�
        continue;
      
      /* �����ļ�,�����ַ��� */
      fn = (uint8 *)(*finfo->lfname?finfo->lfname:finfo->fname);
      strcat((char *)pathname,(char *)fn);
      
      break;
    }
  }
  
  free(finfo->lfname);
  free(fddir);
  free(finfo);
  return TRUE;  
}


/*********************************************************************
 * @fn      SyncData
 *
 * @brief   Sync data һ��ͬ��һ���ļ�
 *
 * @param  
 *
 * @return  
 *
 */
void SyncData(void)
{
  uint8 *dataSendBuffer;
  uint8 *dataSendBufferTemp;
  uint8 i,j;
  uint16 flagNum;
  if(GenericApp_OpenDir() == TRUE) // Ŀ¼�����ļ��������ļ�
  {
    dataSendBuffer = malloc(sizeof(uint8)*512);
    dataSendBufferTemp = malloc(sizeof(uint8)*68);
    if(dataSendBuffer == NULL)
    {
      SpO2SystemStatus = SpO2_ONLINE_IDLE; // ͬ������
      return;
    }
    if(dataSendBufferTemp == NULL)
    {
      free(dataSendBuffer);
      SpO2SystemStatus = SpO2_ONLINE_IDLE; // ͬ������
      return;
    }
    f_open(file,(char *)pathname,FA_OPEN_EXISTING | FA_READ); //  ���ļ�
    f_read(file,dataSendBuffer,SPO2_WAVEFORM_READ_ONE_TIME,&br);
    br = br/SPO2_WAVEFORM_SEND_ONE_TIME;
    j = 0;
    while(br != 0)
    {
      while(br != 0)
      {
        flagNum = j*64;
        for(i = 0; i < 64; ++i)
        {
          dataSendBufferTemp[i] = dataSendBuffer[flagNum+i];
        }
        dataSendBufferTemp[i++] = 0x62;
        dataSendBufferTemp[i++] = 0x00;
        dataSendBufferTemp[i++] = 0x4B;
        dataSendBufferTemp[i++] = 0x00;
        UART1_Send_Buffer(dataSendBufferTemp,68);
        br--;
        ++j;
        halMcuWaitUs(50000);
      }
      f_read(file,dataSendBuffer,SPO2_WAVEFORM_READ_ONE_TIME,&br);
      br = br/SPO2_WAVEFORM_SEND_ONE_TIME;
      j = 0;
    }
    
    // ������� �ر�ɾ���ļ�
    f_close(file);
    f_unlink((char *)pathname);
    free(dataSendBuffer);
    free(dataSendBufferTemp);
  }
  else // û���ļ��ˣ�����ͬ����������
  {
    UART1_Send_Buffer(bufferSendToCC2530,10);   
    SpO2SystemStatus = SpO2_ONLINE_IDLE; // ͬ������
    Show_Wait_Symbol("On_IDLE ");
  }  
}
