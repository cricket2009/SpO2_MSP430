

//#include "device.h"
#include "types.h"               //Basic Type declarations
#include "AFE44x0_main.h"
#include "AFE44x0.h"
#include "msp430f5528.h"
#include "OLED_FRONT.h"
#include "hal_type.h"

/**********************************************************************************************************
* Set_UCB1_SPI													                                          *
**********************************************************************************************************/
void Set_UCA0_SPI(void)
{ 
    P3SEL |= BIT3;  			        // Set SPI peripheral bits p3_3:SIMO,P2_7:SCLK
    P2SEL |= BIT7;
    
    P3DIR |= BIT3;			        // SIMO as output
    P2DIR |= BIT7;                              // SCLK as output
    
//    P3DIR |= BIT1+BIT2+BIT4;                    // IO output,P3_1: OLED_RES#, P3_2: OLED_CS#, P3_4:OLED_D/C#
    P3DIR |= BIT2;                              // IO output, P3_2 USD_CS
    
///////////////////////////////////////////////////////    
    P2DIR |= BIT5;                              // OLED_CS# output
    
    P2OUT |= BIT5;                              // Set OLED_CS# high
//////////////////////////////////////////////////////    
    P3OUT |= BIT2;				
    UCA0CTL1 |= UCSWRST;               		// Enable SW reset
    UCA0CTL0 |= UCMSB+UCCKPH+UCMST+UCSYNC;	// [b0]   1 -  Synchronous mode
                                                // [b2-1] 00-  3-pin SPI
                                                // [b3]   1 -  Master mode
                                                // [b4]   0 - 8-bit data
                                                // [b5]   1 - MSB first
                                                // [b6]   0 - Clock polarity high.
                                                // [b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.
  
    UCA0CTL1 |= UCSSEL_2;               	// select SMCLK as clock source 
    UCA0BR0 = 0x02;                             // 4 MHz
    UCA0BR1 = 0;                                //
    UCA0CTL1 &= ~UCSWRST;              		// Clear SW reset, resume operation
    UCA0IE =0x0;
}



/******************************************************************************
* Name: write_oled_command(unsigned char ucCmd)
* Function: write command ucCmd to oled
******************************************************************************/
void write_oled_command(unsigned char ucCmd)
{  
//   P3OUT &= ~BIT2;  // Set OLED_CS# LOW,enable   
//   P3OUT &= ~BIT4;  // OLED_D/C# LOW, command
   P2OUT &= ~BIT5;    // Set OLED_CS# LOW,enable  
   P2OUT &= ~BIT6;    // OLED_D/C# LOW, command
   

   UCA0TXBUF = ucCmd;       // Send the byte to the TX Buffer
   while ( (UCA0STAT & UCBUSY) );   //等待发送完毕		
 
//   P3OUT |= BIT2;  //  OLED_CS# high
   P2OUT |= BIT5;    // Set OLED_CS# high  
   
   
}


/******************************************************************************
* Name: write_oled_data(unsigned char ucData)
* Function: write ucdata to oled
******************************************************************************/
void write_oled_data(unsigned char ucData)
{  
//   P3OUT &= ~BIT2;  // Set OLED_CS# LOW,enable   
//   P3OUT |= BIT4;  // OLED_D/C# LOW, data
   P2OUT &= ~BIT5;    // Set OLED_CS# LOW,enable  
   P2OUT |= BIT6;    // OLED_D/C# high, data     
   
   UCA0TXBUF = ucData;       // Send the byte to the TX Buffer
   while ( (UCA0STAT & UCBUSY) );   //等待发送完毕		
 
//   P3OUT |= BIT2;  //  OLED_CS# high
   P2OUT |= BIT5;    // Set OLED_CS# high  
}


void delay(unsigned long num)
{
  unsigned long i;
  for(i=0;i<num;i++){ _NOP();}
}




/*OLED的显存
存放格式如下.
[0]0 1 2 3 ... 127	
[1]0 1 2 3 ... 127	
[2]0 1 2 3 ... 127	
[3]0 1 2 3 ... 127 
[4]0 1 2 3 ... 127
[5]0 1 2 3 ... 127
[6]0 1 2 3 ... 127
[7]0 1 2 3 ... 127
*/
UCHAR OLED_GRAM[128][8]; //横坐标 128，纵坐标8page

/*
 * 函数名：OLED_Refresh_Gram
 * 描述  ：更新显示
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void OLED_Refresh_Gram(void)
{
	UCHAR i,n;	
			    
	for(i=0;i<8;i++)  
	{  
//        write_oled_command(0x00);    //set lower column address
//        write_oled_command(0x10);    //set higher column address
//        write_oled_command(0xB0+i);    //set page address  


       //  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

       //  UCB0CTL1 |= UCTXSTT;                    // I2C start 
		
       //  write_oled_data(0x40);
          
         for(n=0;n<128;n++)
	{
    	     write_oled_data(OLED_GRAM[n][i]);	
	}
      // while((UCB0IFG & UCTXIFG)!= 2);         // wait for data has been transffered from the buffer to shift register (UCTXIFG =1)
      //  UCB0CTL1 |= UCTXSTP;                    // stop
	
	}   
}


void OLED_Refresh_Gram_part(UCHAR x1, UCHAR y1, UCHAR x2, UCHAR y2)
{
  int n1,n2,i,n;
  n1 = y1/8;
  n2 = y2/8;
  
  
  	   write_oled_command(0x21);    //Setup column start and end address
	   write_oled_command(x1);	//A[6:0] : Column start address, range : 0-127d, (RESET=0d)
	   write_oled_command(x2);    //B[6:0]: Column end address, range : 0-127d, (RESET =127d)

	   write_oled_command(0x22);    //Setup page start and end address
	   write_oled_command(n1);	//A[2:0] : Page start Address, range : 0-7d, (RESET=0d)
	   write_oled_command(n2);    //B[2:0] : Page end Address, range : 0-7d, (RESET = 7d)
  
  for(i = n1;i<=n2;i++)
  {          
         for(n=x1;n<=x2;n++)
	{
    	     write_oled_data(OLED_GRAM[n][i]);	
	}
    }   
    
}
  
  

/*
 * 函数名：GRAM_Clear
 * 描述  ：清除显示
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void GRAM_Clear(void)  
{  
	UCHAR i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
}

/*
 * 函数名：OLED_Clear
 * 描述  ：清除OLED显示
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void OLED_Clear(void)  
{  
     GRAM_Clear(); 
	OLED_Refresh_Gram();//更新显示
}

/*
 * 函数名：OLED_DrawPoint
 * 描述  ：画点
 * 输入  ：x:0~127
           y:0~31
           t:1 填充, 0 清空
 * 输出  : 无
 * 调用  ：外部调用
*/				   
void OLED_DrawPoint(UCHAR x,UCHAR y,UCHAR t)
{
	UCHAR pos,bx,temp=0;
//	GRAM_Clear();
	if(x>127||y>63)return;//超出范围了.
	pos=y/8;
	bx=y%8;
	temp=1<<(bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

/*
 * 函数名：OLED_Fill
 * 描述  ：区域填充
 * 输入  ：x1,y1,x2,y2 填充区域的对角坐标
           确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=31
           dot:0,清空;1,填充	
 * 输出  : 无
 * 调用  ：外部调用
*/
void OLED_Fill(UCHAR x1,UCHAR y1,UCHAR x2,UCHAR y2,UCHAR dot)  
{  
	UCHAR x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}													    
	//OLED_Refresh_Gram();//更新显示
}

/*
 * 函数名：OLED_ShowChar
 * 描述  ：在指定位置显示一个字符,包括部分字符
 * 输入  ：x:0~127
           y:0~31
		   chr：要显示的字符
		   size:选择字体 32/16/12
		   mode:0,反白显示;1,正常显示 	
 * 输出  : 无
 * 调用  ：外部调用
*/
void OLED_ShowChar(UCHAR x,UCHAR y,UCHAR chr,UCHAR size,UCHAR mode)
{      			    
	UCHAR temp,t,t1;
	UCHAR y0=y;
	chr=chr-' ';//得到偏移后的值
	if(size == 32)
	{    
	    for(t=0;t<64;t++)
	    {   
			temp=oled_asc3_3216[chr][t];  //调用3216字体
	        for(t1=0;t1<8;t1++)
			{
				if(temp&0x80)OLED_DrawPoint(x,y,mode);
				else OLED_DrawPoint(x,y,!mode);
				temp<<=1;
				y++;
				if((y-y0)==size)
				{
					y=y0;
					x++;
					break;
				}
			}  	 
	    }	
	}
	else if(size == 12)
	{	   //单点模式			   
	    for(t=0;t<16;t++)
	    {   
			temp=oled_asc2_1206[chr][t];  //调用1206字体
			for(t1=0;t1<8;t1++)
			{
				if(temp&0x80)OLED_DrawPoint(x,y,mode);
				else OLED_DrawPoint(x,y,!mode);
				temp<<=1;
				y++;
				if((y-y0)==size)
				{
					y=y0;
					x++;
					break;
				}
			}  	 
	    }
	}
    else
    {
	    for(t=0;t<20;t++)
	    {   
			temp=oled_asc2_1608[chr][t];      //调用1608字体 ,每次Load一个byte，含自上到下，8个point,第char 行，第t列                         
	        for(t1=0;t1<8;t1++)
			{
				if(temp&0x80)OLED_DrawPoint(x,y,mode);
				else OLED_DrawPoint(x,y,!mode);
				temp<<=1;
				y++;
				if((y-y0)==size)
				{
					y=y0;
					x++;
					break;
				}
			}  	 
	    }
    }
    
}

/*
 * 函数名：OLED_ShowSymbol
 * 描述  ：在指定位置显示一个符号,包括部分符号
 * 输入  ：x:0~127
           y:0~31
		   sym：要显示的符号
		   mode:0,反白显示;1,正常显示 	
 * 输出  : 无
 * 调用  ：外部调用
*/
void OLED_ShowSymbol(UCHAR x,UCHAR y,UCHAR sym,UCHAR mode)
{
    UCHAR temp,t,t1;
	UCHAR y0=y;
	UCHAR x0=x;
	const unsigned char *cur_sym ;
	  //B0 B1 B2  B3  B4  B5  B6  B7
	  //B8 B9 B10 B11 B12 B13 B14 B15 
		   if(sym ==1)	//bt
		   {cur_sym = oled_bt_symbol ;}
		   else if(sym == 2)   //run
		   {cur_sym = oled_run_symbol ;}
		   else
		   {cur_sym = oled_stop_symbol;}

		  //先画第一行（每个byte高位到低位代表自上而下的8个点，第一行8个byte）
	       for(t=0;t<8;t++)
		    {   
			 temp=cur_sym[t];		 //调用1608字体 ,每次Load一个byte，含自上到下，8个point	                          
		        for(t1=0;t1<8;t1++)
				{
					if(temp&0x80)OLED_DrawPoint(x,y,mode);
					else OLED_DrawPoint(x,y,!mode);
					temp<<=1;
					y++;
					if((y-y0)==8)
					{
						y=y0;
						x++;
						break;
					}
				}  	 
		    }

			//将座标挪向第二行（8bit）
		    x = x0;
			y0 = y0+8;
			y = y0;
			//再画第二行
		   for(t=8;t<16;t++)
		    {   
			 temp=cur_sym[t];		 //调用1608字体 ,每次Load一个byte，含自上到下，8个point	                          
		        for(t1=0;t1<8;t1++)
				{
					if(temp&0x80)OLED_DrawPoint(x,y,mode);
					else OLED_DrawPoint(x,y,!mode);
					temp<<=1;
					y++;
					if((y-y0)==8)
					{
						y=y0;
						x++;
						break;
					}
				}  	 
		    }
		
}

/*
 * 函数名：oled_pow
 * 描述  ：m^n函数
 * 输入  ： 	
 * 输出  : 无
 * 调用  ：外部调用
*/
ULONG oled_pow(UCHAR m,UCHAR n)
{
       
	ULONG result=1;	 
	while(n--)result*=m;    
	return result;
}	

/*
 * 函数名：OLED_ShowNum
 * 描述  ：显示2个数字
 * 输入  ：x,y :起点坐标	
		   num:数值(0~4294967295);
		   len :数字的位数
		   size:字体大小 	
 * 输出  : 无
 * 调用  ：外部调用
*/	 		  
void OLED_ShowNum(UCHAR x,UCHAR y,int num,UCHAR len,UCHAR size)
{         	
	UCHAR t,temp;
	UCHAR enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
                
	}
     
} 

/*
 * 函数名：OLED_ShowString
 * 描述  ：显示字符串
 * 输入  ：x,y :起点坐标	
		   num:数值(0~4294967295);
		   size:字体大小(用16字体) 	
		   *p:字符串起始地址
 * 输出  : 无
 * 调用  ：外部调用
*/
void OLED_ShowString(UCHAR x,UCHAR y,UCHAR size,const UCHAR *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 52          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,size,1);
        if(size == 12)
          x+=8;
        else if(size == 16)
          x+=10;
        else
          x+=size/2;
        p++;
    }  
}


void OLED_SLEEP(UCHAR st)
{
   if(st) 
   {
     write_oled_command(0xAE);    //display off
     write_oled_command(0X8D);
     write_oled_command(0X10);
   }   
   else   
   {
     write_oled_command(0X8D);
     write_oled_command(0X14);
     write_oled_command(0xAF);    //display on
   }
}


void  SSD1306(void)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin   = OLED_SDA_GPIO_PIN;	   //PB0	LED11
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	  //内部上拉电阻
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
//	GPIO_Init(OLED_GPIO_I2C_PORT, &GPIO_InitStructure);
//	
//    GPIO_InitStructure.GPIO_Pin   = OLED_SCL_GPIO_PIN;		   //PB1 LED10
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	  //内部上拉电阻
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
//	GPIO_Init(OLED_GPIO_I2C_PORT, &GPIO_InitStructure);	
//	
//	GPIO_InitStructure.GPIO_Pin   = OLED_RST_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
//	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;	  //内部上拉电阻
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
//	GPIO_Init(OLED_GPIO_RES_PORT, &GPIO_InitStructure);	
//				   
//    RST_H;   
//	delay(20);
//   	RST_L;   
//    delay(20);
//    RST_H;   
//    delay(20);
       
//            P3DIR |= BIT1+BIT4;       // IO output,P3_1: OLED_RES#, P3_4:OLED_D/C#
            P1DIR |= BIT5;            // OLED_RES# output
            P2DIR |= BIT6;            // OLED_DC#, output
//            P3OUT &= ~BIT1;           //RES# low,reset oled, initialize
            P1OUT &= ~BIT5;           //RES# low,reset oled, initialize
            delay(10000);
            
//            P3OUT |= BIT1;            // RES# resume from reset
            P1OUT |= BIT5;            // RES# resume from reset
           // spi_init();
            Set_UCA0_SPI();
  
 /////////////////////////////////////////////////////

	   //基本设置  参考UG-28HSWEG02手册 p14
	   write_oled_command(0xAE);    //display off
           
           
 ///////address mode SETTING////////////////////
	   write_oled_command(0x20);    //Set Memory Addressing Mode   2 bytes cmd
           write_oled_command(0x00);    //A[1:0] = 00b, Horizontal Addressing Mode

	   write_oled_command(0x21);    //Setup column start and end address
	   write_oled_command(0x00);	//A[6:0] : Column start address, range : 0-127d, (RESET=0d)
	   write_oled_command(0x7f);    //B[6:0]: Column end address, range : 0-127d, (RESET =127d)

	   write_oled_command(0x22);    //Setup page start and end address
	   write_oled_command(0x00);	//A[2:0] : Page start Address, range : 0-7d, (RESET=0d)
	   write_oled_command(0x07);    //B[2:0] : Page end Address, range : 0-7d, (RESET = 7d)
           
           
           
           
 //Set Display Clock Divide Ratio/Oscillator Frequency          
	   write_oled_command(0xD5);    //contract control
           write_oled_command(0x80);    //

 //Set Multiplex Ratio
	   write_oled_command(0xA8); 	//Set Multiplex Ratio
	   write_oled_command(0x3F); 

	   write_oled_command(0xD3);    //Set Display Offset
	   write_oled_command(0x00);

	   write_oled_command(0x40);	//Set Display Start Line

	  // write_oled_command(0xA1);
	  // write_oled_command(0x10);

	   write_oled_command(0xA1);    //A1h, X[0]=1b: column address 127 is mapped to SEG0

	   write_oled_command(0xC8);    //C8h, X[3]=1b: remapped mode. Scan from COM[31] to COM0

	   write_oled_command(0xDA);
	   write_oled_command(0x12);

	   write_oled_command(0x81);
	   write_oled_command(0x5F);

	   write_oled_command(0xD9);
	   write_oled_command(0x82);

	   write_oled_command(0xDB);
	   write_oled_command(0x20);

	   write_oled_command(0xA4);

	   write_oled_command(0xA6);

/////////////////////////////

           OLED_Clear();
            
           write_oled_command(0x8D);
           write_oled_command(0x14);
            
          
	   write_oled_command(0xAF);

	   delay(10000);

}

//=================================================================

//===============================================================
//=======================================================
void oledinit(void)
{
	SSD1306(); 

}

void OLED_ShowWaitSymbol(UCHAR x,UCHAR y,UCHAR mode)
{
    UCHAR temp,t,t1;
    UCHAR y0=y;
    for(t=0;t<16;t++)
    {
      temp = oled_testing_point[t]; //调用16X8字体
      for(t1=0;t1<8;t1++)
      {
        if(temp&0x80)OLED_DrawPoint(x,y,mode);
        else OLED_DrawPoint(x,y,!mode);
        temp<<=1;
        y++;
        if((y-y0)==8)
        {
          y=y0;
          x++;
          break;
        }
      }  	
    }  
}

void OLED_ShowHeartSymbol(UCHAR x,UCHAR y,UCHAR mode,UCHAR HeartType)
{
    UCHAR temp,t,t1;
    UCHAR y0=y;
    if(HeartType == 2)
    {
        for(t=0;t<40;t++)
        {
          temp = oled_Big_Heart_point[t]; //调用大心
          for(t1=0;t1<8;t1++)
          {
            if(temp&0x80)OLED_DrawPoint(x,y,mode);
            else OLED_DrawPoint(x,y,!mode);
            temp<<=1;
            y++;
            if((y-y0)==16)
            {
              y=y0;
              x++;
              break;
            }
          }  	
        }
    }
    else
    {
        for(t=0;t<10;t++)
        {
          if(HeartType == 1)temp = oled_Small_Heart_point[t];//清空
          else temp = oled_Clear_Heart_point[t];//清空
          for(t1=0;t1<8;t1++)
          {
            if(temp&0x80)OLED_DrawPoint(x,y,mode);
            else OLED_DrawPoint(x,y,!mode);
            temp<<=1;
            y++;
            if((y-y0)==8)
            {
              y=y0;
              x++;
              break;
            }
          }  	
        }
    }
}


/***************************************************************************************************
 * @fn      HalOledShowPowerSymbol
 *
 * @brief   显示电量符号
 *
 * @param   
 *
 * @return  none
 ***************************************************************************************************/
void HalOledShowPowerSymbol(uint8 x,uint8 y,uint8 mode,uint8 power_num)
{
     uint8 temp,t,t1;
     uint8 y0=y;
     for(t=0;t<48;t++)
     {
       temp = oled_power_symbol[power_num][t]; //oled_power_symbol图标 power_num越小电量越低，10为最大100%,0最小0%
       for(t1=0;t1<8;t1++)
       {
         if(temp&0x80)OLED_DrawPoint(x,y,mode);
         else OLED_DrawPoint(x,y,!mode);
         temp<<=1;
         y++;
         if((y-y0)==12)
         {
           y=y0;
           x++;
           break;
         }
       }  	
     } 
}
















