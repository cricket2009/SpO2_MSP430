

//#include "Spo2_Functions.h"

#include "msp430f5528.h"
//#define MA_N  50

/*
===========Moving_Average_unsig=======================================
���޷��ų��������ݽ���MA_N�׻����˲�
*/
unsigned long Moving_Average_unsig(unsigned long PPG_rawdata,unsigned long MAF_data,unsigned long *p,int MA_NN)
{   
    int i;
    unsigned long New_MAF_data,data_out;

//===========FIFO ��λ�������һ��λ�������Ƴ������һ��λ������������====================//    
    data_out = *(p+0);
    for (i = 0; i < MA_NN-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+MA_NN-1)= PPG_rawdata;
    
    New_MAF_data = MAF_data +PPG_rawdata/MA_NN - data_out/MA_NN;  //�����˲��� MAF_data Ϊ�ϴδ������ý��
    
    return New_MAF_data;


}




    
/*
===========Moving_Average_unsig=======================================
���޷����������ݽ���MA_N�׻����˲�
*/
unsigned int Moving_Average_unsigint(unsigned int PPG_rawdata,unsigned int MAF_data,unsigned int *p,int MA_NN)
{   
    int i;
    unsigned int New_MAF_data,data_out;

//===========FIFO ��λ�������һ��λ�������Ƴ������һ��λ������������====================//    
    data_out = *(p+0);
    for (i = 0; i < MA_NN-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+MA_NN-1)= PPG_rawdata;
    
    New_MAF_data = MAF_data +PPG_rawdata/MA_NN - data_out/MA_NN;  //�����˲��� MAF_data Ϊ�ϴδ������ý��
    
    return New_MAF_data;


}





/*
===========Moving_Average_sig=======================================
���з����������ݽ���MA_N�׻����˲�
*/
signed int Moving_Average_sig(signed int PPG_rawdata,signed int MAF_data,signed int *p,int MA_NN)
{   
    int i;
    signed int New_MAF_data,data_out;
   
    data_out = *(p+0);
    for (i = 0; i < MA_NN-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+MA_NN-1)= PPG_rawdata;
    
    New_MAF_data = MAF_data +PPG_rawdata/MA_NN - data_out/MA_NN;
    
    return New_MAF_data;
}

/*
===========Moving_Average_float=======================================
�Ը������ݽ���MA_N�׻����˲�
*/
float Moving_Average_float(float PPG_rawdata,float MAF_data,float *p,int MA_NN)
{   
    int i;
    float New_MAF_data;
    float data_out;
   
    data_out = *(p+0);
    for (i = 0; i < MA_NN-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+MA_NN-1)= PPG_rawdata;
    
    New_MAF_data = MAF_data +(float)(PPG_rawdata - data_out)/MA_NN;
    
    return New_MAF_data;
}






unsigned long DC_track(unsigned long PPG_datain, unsigned long dc_offset)
{
  unsigned long dc_offset_out;
  
  dc_offset_out = dc_offset + (PPG_datain >>11) - (dc_offset >> 11);
  
  return dc_offset_out;
}


/*
===========fifo_move=======================================
�������FIFO��λ������������ҪΪsigned int
*/
void fifo_move(signed int newdata, signed int *p, int L)   
{
    int i;
   // signed int data_out;
   
   // data_out = *(p+0);
    for (i = 0; i < L-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+L-1)= newdata;
}



/*
===========rms_fifo=======================================
�������FIFO��λ����ˮ��ʽ�������������ƽ����
*/
#if 0
unsigned long rms_fifo(signed int led_ac,unsigned long prev_rms,signed int *p,int rms_N)
{   
    int i;
    unsigned long rms_new,temp1,temp2;
    signed int data_out;
//===========FIFO ��λ�������һ��λ�������Ƴ������һ��λ������������====================//    
    data_out = *(p+0);
    for (i = 0; i < rms_N-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+rms_N-1)= led_ac;
    
    MPY = abs(led_ac);   //OP1���˼�����
    OP2  = abs(led_ac);
    
    temp1 = RESHI;
    temp1 = (temp1 << 16) | RESLO;
    RESHI =0;  // ����Ĵ�������
    RESLO = 0;
    
    MPY = abs(data_out);
    OP2 = abs(data_out);
    
    temp2 = RESHI;
    temp2 = (temp2 <<16) | RESLO;
    
    rms_new = prev_rms + (unsigned long)temp1 - (unsigned long)temp2;  //ƽ����
    
    return rms_new;


}
#endif



/*
===========sum_dc=======================================
�������FIFO��λ����ˮ��ʽ������������֮��
*/

unsigned long sum_dc(unsigned long led_dc,unsigned long prev_sum_dc,unsigned long *p,int rms_N)
{   
    int i;
    unsigned long sum_dc_new,data_out;

//===========FIFO ��λ�������һ��λ�������Ƴ������һ��λ������������====================//    
    data_out = *(p+0);
    for (i = 0; i < rms_N-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+rms_N-1)= led_dc;
    
    sum_dc_new = prev_sum_dc +led_dc - data_out;  //��
    
    return sum_dc_new;


}



/*
===========sum_dc=======================================
�������FIFO��λ����ˮ��ʽ������������֮��
*/

unsigned long sum_AC(unsigned int led_ac,unsigned long prev_sum_ac,unsigned int *p,int rms_N)
{   
    int i;
    unsigned long sum_ac_new;
    unsigned int data_out;

//===========FIFO ��λ�������һ��λ�������Ƴ������һ��λ������������====================//    
    data_out = *(p+0);
    for (i = 0; i < rms_N-1; i++)
    {
     
      *(p+i) = *(p+i+1);
      
    }
    
    *(p+rms_N-1)= led_ac;
    
    sum_ac_new = (unsigned long) prev_sum_ac  - data_out + led_ac;  //��
    
    return sum_ac_new;


}






















//int fifo_average(int datain, int average_prev, int *p, int M)
//{
//  int i, data_out;
//  data_out = *(p+0);
//  
//  for(i=0;i<M-1;i++)
//  {
//    *(p+i) = *(p+i+1);
//  }
//  
//  
//}





//
//float max(float *p,int spo2_N)
//{
//  float max_value;
//  
//  max_value = *(p+0);
//  
//  for (int i=1; i<spo2_N;i++)
//  {
//    if (*(p+i) > max_value)
//    {max_value = *(p+i);}    
//  }
//  
//  return max_value;
//  
//}
//
//
//float min(float *p,int spo2_N)
//{
//  float min_value;
//  
//  min_value = *(p+0);
//  
//  for (int i=1; i<spo2_N;i++)
//  {
//    if (*(p+i) < min_value)
//    {min_value = *(p+i);}    
//  }
//  
//  return min_value;
//  
//}




















