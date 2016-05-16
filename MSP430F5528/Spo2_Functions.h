


#ifndef SPO2_FUNCTIONS_H_
#define SPO2_FUNCTIONS_H_




//static unsigned long MA_buf[MA_N];

unsigned long Moving_Average_unsig(unsigned long PPG_rawdata,unsigned long MAF_data,unsigned long *p,int MA_NN);
signed int Moving_Average_sig(signed int PPG_rawdata,signed int MAF_data,signed int *p,int MA_NN);

unsigned int Moving_Average_unsigint(unsigned int PPG_rawdata,unsigned int MAF_data,unsigned int *p,int MA_NN);

void fifo_move(signed int newdata, signed int *p, int L);
int Is_mid_peak(signed int *p, int L);
unsigned long DC_track(unsigned long PPG_datain, unsigned long dc_offset);
unsigned long sum_AC(unsigned int led_ac,unsigned long prev_sum_ac,unsigned int *p,int rms_N);
unsigned long sum_dc(unsigned long led_dc,unsigned long prev_sum_dc,unsigned long *p,int rms_N);
unsigned long rms_fifo(signed int led_ac,unsigned long prev_rms,signed int *p,int rms_N);
float max(float *p,int spo2_N);
float min(float *p,int spo2_N);

float Moving_Average_float(float PPG_rawdata,float MAF_data,float *p,int MA_NN);


unsigned long dc_offset_Led1 = 0;
unsigned long dc_offset_Led2 = 0;





#endif