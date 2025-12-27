/*---------------------------------------------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 * Copyright(c) 2023 Nuvoton
 *--------------------------------------------------------------------------------------------------------*/

#include "numicro_8051.h"
#include <intrins.h>
#include <math.h>

/* =====================================================================================
 * PIN DEFINITIONS
 * ===================================================================================*/
#define TM_CLK   P00          // TM1637 CLK
#define TM_DIO   P01          // TM1637 DIO

/* =====================================================================================
 * GLOBAL VARIABLES
 * ===================================================================================*/
uint16_t ADCdataAIN;

float v_out = 0.0;
float i_rms = 0.0;
float v_in  = 0.0;
float error = 0.0;
float ref   = 0.0;
bit relay_flag = 0;
int start_flag = 0;

int target = 230;

/* --- Timer control flags --- */
volatile bit display_update_flag = 0;
volatile unsigned int ms_counter = 0;
volatile unsigned int loop_flag  = 0;

unsigned char counter_display = 0;

/* =====================================================================================
 * ADC CONFIGURATION
 * ===================================================================================*/
#define ADC_MAX        4095.0
#define ADC_REF_VOLT   3.3
#define ADC_OFFSET     1.640
#define GAIN_FACTOR    0.000202020
#define SAMPLE_COUNT   202

#define CUR_ADC_OFFSET  1.641
#define BURDEN_GAIN     33.0
#define CURRENT_SAMPLES 200

/* =====================================================================================
 * TM1637 SEGMENTS
 * ===================================================================================*/
#define SEG_BLANK  0x00
#define SEG_I      0x06
#define SEG_O      0x3F
#define SEG_P      0x73
#define SEG_A      0x77
#define SEG_DP     0x80

const unsigned char segCode[10] =
{
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

/* =====================================================================================
 * FUNCTION PROTOTYPES
 * ===================================================================================*/
void Delay_us(unsigned int us);

int   adc_data(void);
float ac_voltage_rms(void);
float ac_voltage_rms_input(void);
float ac_current_rms(void);

void TM1637_Start(void);
void TM1637_Stop(void);
void TM1637_WriteByte(unsigned char dat);
void TM1637_DisplayRaw(unsigned char d0,
                       unsigned char d1,
                       unsigned char d2,
                       unsigned char d3);
void TM1637_DisplayString(const char *str);
void TM1637_DisplayCurrent(float current);
void TM1637_DisplayNumber(unsigned int num);

void Timer1_Init_20ms(void);

/* =====================================================================================
 * MICROSECOND DELAY
 * ===================================================================================*/
void Delay_us(unsigned int us)
{
    while (us--)
    {
        _nop_();
        _nop_();
        _nop_();
        _nop_();
    }
}

/* =====================================================================================
 * ADC FUNCTIONS
 * ===================================================================================*/
int adc_data(void)
{
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;

    while (!(ADCCON0 & SET_BIT7));

    ADCdataAIN  = (ADCRH << 4);
    ADCdataAIN |= (ADCRL & 0x0F);

    return ADCdataAIN;
}

float ac_voltage_rms(void)
{
	  
    unsigned int i;
    float sum_squares = 0.0;
    float adc_val, v_adc, ac_signal, actual_voltage;

    ENABLE_ADC_AIN4;
    ENABLE_ADC;

    ref = 0;
     
	  EA = 0;
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = adc_data();
        v_adc   = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        if (v_adc > ref)
            ref = v_adc;

        ac_signal      = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;
				
				

        sum_squares += actual_voltage * actual_voltage;
    }
		EA = 1; 
		
    DISABLE_ADC;

    //printf("The ref = %0.3f", ref);
    ref = 0;

    return sqrt(sum_squares / SAMPLE_COUNT);
}

float ac_voltage_rms_input(void)
{
    unsigned int i;
    float sum_squares = 0.0;
    float adc_val, v_adc, ac_signal, actual_voltage;

    ENABLE_ADC_AIN1;
    ENABLE_ADC;

    ref = 0;
     EA = 0;
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = adc_data();
        v_adc   = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        if (v_adc > ref)
            ref = v_adc;

        ac_signal      = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;

        sum_squares += actual_voltage * actual_voltage;
    }
    EA = 1; 
    DISABLE_ADC;

    return sqrt(sum_squares / SAMPLE_COUNT);
}

float ac_current_rms(void)
{
    unsigned int i;
    float sum_squares = 0.0;
    float adc_val, v_adc, v_ac, i_secondary, i_primary;

    ENABLE_ADC_AIN5;
    ENABLE_ADC;

    for (i = 0; i < CURRENT_SAMPLES; i++)
    {
        adc_val = adc_data();
        v_adc   = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        v_ac        = v_adc - CUR_ADC_OFFSET;
			
        i_secondary = v_ac / BURDEN_GAIN;
        i_primary   = i_secondary * 2500;
       
        sum_squares += i_primary * i_primary;

        Delay_us(150);
    }

    DISABLE_ADC;

    return sqrt(sum_squares / CURRENT_SAMPLES);
}

/* =====================================================================================
 * TM1637 LOW LEVEL
 * ===================================================================================*/
void TM1637_Start(void)
{
    TM_CLK = 1;
    TM_DIO = 1;
    Delay_us(5);

    TM_DIO = 0;
    Delay_us(5);

    TM_CLK = 0;
}

void TM1637_Stop(void)
{
    TM_CLK = 0;
    TM_DIO = 0;
    Delay_us(5);

    TM_CLK = 1;
    Delay_us(5);

    TM_DIO = 1;
}

void TM1637_WriteByte(unsigned char dat)
{
    unsigned char i;

    for (i = 0; i < 8; i++)
    {
        TM_CLK = 0;
        TM_DIO = dat & 0x01;
        Delay_us(2);

        TM_CLK = 1;
        Delay_us(2);

        dat >>= 1;
    }

    TM_CLK = 0;
    TM_DIO = 1;
    Delay_us(2);

    TM_CLK = 1;
    Delay_us(2);

    TM_CLK = 0;
}

/* =====================================================================================
 * TM1637 DISPLAY
 * ===================================================================================*/
void TM1637_DisplayRaw(unsigned char d0,
                       unsigned char d1,
                       unsigned char d2,
                       unsigned char d3)
{
    TM1637_Start();
    TM1637_WriteByte(0x40);
    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0xC0);

    TM1637_WriteByte(d0);
    TM1637_WriteByte(d1);
    TM1637_WriteByte(d2);
    TM1637_WriteByte(d3);

    TM1637_Stop();

    TM1637_Start();
    TM1637_WriteByte(0x8F);
    TM1637_Stop();
}

void TM1637_DisplayString(const char *str)
{
    unsigned char d[4] = {SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK};

    if (str[0] == 'I') d[0] = SEG_I;
    if (str[0] == 'O') d[0] = SEG_O;

    if (str[1] == 'P') d[1] = SEG_P;
    if (str[1] == 'A') d[1] = SEG_A;

    TM1637_DisplayRaw(d[0], d[1], d[2], d[3]);
}

void TM1637_DisplayCurrent(float current)
{
    unsigned int int_part;
    unsigned int dec_part;
    unsigned char d0, d1, d2, d3;

    if (current < 0)    current = 0;
    if (current > 99.99) current = 99.99;

    int_part = (unsigned int)current;
    dec_part = (unsigned int)((current - int_part) * 100);

    d0 = segCode[int_part / 10];
    d1 = segCode[int_part % 10];
    d2 = segCode[dec_part / 10];
    d3 = segCode[dec_part % 10];

    d1 |= SEG_DP;

    TM1637_DisplayRaw(d0, d1, d2, d3);
}

void TM1637_DisplayNumber(unsigned int num)
{
    unsigned char d[4];

    if (num > 9999)
        num = 9999;

    d[0] = segCode[num / 1000];
    d[1] = segCode[(num / 100) % 10];
    d[2] = segCode[(num / 10) % 10];
    d[3] = segCode[num % 10];

    TM1637_DisplayRaw(d[0], d[1], d[2], d[3]);
}

/* =====================================================================================
 * TIMER-1 (20 ms)
 * ===================================================================================*/
void Timer1_Init_20ms(void)
{
    TMOD &= 0x0F;
    TMOD |= 0x10;

    TH1 = 0x63;
    TL1 = 0xC0;

    TF1 = 0;
    ET1 = 1;
    EA  = 1;
    TR1 = 1;
}

void Timer1_Enable(void)
{
    TF1 = 0;   // Clear overflow
    ET1 = 1;   // Enable Timer-1 interrupt
    TR1 = 1;   // Start timer
}

void Timer1_Disable(void)
{
    ET1 = 0;   // Disable Timer-1 interrupt
    TR1 = 0;   // Stop timer
}

void Timer1_ISR(void) interrupt 3
{
    TF1 = 0;

    ms_counter++;

    if (ms_counter >=40)
    {
			  ++start_flag;
        ms_counter = 0;
        display_update_flag = 1;
    }
}

/* =====================================================================================
 * MAIN
 * ===================================================================================*/
void main(void)
{
    MODIFY_HIRC(HIRC_24);
    //Enable_UART0_VCOM_printf_24M_115200();

    P01_QUASI_MODE;
    P00_QUASI_MODE;
    P12_PUSHPULL_MODE;
    P13_PUSHPULL_MODE;
	  P17_PUSHPULL_MODE;

    Timer1_Init_20ms();

    while (1)
    {
			  
        v_out = (ac_voltage_rms()- 10);
        i_rms = ac_current_rms();
        v_in  = (ac_voltage_rms_input() - 15);
			 
			 
			if(v_out < 100 || v_out > 1000) {
				v_out = 0;
			}
			if(v_in < 100 || v_in > 1000) {
				v_in = 0;
			}
			 if(i_rms < 0.6) {
					i_rms = 0;
				}

        error = v_out - target;

        if (error > 6)
        {
					//Timer1_Disable();
            P12 = 0;
            P13 = 1;
        }
        else if (error < -6)
        {
					//Timer1_Disable();
            P13 = 0;
            P12 = 1;
        }
        else
        {
					  //Timer1_Enable();
            P12 = 0;
            P13 = 0;
        }
				
				if(v_out > 200  && v_out < 350 && v_in < 280 && v_in > 50 && i_rms < 2 && start_flag > 3) {
					 relay_flag = 1;
				}
				else {
				relay_flag =0;
				}
				
				
				if(relay_flag == 1) {
					P17 = 1;
				}
				else {
					P17 = 0;
				}


        if (display_update_flag)
        {
            display_update_flag = 0;

            switch (counter_display)
            {
                case 0: TM1637_DisplayString("IP"); break;
                case 1: TM1637_DisplayNumber((unsigned int)v_in); break;
                case 2: TM1637_DisplayString("OP"); break;
                case 3: TM1637_DisplayNumber((unsigned int)v_out); break;
                case 4: TM1637_DisplayString("OA"); break;
                case 5: TM1637_DisplayCurrent(i_rms); break;
            }

            counter_display++;
            if (counter_display > 5)
                counter_display = 0;
        }
    }
}

