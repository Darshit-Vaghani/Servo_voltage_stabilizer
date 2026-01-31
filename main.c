//---------Header File Section-------------------------------------------
#include "numicro_8051.h"
#include <intrins.h>
#include <math.h>
//----------------------------------------------------------------------------


//----------------------------Pin Defination-------------------------------------
#define TM_CLK P00 // TM1637 CLK
#define TM_DIO P01 // TM1637 DIO
#define SETTING P10  
#define UP P03
#define DOWN P02
//--------------------------------------------------------------------------------

//---------------------Protection Configuration-----------------------------------

#define OUTPUT_HIGH 310
#define OUTPUT_LOW  150
#define INPUT_HIGH  280
#define INPUT_LOW   150
#define OVERCURRENT  5
#define START_TIME_IN_SEC 3

//--------------------------------------------------------------------------------

//----------------------------UI Defination--------------------------------------
#define ui_normal 0
#define ui_password 1
#define ui_cal 2
#define ui_vol 3
#define ui_ol 4
#define ui_end 5
#define ui_cur 6
#define ui_exit 7
//-------------------------------------------------------------------------------


//-------------------------Charecter Defination----------------------------------
#define SEG_A 0x77
#define SEG_B 0x7C // b
#define SEG_C 0x39
#define SEG_D 0x5E // d
#define SEG_E 0x79
#define SEG_F 0x71
#define SEG_G 0x3D
#define SEG_H 0x76
#define SEG_I 0x06 // same as 1
#define SEG_J 0x1E
#define SEG_K 0x75 // approx
#define SEG_L 0x38
#define SEG_M 0x37 // approx
#define SEG_N 0x54
#define SEG_O 0x3F
#define SEG_P 0x73
#define SEG_Q 0x67 // approx
#define SEG_R 0x50
#define SEG_S 0x6D
#define SEG_T 0x78
#define SEG_U 0x3E
#define SEG_V 0x3E // same as U
#define SEG_W 0x2A // approx
#define SEG_X 0x76 // same as H
#define SEG_Y 0x6E
#define SEG_Z 0x5B // same as 2

#define SEG_DASH 0x40  // -
#define SEG_UNDER 0x08 // _
#define SEG_BLANK 0x00 // space
#define SEG_DP 0x80    // decimal point

const unsigned char segCode[10] =
    {
        0x3F, 0x06, 0x5B, 0x4F, 0x66,
        0x6D, 0x7D, 0x07, 0x7F, 0x6F};

//-----------------------------------------------------------------------------------


//--------------------Use for Charcter Collection------------------------------------
unsigned char TM1637_CharToSeg(char c)
{
    switch (c)
    {

    case 'A':
        return SEG_A;
    case 'B':
        return SEG_B;
    case 'C':
        return SEG_C;
    case 'D':
        return SEG_D;
    case 'E':
        return SEG_E;
    case 'F':
        return SEG_F;
    case 'G':
        return SEG_G;
    case 'H':
        return SEG_H;
    case 'I':
        return SEG_I;
    case 'L':
        return SEG_L;
    case 'N':
        return SEG_N;
    case 'O':
        return SEG_O;
    case 'P':
        return SEG_P;
    case 'R':
        return SEG_R;
    case 'S':
        return SEG_S;
    case 'U':
        return SEG_U;
    case 'V':
        return SEG_V;
    case 'Y':
        return SEG_Y;

    case ' ':
        return SEG_BLANK;

    default:
        return SEG_BLANK;
    }
}

//---------------------------------------------------------------------------------

//------------------------Global Variable Section----------------------------------
uint16_t ADCdataAIN;

float v_out = 0.0;
float i_rms = 0.0;
float v_in = 0.0;
int error = 0;
float ref = 0.0;
bit relay_flag = 0;
bit start_flag = 0;
bit error_flag = 0;
bit buzer = 0;
unsigned int error_count = 0;
int zc_flag = 0, temp = 0;
unsigned char update_rate = 45, ui_state = ui_normal;
int target = 230;
volatile bit display_update_flag = 0;
volatile unsigned int ms_counter = 0, ui = 0,timer_20ms =0,buzer_count=0;
volatile unsigned int loop_flag = 0;
unsigned char counter_display = 1;

//-----------------------------------------------------------------------------------

//-----------------Analog Measurment Parameter Configuration-------------------------
#define ADC_MAX 4095.0
#define ADC_REF_VOLT 5.00
//#define ADC_OFFSET 1.649
#define ADC_OFFSET 2.06
//#define GAIN_FACTOR 0.000202020
#define GAIN_FACTOR 0.002222
#define SAMPLE_COUNT 402

#define CUR_ADC_OFFSET 2.10
#define BURDEN_GAIN 36.0
#define CURRENT_SAMPLES 202

//------------------------------------------------------------------------------------



//------------------------Function Prototype-------------------------------------
void Delay_us(unsigned int us);

int adc_data(void);
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

void Timer0_Init_2ms(void);

//--------------------------------------------------------------------------------

//-------------------Delay US Function For Display--------------------------------
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
//-----------------------------------------------------------------------------------

//----------------- ----ADC Read With INT Return-------------------------------------
int adc_data(void)
{
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;

    while (!(ADCCON0 & SET_BIT7))
        ;

    ADCdataAIN = (ADCRH << 4);
    ADCdataAIN |= (ADCRL & 0x0F);

    return ADCdataAIN;
}
//----------------------------------------------------------------------------------


//--------------------OUTPUT Voltage Measurment With Float Return-------------------
float ac_voltage_rms(void)
{

    unsigned int i, adc_val;
    float sum_squares = 0.0;
    float v_adc, ac_signal, actual_voltage;

    ENABLE_ADC_AIN4;
    ENABLE_ADC;

    ref = 0;

    zc_flag = 0;
    
	  while(P15 != 0 && (++zc_flag < 300));
    while(P15 == 0 && (++zc_flag < 300));
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = adc_data();
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        //printf("Vadc = %0.3f\n",v_adc); // print analog voltage Reading
        ref += v_adc;

        ac_signal = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;
        sum_squares += actual_voltage * actual_voltage;
    }
    

    DISABLE_ADC;

    // printf("The ref = %0.3f \n", ref/SAMPLE_COUNT); //For Know Refrence Voltage
    ref = 0;

    return sqrt(sum_squares / SAMPLE_COUNT);
}
//----------------------------------------------------------------------------------


//-------------------------Input Voltage Measurment---------------------------------
float ac_voltage_rms_input(void)
{
    unsigned int i;
    float sum_squares = 0.0;
    float adc_val, v_adc, ac_signal, actual_voltage;

    ENABLE_ADC_AIN1;
    ENABLE_ADC;

    ref = 0;
    // EA = 0;
	zc_flag = 0;
    
	  while(P15 != 0 && (++zc_flag < 300));
    while(P15 == 0 && (++zc_flag < 300));
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = adc_data();
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
        ac_signal = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;

        sum_squares += actual_voltage * actual_voltage;
    }
    // EA = 1;
    DISABLE_ADC;

    return sqrt(sum_squares / SAMPLE_COUNT);
}

//---------------------------------------------------------------------------------

//-----------------OUTPUT Current Measurment----------------------------------------
float ac_current_rms(void)
{
    unsigned int i;
    float sum_squares = 0.0;
    float adc_val, v_adc, v_ac, i_secondary, i_primary;

    ENABLE_ADC_AIN5;
    ENABLE_ADC;
ref =5;
    for (i = 0; i < CURRENT_SAMPLES; i++)
    {
        adc_val = adc_data();
			//printf("The offset is %0.3f \n",v_adc);
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
        
			
        v_ac = v_adc - CUR_ADC_OFFSET;

        i_secondary = v_ac / BURDEN_GAIN;
        i_primary = i_secondary * 3113;

        sum_squares += i_primary * i_primary;

        Delay_us(150);
    }

    DISABLE_ADC;
    //printf("The offset is %0.3f \n",ref);
    return sqrt(sum_squares / CURRENT_SAMPLES);
}

//----------------------------------------------------------------------------------

//---------------------All Disply Function------------------------------------------
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
    unsigned char i = 0;

    while (str[i] != '\0' && i < 4)
    {
        d[i] = TM1637_CharToSeg(str[i]);
        i++;
    }

    TM1637_DisplayRaw(d[0], d[1], d[2], d[3]);
}

void TM1637_DisplayCurrent(float current)
{
    unsigned int int_part;
    unsigned int dec_part;
    unsigned char d0, d1, d2, d3;

    if (current < 0)
        current = 0;
    if (current > 99.99)
        current = 99.99;

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

//----------------------------------------------------------------------------------

//----------------------Buzer Count-------------------------------------------------

void buzer_on() {
	
	 
	if(buzer_count < 15) {
		//printf("buzer on \n");
		buzer = 1;
	}
	
	else {
		buzer = 0;
	}
	++buzer_count;
	
	if(buzer_count > 30) {
		buzer_count = 0;
	}
	
	P16 = buzer;
}

//----------------------------------------------------------------------------------

//-------------Timer0 Innetrupt At Every 20ms--------------------------------------
void Timer0_Init_2ms(void)
{
    TMOD &= 0xF0; // Clear Timer0 bits
    TMOD |= 0x01; // Timer0 Mode 1 (16-bit)

    TH0 = 0xF0; // Load for 2ms delay
    TL0 = 0x60;

    TF0 = 0; // Clear overflow flag
    ET0 = 1; // Enable Timer0 interrupt
    EA = 1;  // Enable global interrupts
    TR0 = 1; // Start Timer0
}
//----------------------------------------------------------------------------------

//--------------------Timer0 ISR Function-------------------------------------------
void Timer0_ISR(void) interrupt 1
{
    // Reload timer for next 2ms
    TH0 = 0xF0;
    TL0 = 0x60;

    ms_counter++;

    if (SETTING == 1)
    {
        ++ui;
    }

    if (ui > 100)
    {
        update_rate = 0;
        ui = 0;
        temp = 999;
        ui_state = ui_password;
    }

    if (ms_counter >= update_rate) // 1 × 2ms = 2ms
    {
        // printf("---------ok--------\n");
        // printf("ms counter = %d\n",ms_counter);
        ms_counter = 0;
        display_update_flag = 1;
    }
		++timer_20ms;

		
		
    if(timer_20ms > (START_TIME_IN_SEC * 50)) {
    start_flag = 1;
			P16=0;
			//printf("\n buzer off flag 1 \n");
		}		
   else if(start_flag == 0) {
		 buzer_on();
	 }
}
//----------------------------------------------------------------------------------

void main(void)
{
	
    MODIFY_HIRC(HIRC_24);
    Enable_UART0_VCOM_printf_24M_115200();

    P01_QUASI_MODE;
    P00_QUASI_MODE;
    P12_PUSHPULL_MODE;
    P13_PUSHPULL_MODE;
    P17_PUSHPULL_MODE;
	  P16_PUSHPULL_MODE;
    P15_INPUT_MODE;
    P11_INPUT_MODE;
    P10_INPUT_MODE;
    P03_INPUT_MODE;
	  P12=1;
	  P13=1;
	  P17=0;

	  TM1637_DisplayString("IP");
    Timer0_Init_2ms();

    while (1)
    {
        EA = 0;
        v_out = (ac_voltage_rms() * 0.92);
        i_rms = (ac_current_rms() - 5.7);
     // v_in  = (ac_voltage_rms_input());

        if (error_flag == 0 || error < (-220) || error_count > 1500)
        {
            v_in = (ac_voltage_rms_input() * 0.92);
            EA = 1;
            error_count = 0;
        }
        else
        {
            ++error_count;
            //printf("error = %d\n",error_count); //For Error Count print
        }
        if (v_out < 100 || v_out > 1000)
        {
            v_out = 0;
        }
        if (v_in < 100 || v_in > 1000)
        {
            v_in = 0;
        }
        if (i_rms < 0.6)
        {
            i_rms = 0;
        }
        //printf("The voltage is %0.3f \n", v_out); // For output voltage print
				//printf("The current is %0.3f \n",i_rms);
				
        error = (int)(v_out - target);
        // printf("The error is %d \n",(int)error); //For Error Print

        if (error >= 6)
        {
         // printf("in inc\n");
            //P12 = 0;
            //P13 = 1;
					P12=1;
					P13=0;
            //error_flag = 1;
        }
        else if (error <= -6)
        {
         // printf("Enter in dec\n");
           // P13 = 0;
            //P12 = 1;
					P13=1;
					P12=0;
            error_flag = 1;
        }
        else
        {
            //P12 = 0;
            //P13 = 0;
					P12=1;
					P13=1;
            error_flag = 0;
        }

        //if (v_out > OUTPUT_LOW && v_out < OUTPUT_HIGH && v_in < INPUT_HIGH && v_in > INPUT_LOW && i_rms < OVERCURRENT && start_flag==1)
				if (start_flag==1)
        {
            relay_flag = 1;
					//printf("relay On \n");
        }
        else
        {
					//printf("relay Off \n");

            relay_flag = 0;
        }                 

        P17 = relay_flag;

        if (display_update_flag)
        {
            display_update_flag = 0;

            if (ui_state == ui_normal)
            {
                switch (counter_display)
                {
                case 0:
                    TM1637_DisplayString("IP");
                    break;
                case 1:
                    TM1637_DisplayNumber((unsigned int)v_in);
                    break;
                case 2:
                    TM1637_DisplayString("OP");
                    break;
                case 3:
                    TM1637_DisplayNumber((unsigned int)v_out);
                    break;
                case 4:
                    TM1637_DisplayString("OA");
                    break;
                case 5:
                    TM1637_DisplayCurrent(i_rms);
                    break;
                }

                counter_display++;
                if (counter_display > 5)
                    counter_display = 0;
            }
            else
            {

                switch (ui_state)
                {

                case ui_password:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1 && temp == 998)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_cal;
                    }
                    break;

                case ui_cal:
                    TM1637_DisplayString("CAL");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_vol;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_ol;
                    }
                    break;

                case ui_vol:
                    TM1637_DisplayString("VOL");
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_cur;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_exit;
                    }
                    break;

                case ui_cur:
                    TM1637_DisplayString("CUR");
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_exit;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_vol;
                    }
                    break;

                case ui_exit:
                    TM1637_DisplayString("E");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_cal;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_vol;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_cur;
                    }
                    break;

                case ui_ol:
                    TM1637_DisplayString("OVL");
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_end;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_cal;
                    }
                    break;

                case ui_end:
                    TM1637_DisplayString("END");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_normal;
                        update_rate = 55;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_cal;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ui_ol;
                    }
                }

                if (UP == 1)
                {
                    Timer2_Delay(24000000, 1, 100);
                    ++temp;
                }
                else if (DOWN == 1)
                {
                    Timer2_Delay(24000000, 1, 100);
                    --temp;
                }
            }
        }

        // Timer0_Delay(24000000,300,1000);
    }
}
