//---------Header File Section-------------------------------------------
#include "numicro_8051.h"
#include <intrins.h>

//----------------------------------------------------------------------------

//----------------------------Pin Defination-------------------------------------
#define TM_CLK P00 // TM1637 CLK
#define TM_DIO P01 // TM1637 DIO
#define SETTING P10
#define UP P03
#define DOWN P02
//--------------------------------------------------------------------------------

//----------flash---------------------------------------------------

//#define CONFIG_FLASH_ADDR 0x3C00
#define CONFIG_FLASH_ADDR  0x3CCD   // exact 15.2 KB 15565
//#define CONFIG_FLASH_SIZE 1 // 0.5 KB

typedef struct
{
    int16_t target_voltage; // 2 bytes
    int16_t on_delay;       // 2
    int16_t input_low;      // 2
    int16_t input_high;     // 2
    int16_t output_low;     // 2
    int16_t output_high;    // 2
    uint8_t dis;            // 1

    int16_t hlt_delay; // 2

    uint8_t bur;        // bit (use 0/1)
    uint8_t regulation; // 8 bit
    uint8_t olr;        // bit
    uint8_t ol1;        // 8 bit
    uint8_t olt;        // 8 bit
    uint8_t ol2;        // 8 bit

    float ipc; // 4
    float opc; // 4
    float oac; // 4

    uint8_t auto_status; // bit

    uint16_t crc; // for data validation

} UI_Config_t;

UI_Config_t config;

uint16_t CalcCRC(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xA55A;
    while (len--)
        crc = (crc << 1) ^ *buf++;
    return crc;
}

void Save_Config(void)
{
    config.crc = CalcCRC((uint8_t *)&config,
                         sizeof(UI_Config_t) - 2);

    Write_DATAFLASH_ARRAY(CONFIG_FLASH_ADDR,
                          (uint8_t *)&config,
                          sizeof(UI_Config_t));
}

bit Load_Config(void)
{
    uint16_t i;
    uint8_t *p = (uint8_t *)&config;

    for (i = 0; i < sizeof(UI_Config_t); i++)
        p[i] = Read_APROM_BYTE(CONFIG_FLASH_ADDR + i);

    if (config.crc ==
        CalcCRC((uint8_t *)&config,
                sizeof(UI_Config_t) - 2))
        return 1; // valid
    else
        return 0; // corrupted
}

//-------------------------------------------------------------------

//---------------------Protection Configuration-----------------------------------

//#define START_TIME_IN_SEC 1 // at 1 12sec   50 count

//--------------------------------------------------------------------------------

//----------------------------UI Defination--------------------------------------
typedef enum
{
    /* ---- Basic UI ---- */
    UI_NORMAL = 0,
    UI_PASSWORD,
    UI_CAL,
    UI_VOL,
    UI_OL,
    UI_END,
    UI_CUR,
    UI_EXIT,
    SOP,
    TARGET_VOLTAGE,
    POD,
    ON_DELAY_SET,
    IPL,
    INPUT_LOW_SET,
    IN_HIGH,
    INPUT_HIGH_SET,
    DIS,
    INPUT_VOLTAGE,
    OPL,
    OUTPUT_LOW_SET,
    OPH,
    OUTPUT_HIGH_SET,
    OP,
    OA,
    HLT,
    HLT_DELAY_SET,
    BUR,
    BUR_ON,
    BUR_OFF,
    REG,
    REGULATION_SET,
    OLR,
    OLR_ON,
    OLR_OFF,
    OL1,
    OL1_SET,
    OLT,
    OLT_SET,
    OL2,
    OL2_SET,
    IPC,
    IPC_SET,
    OPC,
    OPC_SET,
    OAC,
    OAC_SET,
    ALL,
    US,
    FS,
    AUT,
    AUT_ON,
    AUT_OFF,
    US_PASSWORD,
    FS_PASSWORD,
    UI_US_EXIT,
    UI_FS_EXIT

} ui_state_t;

ui_state_t ui_state;

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
//#define SEG_K 0x75 // approx
#define SEG_L 0x38
//#define SEG_M 0x37 // approx
#define SEG_N 0x54
#define SEG_O 0x3F
#define SEG_P 0x73
//#define SEG_Q 0x67 // approx
#define SEG_R 0x50
#define SEG_S 0x6D
#define SEG_T 0x78
#define SEG_U 0x3E
//#define SEG_V 0x3E // same as U
//#define SEG_W 0x2A // approx
//#define SEG_X 0x76 // same as H
//#define SEG_Y 0x6E
//#define SEG_Z 0x5B // same as 2

#define SEG_DASH 0x40  // -
#define SEG_UNDER 0x08 // _
#define SEG_BLANK 0x00 // space
#define SEG_DP 0x80    // decimal point

#define SEG_0 0x3F // 0
#define SEG_1 0x06 // 1
#define SEG_2 0x5B // 2
#define SEG_3 0x4F // 3
#define SEG_4 0x66 // 4
#define SEG_5 0x6D // 5
#define SEG_6 0x7D // 6
#define SEG_7 0x07 // 7
#define SEG_8 0x7F // 8
#define SEG_9 0x6F // 9

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
    case 'G':
        return SEG_G;
		case 'F':
        return SEG_F;
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
		case 'T':
			return SEG_T;
        case 'U':
        return SEG_U;
   
    case '-':
        return SEG_DASH;

    case '0':
        return SEG_0;

     case '1':
        return SEG_1;
		 case '2':
        return SEG_2;
		 case '3':
        return SEG_3;
		 case '4':
        return SEG_4;
		 case '5':
        return SEG_5;

    case ' ':
        return SEG_BLANK;

    default:
        return SEG_BLANK;
    }
}

//---------------------------------------------------------------------------------
//dis update 45 at 10 sec
//------------------------Global Variable Section----------------------------------
uint16_t ADCdataAIN;

int v_out = 0,v_out_raw=0;
float i_rms = 0;
int v_in = 0,v_in_raw=0;
float output_cal = 0.92, input_cal = 0.92, current_cal = 1;
int error = 0;
float ref = 0.0;
bit relay_flag = 0,error_code=0;
bit start_flag = 0;
bit error_flag = 0, aut = 1;
bit buzer = 0;
bit setting_press = 0;
unsigned int error_count = 0,disp_update_error=0, out_high = 270, out_low = 210, in_high = 270, in_low = 150, ol1 = 3, ol2 = 5, hlt = 5, olt = 120,pod_delay=41;
int zc_flag = 0, temp = 0;
unsigned char update_rate = 20;
int target = 230;
volatile bit display_update_flag = 0;
volatile unsigned int ms_counter = 0, ui = 0, timer_20ms = 0, buzer_count = 0, trip_count = 0;
volatile unsigned int loop_flag = 0;
volatile char band_inc = 3, band_dec = -3;
unsigned char counter_display = 1, dis_start = 0, dis_end = 5;

//-----------------------------------------------------------------------------------

//-----------------Analog Measurment Parameter Configuration-------------------------
#define ADC_MAX 4095.0
#define ADC_REF_VOLT 5.00
// #define ADC_OFFSET 1.649
#define ADC_OFFSET 2.06
// #define GAIN_FACTOR 0.000202020
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
void trip(char n);

uint16_t CalcCRC(uint8_t *buf, uint16_t len);
void Save_Config(void);
bit Load_Config(void);
unsigned long isqrt32(unsigned long x);
void buzer_on();
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

// 32-bit integer square root for C51
unsigned long isqrt32(unsigned long x)
{
    unsigned long res = 0;
    unsigned long mask = 0x40000000UL; // renamed

    while (mask > x)
        mask >>= 2;

    while (mask != 0)
    {
        if (x >= res + mask)
        {
            x -= res + mask;
            res = (res >> 1) + mask;
        }
        else
        {
            res >>= 1;
        }
        mask >>= 2;
    }
    return res;
}

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

    while (P15 != 0 && (++zc_flag < 300))
        ;
    while (P15 == 0 && (++zc_flag < 300))
        ;
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = adc_data();
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        // printf("Vadc = %0.3f\n",v_adc); // print analog voltage Reading
        ref += v_adc;

        ac_signal = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;
        sum_squares += actual_voltage * actual_voltage;
    }

    DISABLE_ADC;

    // printf("The ref = %0.3f \n", ref/SAMPLE_COUNT); //For Know Refrence Voltage
    ref = 0;

    return isqrt32(sum_squares / SAMPLE_COUNT);
    // return sqrt(sum_squares / SAMPLE_COUNT);
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

    while (P15 != 0 && (++zc_flag < 300))
        ;
    while (P15 == 0 && (++zc_flag < 300))
        ;
    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = adc_data();
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
			ref += v_adc;
        ac_signal = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;

        sum_squares += actual_voltage * actual_voltage;
    }
    // EA = 1;
    DISABLE_ADC;

    return isqrt32(sum_squares / SAMPLE_COUNT);
    // return sqrt(sum_squares / SAMPLE_COUNT);
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
    ref = 5;
    for (i = 0; i < CURRENT_SAMPLES; i++)
    {
        adc_val = adc_data();
        // printf("The offset is %0.3f \n",v_adc);
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        v_ac = v_adc - CUR_ADC_OFFSET;

        i_secondary = v_ac / BURDEN_GAIN;
        i_primary = i_secondary * 3113;

        sum_squares += i_primary * i_primary;

        Delay_us(150);
    }

    DISABLE_ADC;
    // printf("The offset is %0.3f \n",ref);
    return isqrt32(sum_squares / CURRENT_SAMPLES);
    // return sqrt(sum_squares / CURRENT_SAMPLES);
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

//---------------------------------------------------------------------------------------

void trip(char n)
{

    ++trip_count;
    
    switch (n)
    {

    case 1:
        P17 = 0;
		    P13 = 1;
		    P12 = 1;
		    EA = 0;
        while (UP == 0)
        {
            TM1637_DisplayString("E-01");
            P16 = 1;
            Timer2_Delay(24000000, 13, 500);
            P16 = 0;
            if (error < 0)
            {
                TM1637_DisplayString("INC");
            }
            else
            {
                TM1637_DisplayString("DEC");
            }
            Timer2_Delay(24000000, 11, 700);
        }
				start_flag = 0;
				timer_20ms = 0;
				EA = 1;
        break;

    case 2:
		    ++disp_update_error;
		    
		     if(disp_update_error < 6) {
					 TM1637_DisplayString("E-02");
				 }
		     else if(disp_update_error < 12) {
				 TM1637_DisplayString("IPH");
				 }
				 else {
				 disp_update_error = 0;
				 }
        if (trip_count > hlt)
        { // 40 9sec
            relay_flag = 0;
        }
        else
        {
            buzer_on();
        }
        break;
				

    case 3:

        if (trip_count > olt)
        {   
					  P17 = 0;
            P16 = 0;
				    P13 = 1;
		        P12 = 1;
				    EA = 0;
            while (UP != 1)
            {
                TM1637_DisplayString("OL-1");
            }
						
						start_flag = 0;
				    timer_20ms = 0;
						i_rms = 0;
						relay_flag = 0;
						counter_display=1;
						TM1637_DisplayString("IP");
				    EA = 1;
        }
        else
        {
            buzer_on();
        }
        break;
				
				case 4:
		    ++disp_update_error;
		    
		     if(disp_update_error < 6) {
					 TM1637_DisplayString("E-03");
				 }
		     else if(disp_update_error < 12) {
				 TM1637_DisplayString("IPL");
				 }
				 else {
				 disp_update_error = 0;
				 }
        if (trip_count > hlt)
        { // 40 9sec
            relay_flag = 0;
        }
        else
        {
            buzer_on();
        }
        break;
				
				case 5:
		    ++disp_update_error;
		    
		     if(disp_update_error < 6) {
					 TM1637_DisplayString("E-04");
				 }
		     else if(disp_update_error < 12) {
				 TM1637_DisplayString("OPL");
				 }
				 else {
				 disp_update_error = 0;
				 }
        if (trip_count > hlt)
        { // 40 9sec
            relay_flag = 0;
        }
        else
        {
            buzer_on();
        }
        break;
				
				
				case 6:
		    ++disp_update_error;
		    
		     if(disp_update_error < 6) {
					 TM1637_DisplayString("E-05");
				 }
		     else if(disp_update_error < 12) {
				 TM1637_DisplayString("OPH");
				 }
				 else {
				 disp_update_error = 0;
				 }
        if (trip_count > hlt)
        { // 40 9sec
            relay_flag = 0;
        }
        else
        {
            buzer_on();
        }
        break;
				
				case 7:
					
            P17 = 0;
            P16 = 0;
				    P13 = 1;
		        P12 = 1;
				    EA = 0;
            while (UP != 1)
            {
                TM1637_DisplayString("OL-2");
            }
						start_flag = 0;
				    timer_20ms = 0;
						i_rms = 0;
						relay_flag = 0;
						counter_display=1;
						TM1637_DisplayString("IP");
				    EA = 1;
        break;
					
    }
}

//---------------------------------------------------------------------------------------

void set_dis()
{
    switch (config.dis)
    {

    case 1:
        dis_start = 0;
        dis_end = 1;
        break;
    case 2:
        dis_start = 2;
        dis_end = 3;
        break;
    case 3:
        dis_start = 4;
        dis_end = 5;
        break;
    case 4:
        dis_start = 0;
        dis_end = 5;
        break;
    }
}

//----------------------Buzer Count-------------------------------------------------

void buzer_on()
{

    if (buzer_count < 15)
    {
        // printf("buzer on \n");
        buzer = 1;
    }

    else
    {
        buzer = 0;
    }
    ++buzer_count;

    if (buzer_count > 30)
    {
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

    if (ui > 30)
    {
        update_rate = 0;
        ui = 0;
        temp = 999;
        ui_state = AUT;
    }

    if (ms_counter >= update_rate) // 1 × 2ms = 2ms
    {
        // printf("---------ok--------\n");
        // printf("ms counter = %d\n",ms_counter);
        ms_counter = 0;
        display_update_flag = 1;
    }
    ++timer_20ms;

    if (timer_20ms > pod_delay)
    {
        start_flag = 1;
        P16 = 0;
        // printf("\n buzer off flag 1 \n");
    }
    else if (start_flag == 0)
    {
        buzer_on();
    }
}

//------------------------update----------------------

void set_parameter()
{

    target = config.target_voltage;
    set_dis();
    in_high = config.input_high;
    in_low = config.input_low;
    out_high = config.output_high;
    out_low = config.output_low;
    hlt = (int)(config.hlt_delay * 4.44);
    band_inc = config.regulation;
    band_dec = config.regulation * (-1);
    ol1 = config.ol1;
    ol2 = config.ol1 + 3;
    olt = (int)(config.olt * 4.44);
	  pod_delay = (int)(config.on_delay  * 4.16);
    input_cal = config.ipc;
    output_cal = config.opc;
    current_cal = config.oac;
}

//---------------------------------------------------

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
    P12 = 1;
    P13 = 1;
    P17 = 0;

    if (Load_Config() == 0)
    {
        // first time defaults
        config.target_voltage = 230;
        config.on_delay = 10;
        config.input_low = 180;
        config.input_high = 270;
        config.output_low = 210;
        config.output_high = 270;
        config.dis = 4;
        config.hlt_delay = 5;
        config.bur = 0;
        config.regulation = 3;
        config.olr = 1;
        config.ol1 = 10;
        config.olt = 10;
        config.ol2 = 13;
        config.ipc = 0.92;
        config.opc = 0.92;
        config.oac = 1;
        config.auto_status = 1;

        Save_Config(); // store defaults
    }
    set_parameter();
		
		 v_in = (int)(ac_voltage_rms_input() * input_cal);

    TM1637_DisplayString("IP");
    Timer0_Init_2ms();

    while (1)
    {
        EA = 0;
        v_out = (int)(ac_voltage_rms() * output_cal);
        if (v_out < 100 || v_out > 1000)
        {
            v_out = 0;
        }
        error = (int)(v_out - target);

        if (error >= band_inc && aut)
        {
            P12 = 1;
            P13 = 0;
            error_flag = 1;
        }
        else if (error <= band_dec && aut)
        {
            P13 = 1;
            P12 = 0;
            error_flag = 1;
        }
        else if (aut == 1)
        {
            P12 = 1;
            P13 = 1;
            error_flag = 0;
        }
        if (error_flag == 0 || error < (-220) || error_count > 50)
        {
            v_in = (int)(ac_voltage_rms_input() * input_cal);
            i_rms = (ac_current_rms() - 5.4) * current_cal;
            if (v_in < 100 || v_in > 1000)
            {
                v_in = 0;
            }
            if (i_rms < 0.6)
            {
                i_rms = 0;
            }
            EA = 1;

            if (error_count > 80)
            {
                trip(1);
                error_count = 0;
            }

            if (error_count > 50)
            {

                ++error_count;
            }
            else
            {
                error_count = 0;
            }
            // error_count = 0;
        }
        else
        {
            if (error_count > 20)
                P16 = 0;
            ++error_count;
        }

        // printf("The voltage is %0.3f \n", v_out); // For output voltage print
        // printf("The current is %0.3f \n",i_rms);
        // printf("The error is %d \n",(int)error); //For Error Print
        // if (v_out > OUTPUT_LOW && v_out < OUTPUT_HIGH && v_in < INPUT_HIGH && v_in > INPUT_LOW && i_rms < OVERCURRENT && start_flag==1)
        if (start_flag == 1)
        {
            relay_flag = 1;
        }
        else
        {
            relay_flag = 0;
        }

        //-----------------------manual------------

        if (aut == 0 && ui_state == UI_NORMAL)
        {

            if (UP == 1)
            {
                P13 = 1;
                P12 = 0;
            }

            else if (DOWN == 1)
            {
                P12 = 1;
                P13 = 0;
            }

            else
            {
                P12 = 1;
                P13 = 1;
            }
        }

        //------------------------------------

        if (v_out > out_high)
        {
            error_code = 1;
            trip(6);
        }
				else if(v_out < out_low) {
					error_code = 1;
				 trip(5);
				}
        else if (v_in > in_high)
        {
					error_code = 1;
            trip(2);
        }
				else if(v_in < in_low) {
					error_code = 1;
				trip(4);
				}
        else if (i_rms > ol1)
        {
            trip(3);
        }
				else if(i_rms > ol2) {
				    trip(7);
				}
        else if (start_flag == 1)
        {
            trip_count = 0;
            relay_flag = 1;
					  error_code = 0;
        }
        else
        {
					  error_code = 0;
            trip_count = 0;
        }
        P17 = relay_flag;

        if (display_update_flag)
        {
            display_update_flag = 0;

            if (ui_state == UI_NORMAL && error_code == 0)
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
                if (counter_display > dis_end)
                {
                    counter_display = dis_start;
                }
            }
            else
            {

                switch (ui_state)
                {

                case AUT:
                    TM1637_DisplayString("AUT");
                    if (SETTING == 1 && setting_press == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT_ON;
                    }
                    if (SETTING == 0)
                    {
                        setting_press = 1;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = US;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = UI_EXIT;
                    }
                    break;
                case AUT_ON:
                    TM1637_DisplayString("ON");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.auto_status = 1;
                        aut = 1;
                        ui_state = AUT;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT_OFF;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT_OFF;
                    }
                    break;

                case AUT_OFF:
                    TM1637_DisplayString("OFF");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.auto_status = 0;
                        aut = 0;
                        ui_state = AUT;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT_ON;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT_ON;
                    }
                    break;

                case US:
                    TM1637_DisplayString("US");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = 0;
                        ui_state = US_PASSWORD;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = FS;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT;
                    }
                    break;

                case US_PASSWORD:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1 && temp == 5)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = SOP;
                    }
                    break;

                case SOP:
                    TM1637_DisplayString("SOP");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.target_voltage;
                        ui_state = TARGET_VOLTAGE;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = POD;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = UI_US_EXIT;
                    }
                    break;

                case TARGET_VOLTAGE:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.target_voltage = temp;
                        target = temp;
                        ui_state = SOP;
                    }
                    break;

                case POD:
                    TM1637_DisplayString("POD");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.on_delay;
                        ui_state = ON_DELAY_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IPL;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = SOP;
                    }
                    break;

                case ON_DELAY_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.on_delay = temp;
											  pod_delay = (int)(config.on_delay  * 4.16);
                        ui_state = POD;
                    }
                    break;

                case IPL:
                    TM1637_DisplayString("IPL");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.input_low;
                        ui_state = INPUT_LOW_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IN_HIGH;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = POD;
                    }
                    break;

                case INPUT_LOW_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.input_low = temp;
                        in_low = temp;
                        ui_state = IPL;
                    }
                    break;

                case IN_HIGH:
                    TM1637_DisplayString("IPH");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.input_high;

                        ui_state = INPUT_HIGH_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OPL;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IPL;
                    }
                    break;

                case INPUT_HIGH_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.input_high = temp;
                        in_high = temp;
                        ui_state = IN_HIGH;
                    }
                    break;

                case OPL:
                    TM1637_DisplayString("OPL");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.output_low;
                        ui_state = OUTPUT_LOW_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OPH;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IN_HIGH;
                    }
                    break;

                case OUTPUT_LOW_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.output_low = temp;
                        out_low = temp;
                        ui_state = OPL;
                    }
                    break;

                case OPH:
                    TM1637_DisplayString("OPH");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.output_high;
                        ui_state = OUTPUT_HIGH_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = HLT;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OPL;
                    }
                    break;

                case OUTPUT_HIGH_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.output_high = temp;
                        out_high = temp;
                        ui_state = OPH;
                    }
                    break;

                case HLT:
                    TM1637_DisplayString("HLT");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.hlt_delay;
                        ui_state = HLT_DELAY_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = DIS;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OPH;
                    }
                    break;

                case HLT_DELAY_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.hlt_delay = temp;
                        hlt = (int)(temp * 4.44);
                        ui_state = HLT;
                    }
                    break;

                case DIS:
                    TM1637_DisplayString("DIS");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = 5;
                        ui_state = INPUT_VOLTAGE;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = HLT;
                    }
                    break;

                case INPUT_VOLTAGE:
                    TM1637_DisplayString("IP");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.dis = 1;
                        set_dis();
                        ui_state = DIS;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OP;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ALL;
                    }
                    break;

                case OP:
                    TM1637_DisplayString("OP");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.dis = 2;
                        set_dis();
                        ui_state = DIS;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OA;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IP;
                    }
                    break;

                case OA:
                    TM1637_DisplayString("OA");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.dis = 3;
                        set_dis();
                        ui_state = DIS;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = ALL;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OP;
                    }
                    break;

                case ALL:
                    TM1637_DisplayString("ALL");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.dis = 4;
                        set_dis();
                        ui_state = DIS;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = INPUT_VOLTAGE;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OA;
                    }
                    break;

                case BUR:
                    TM1637_DisplayString("BUR");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR_ON;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = UI_US_EXIT;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = DIS;
                    }
                    break;
                case BUR_ON:
                    TM1637_DisplayString("ON");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.bur = 1;
                        ui_state = BUR;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR_OFF;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR_OFF;
                    }
                    break;
                case BUR_OFF:
                    TM1637_DisplayString("OFF");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.bur = 0;
                        ui_state = BUR;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR_ON;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR_ON;
                    }
                    break;

                case FS:
                    TM1637_DisplayString("FS");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = 999;
                        ui_state = FS_PASSWORD;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = UI_EXIT;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = US;
                    }
                    break;

                case FS_PASSWORD:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1 && temp == 998)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = REG;
                    }
                    break;

                case REG:
                    TM1637_DisplayString("REG");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.regulation;
                        ui_state = REGULATION_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = UI_FS_EXIT;
                    }
                    break;

                case REGULATION_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.regulation = temp;
                        band_inc = temp;
                        band_dec = (-1) * temp;
                        ui_state = REG;
                    }
                    break;

                case OLR:
                    TM1637_DisplayString("OLR");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR_ON;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OL1;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = REG;
                    }
                    break;
                case OLR_ON:
                    TM1637_DisplayString("ON");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.olr = 1;
                        ui_state = OLR;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR_OFF;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR_OFF;
                    }
                    break;
                case OLR_OFF:
                    TM1637_DisplayString("OFF");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.olr = 0;
                        ui_state = OLR;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR_ON;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR_ON;
                    }
                    break;

                case OL1:
                    TM1637_DisplayString("OL1");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.ol1;
                        ui_state = OL1_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLT;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLR;
                    }
                    break;

                case OL1_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.ol1 = temp;
											  ol1 = temp;
                        config.ol2 = temp + 3;
                        ui_state = OL1;
                    }
                    break;

                case OLT:
                    TM1637_DisplayString("OLT");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.olt;
                        ui_state = OLT_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OL2;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OL1;
                    }
                    break;

                case OLT_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.olt = temp;
											  olt = (int)(config.olt * 4.44);
                        ui_state = OLT;
                    }
                    break;

                case OL2:
                    TM1637_DisplayString("OL2");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = config.ol2;
                        ui_state = OL2_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IPC;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OLT;
                    }
                    break;

                case OL2_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.ol2 = temp;
                        ol2 = temp;
                        ui_state = OL2;
                    }
                    break;

                case IPC:
                    TM1637_DisplayString("IPC");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
											  v_in_raw = (int)(ac_voltage_rms_input());
                        temp = v_in_raw;
                        ui_state = IPC_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OPC;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OL2;
                    }
                    break;

                case IPC_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.ipc = (((float)temp) / v_in_raw);
                        input_cal = config.ipc;
                        ui_state = IPC;
                    }
                    break;

                case OPC:
                    TM1637_DisplayString("OPC");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
											 v_out_raw = (int)(ac_voltage_rms());
                        temp = 230;
                        ui_state = OPC_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OAC;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = IPC;
                    }
                    break;

                case OPC_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.opc = (float)(((float)temp) /v_out_raw);
                        output_cal = config.opc;
                        ui_state = OPC;
                    }
                    break;

                case OAC:
                    TM1637_DisplayString("OAC");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        temp = 5;
                        ui_state = OAC_SET;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = UI_FS_EXIT;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OPC;
                    }
                    break;

                case OAC_SET:
                    TM1637_DisplayNumber((unsigned int)temp);
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        config.oac = (float)(temp / i_rms);
                        // current_cal = config.oac;
                        ui_state = OAC;
                    }
                    break;

                case UI_EXIT:
                    TM1637_DisplayString("-E-");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        setting_press = 0;
                        Save_Config();
                        ui_state = UI_NORMAL;
                        update_rate = 20;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = AUT;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = FS;
                    }
                    break;

                case UI_US_EXIT:
                    TM1637_DisplayString("-E-");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = US;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = SOP;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = BUR;
                    }
                    break;

                case UI_FS_EXIT:
                    TM1637_DisplayString("-E-");
                    if (SETTING == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = FS;
                    }
                    if (UP == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = REG;
                    }
                    if (DOWN == 1)
                    {
                        Timer2_Delay(24000000, 1, 100);
                        ui_state = OAC;
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
