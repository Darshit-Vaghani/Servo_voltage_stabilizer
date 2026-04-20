//---------Header File Section-------------------------------------------
#include "numicro_8051.h"
#include <intrins.h>
#include <math.h>
//----------------------------------------------------------------------------

//----------------------------Pin Defination-------------------------------------
#define TM1637_CLK_PIN P13 // TM1637 CLK
#define TM1637_DIO_PIN P14 // TM1637 DIO
#define BTN_SET P30
#define BTN_UP P11
#define BTN_DOWN P10
#define BTN_HOME P02
//--------------------------------------------------------------------------------

//-------------------------LED_TYPES----------------------------------------------

typedef enum
{
    LED_OVL = 0, // A
    LED_IP,      // B
    LED_OP,      // C
    LED_OA,      // D
    LED_AUTO,    // E
    LED_DELAY,   // F
    LED_HILO,    // G
    LED_OUT      // DP
} status_led_t;

//-----------------------------------------------------------------------------------

//----------flash---------------------------------------------------

#define CONFIG_FLASH_ADDR 0x3E00 // 15872
// #define CONFIG_FLASH_ADDR  0x3CCD   // exact 15.2 KB 15565
// #define CONFIG_FLASH_SIZE 1 // 0.5 KB

typedef struct
{
    int16_t target_voltage; // 2 bytes
    int16_t on_delay;       // 2
    int16_t input_low;      // 2
    int16_t input_high;     // 2
    int16_t output_low;     // 2
    int16_t output_high;    // 2
    uint8_t display_mode;            // 1
	uint16_t earth_trip;
    int16_t ir_high;
    int16_t ir_low;

    int16_t hlt_delay; // 2

    uint8_t contactor_monitor_enabled;        // bit (use 0/1)
    uint8_t regulation; // 8 bit
    uint8_t olr;        // bit
    uint16_t overload_level1;        // 8 bit
    uint8_t overload_trip_ticks;        // 8 bit
    uint16_t overload_level2;        // 8 bit
	uint8_t earth_monitor_enabled;

    float input_voltage_calibration; // 4
    float output_voltage_calibration; // 4
    float current_calibration_factor; // 4

    uint8_t auto_mode_status; // bit

    uint16_t crc; // for data validation

} runtime_config_t;

runtime_config_t g_config;

uint16_t calculate_crc16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xA55A;
    while (len--)
        crc = (crc << 1) ^ *buf++;
    return crc;
}

void save_config_to_flash(void)
{
    g_config.crc = calculate_crc16((uint8_t *)&g_config,
                         sizeof(runtime_config_t) - 2);

    Write_DATAFLASH_ARRAY(CONFIG_FLASH_ADDR,
                          (uint8_t *)&g_config,
                          sizeof(runtime_config_t));
}

bit load_config_from_flash(void)
{
    uint16_t i;
    uint8_t *p = (uint8_t *)&g_config;

    for (i = 0; i < sizeof(runtime_config_t); i++)
        p[i] = Read_APROM_BYTE(CONFIG_FLASH_ADDR + i);

    if (g_config.crc ==
        calculate_crc16((uint8_t *)&g_config,
                sizeof(runtime_config_t) - 2))
        return 1; // valid
    else
        return 0; // corrupted
}

//-------------------------------------------------------------------

//---------------------Protection Configuration-----------------------------------

// #define START_TIME_IN_SEC 1 // at 1 12sec   50 count

//--------------------------------------------------------------------------------

//--------------------------define for max-min-----------------------------------

#define MAX_TARGET_VOLTAGE 250
#define MIN_TARGET_VOLTAGE 210
#define DEFAULT_TARGET_VOLTAGE 240

#define MAX_ON_DELAY 60
#define MIN_ON_DELAY 3
#define DEFAULT_ON_DELAY 10

#define MAX_INPUT_LOW 40 // setvoltage-40
#define MIN_INPUT_LOW 130
#define DEFAULT_INPUT_LOW 150

#define MAX_INPUT_HIGH 300
#define MIN_INPUT_HIGH 30 // setvoltage+30
#define DEFAULT_INPUT_HIGH 290

#define MAX_OUTPUT_HIGH 280
#define MIN_OUTPUT_HIGH 30 // setvoltage + 30
#define DEFAULT_OUTPUT_HIGH 270

#define MAX_OUTPUT_LOW 30 // setvoltage - 30
#define MIN_OUTPUT_LOW 130
#define DEFAULT_OUTPUT_LOW 200

#define MAX_OL1 100
#define MIN_OL1 2
#define DEFAULT_OL1 10 // current set in decimal point not in integer form ***

#define MAX_OL1_TIME 150
#define MIN_OL1_TIME 10 // default time is 90 sec
#define DEFAULT_OL1_TIME 90

#define MAX_REG 10
#define MIN_REG 2
#define DEFAULT_REG 3

// input betwwn 170 to 270 is motor side fault show M.F on display
// 1 second delay in imd handle_trip

// 1.contactor handle_trip detection_show disply is C.F,2. disply handle_trip show contrinusoly with their value. 3.

//-------------------------------------------------------------------------------

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
    CON,
    CON_ON,
    CON_OFF,
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
		EARTH,
		EARTH_HIGH_SET,
		EARTH_HIGH,
		EARTH_ON,
		EARTH_OFF,
        IR_HIGH,
        IR_HIGH_SET,
        IR_LOW,
        IR_LOW_SET,
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

} ui_state_id_t;

ui_state_id_t g_ui_state;

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
// #define SEG_K 0x75 // approx
#define SEG_L 0x38
#define SEG_M 0x55 // approx
#define SEG_N 0x54
#define SEG_O 0x3F
#define SEG_P 0x73
// #define SEG_Q 0x67 // approx
#define SEG_R 0x50
#define SEG_S 0x6D
#define SEG_T 0x78
#define SEG_U 0x3E
// #define SEG_V 0x3E // same as U
// #define SEG_W 0x2A // approx
// #define SEG_X 0x76 // same as H
#define SEG_Y 0x6E
// #define SEG_Z 0x5B // same as 2

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

const unsigned char digit_segments[10] =
    {
        0x3F, 0x06, 0x5B, 0x4F, 0x66,
        0x6D, 0x7D, 0x07, 0x7F, 0x6F};

static const char STR_ON[] = "ON";
static const char STR_OFF[] = "OFF";
static const char STR_END[] = "END";

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
        case 'M':
        return SEG_M;
				 case 'Y':
        return SEG_Y;

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
// display_mode update 45 at 10 sec
//------------------------Global Variable Section----------------------------------
uint16_t adc_data_ain;

int output_voltage = 0, output_voltage_raw = 0;
float output_current_rms = 0,overload_level1 = 10.0, overload_level2 = 13.0,current_state=0.0;
int input_voltage = 0, input_voltage_raw = 0;
int contactor_voltage = 0,earth_voltage=0;
float output_calibration = 0.92, input_calibration = 0.92, current_calibration = 1;
int voltage_control_error = 0;
unsigned int up_press_counter = 0, down_press_counter = 0, menu_timeout_counter = 0, edit_value_max = 1000, edit_value_min = 0;
float adc_voltage_accumulator = 0.0;
bit relay_enabled = 0, error_active = 0, input_low_latched = 0, input_high_latched = 0, trip_latched_flag = 0, output_low_latched = 0, output_high_latched = 0;
bit startup_complete = 0,home_state=0;
bit error_flag = 0, auto_mode_enabled = 1, startup_delay_active = 1;
bit buzzer_output = 0;
bit settings_page_active = 0, up_key_latched = 0, down_key_latched = 0, set_key_latched = 0;
unsigned int error_count_ticks = 0, trip_hold_ticks = 0, trip_display_cycle = 0, output_high_limit = 270, output_low_limit = 210, input_high_limit = 260, input_low_limit = 180, hi_lo_trip_ticks = 5, overload_trip_ticks = 120, power_on_delay_ticks = 41,earth_trip_threshold= 15;
unsigned char ir_high = 270, ir_low = 150;
int zero_cross_wait_counter = 0, edit_value = 0;
unsigned char display_update_ticks = 25, startup_countdown_seconds = 10, ticks_per_second = 5;
int target_voltage = 230;
volatile bit display_refresh_pending = 0;
volatile unsigned int display_tick_counter = 0, set_key_hold_ticks = 0, startup_timer_ticks = 0, buzzer_cycle_ticks = 0, trip_elapsed_ticks = 0;
volatile unsigned int main_loop_counter = 0;
volatile char regulation_band_high = 3, regulation_band_low = -3, input_voltage_sample_count = 0;
volatile unsigned char manual_led_tick = 0;
volatile unsigned char startup_delay_profile = 0;
volatile unsigned int overload_recovery_seconds = 0;
volatile unsigned int overload_trip_current_display = 0;
unsigned char display_page_index = 0, display_page_start = 0, display_page_end = 2, voltage_scan_step = 1;
unsigned char status_led_bitmap = 0;

//-----------------------------------------------------------------------------------

//-----------------Analog Measurment Parameter Configuration-------------------------
#define ADC_MAX 4095.0
#define ADC_REF_VOLT 5.00  //5.00
// #define ADC_OFFSET 1.649
#define ADC_OFFSET 2.5 // 2.47 , 2.5
// #define GAIN_FACTOR 0.000202020
#define GAIN_FACTOR 0.002222
#define SAMPLE_COUNT 402 // 402

#define CUR_ADC_OFFSET 2.47 // 2.10 , 2.47
#define BURDEN_GAIN 36.0
#define CURRENT_SAMPLES 101

//------------------------------------------------------------------------------------

//------------------------Function Prototype-------------------------------------
void delay_us(unsigned int us);

int read_adc_sample(void);
// int ac_voltage_rms(void);
int measure_ac_voltage_rms(uint8_t state);
float measure_ac_current_rms(void);

void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(unsigned char dat);
void tm1637_display_raw(unsigned char d0,
                       unsigned char d1,
                       unsigned char d2);
void tm1637_display_text(const char *str);
void tm1637_display_current(float current);
void tm1637_display_number(unsigned int num);

void timer0_init_2ms(void);
void handle_trip(char n);

uint16_t calculate_crc16(uint8_t *buf, uint16_t len);
void save_config_to_flash(void);
bit load_config_from_flash(void);
unsigned long isqrt32(unsigned long x);
void buzzer_on();
void start_overload_auto_recovery(unsigned char profile);
void start_manual_restart_delay(void);
//--------------------------------------------------------------------------------

//-------------------Delay US Function For Display--------------------------------
void delay_us(unsigned int us)
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
int read_adc_sample(void)
{
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;

    while (!(ADCCON0 & SET_BIT7))
        ;

    adc_data_ain = (ADCRH << 4);
    adc_data_ain |= (ADCRL & 0x0F);

    return adc_data_ain;
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
// int ac_voltage_rms(void)
// {

//     unsigned int i, adc_val;
//     float sum_squares = 0.0;
//     float v_adc, ac_signal, actual_voltage;

//     ENABLE_ADC_AIN4;
//     ENABLE_ADC;

//     adc_voltage_accumulator = 0;

//     zero_cross_wait_counter = 0;

//     while (P15 != 0 && (++zero_cross_wait_counter < 300))
//         ;
//     while (P15 == 0 && (++zero_cross_wait_counter < 300))
//         ;
//     for (i = 0; i < SAMPLE_COUNT; i++)
//     {
//         adc_val = read_adc_sample();
//         v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;

//         // printf("Vadc = %0.3f\n",v_adc); // print analog voltage Reading
//         adc_voltage_accumulator += v_adc;

//         ac_signal = v_adc - ADC_OFFSET;
//         actual_voltage = ac_signal / GAIN_FACTOR;
//         sum_squares += actual_voltage * actual_voltage;
//     }

//     DISABLE_ADC;

//     // printf("The adc_voltage_accumulator = %0.3f \n", adc_voltage_accumulator/SAMPLE_COUNT); //For Know Refrence Voltage
//     adc_voltage_accumulator = 0;

//     return isqrt32(sum_squares / SAMPLE_COUNT);
//     // return sqrt(sum_squares / SAMPLE_COUNT);
// }
// //----------------------------------------------------------------------------------

//-------------------------Input Voltage Measurment---------------------------------
int measure_ac_voltage_rms(uint8_t state)
{
    unsigned int i;
    float sum_squares = 0.0;
    float adc_val, v_adc, ac_signal, actual_voltage;

    if (state == 1)
    {
        ENABLE_ADC_AIN4; // input,ENABLE_ADC_AIN1;
    }
    else if(state == 0)
    {
        ENABLE_ADC_AIN5; // output,ENABLE_ADC_AIN4
    }
    else if(state == 2)
    {
        ENABLE_ADC_AIN2; // output,ENABLE_ADC_AIN4
    }
    else if(state == 3)
    {
        ENABLE_ADC_AIN3; // output,ENABLE_ADC_AIN4
    }
    ENABLE_ADC;

    adc_voltage_accumulator = 0;
    // EA = 0;
    zero_cross_wait_counter = 0;

    while (P15 != 0 && (++zero_cross_wait_counter < 300))
        ;
    while (P15 == 0 && (++zero_cross_wait_counter < 300))
        ;

    for (i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = read_adc_sample();
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
        adc_voltage_accumulator += v_adc;
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
float measure_ac_current_rms(void)
{
    unsigned int i;
    double sum_squares = 0.0;
    float adc_val, v_adc, v_ac, i_secondary, i_primary;

    ENABLE_ADC_AIN6;
    ENABLE_ADC;
    adc_voltage_accumulator = 5;
    for (i = 0; i < CURRENT_SAMPLES; i++)
    {
        adc_val = read_adc_sample();
        // printf("The offset is %0.3f \n",v_adc);
        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;

        v_ac = v_adc - CUR_ADC_OFFSET;

        i_secondary = v_ac / BURDEN_GAIN;
        i_primary = i_secondary * 3113;

        sum_squares += i_primary * i_primary;
    }

    DISABLE_ADC;
    // printf("The offset is %0.3f \n",adc_voltage_accumulator);
    // return isqrt32(sum_squares / CURRENT_SAMPLES);
    return sqrt(sum_squares / CURRENT_SAMPLES);
}

//----------------------------------------------------------------------------------

//---------------------All Disply Function------------------------------------------
void tm1637_start(void)
{
    TM1637_CLK_PIN = 1;
    TM1637_DIO_PIN = 1;
    delay_us(5);

    TM1637_DIO_PIN = 0;
    delay_us(5);

    TM1637_CLK_PIN = 0;
}

void tm1637_stop(void)
{
    TM1637_CLK_PIN = 0;
    TM1637_DIO_PIN = 0;
    delay_us(5);

    TM1637_CLK_PIN = 1;
    delay_us(5);

    TM1637_DIO_PIN = 1;
}

void tm1637_write_byte(unsigned char dat)
{
    unsigned char i;

    for (i = 0; i < 8; i++)
    {
        TM1637_CLK_PIN = 0;
        TM1637_DIO_PIN = dat & 0x01;
        delay_us(2);

        TM1637_CLK_PIN = 1;
        delay_us(2);

        dat >>= 1;
    }

    TM1637_CLK_PIN = 0;
    TM1637_DIO_PIN = 1;
    delay_us(2);

    TM1637_CLK_PIN = 1;
    delay_us(2);

    TM1637_CLK_PIN = 0;
}

void tm1637_display_raw(unsigned char d0,
                       unsigned char d1,
                       unsigned char d2)
{
    tm1637_start();
    tm1637_write_byte(0x40);
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0xC0);

    tm1637_write_byte(d0);
    tm1637_write_byte(d1);
    tm1637_write_byte(d2);
    // tm1637_write_byte(d3);

    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0x8F);
    tm1637_stop();
}

void tm1637_display_text(const char *str)
{
    unsigned char d[3] = {SEG_BLANK, SEG_BLANK, SEG_BLANK};
    unsigned char i = 0, j = 0;

    while (str[j] != '\0' && i < 3)
    {

        // IP.C
        if (str[j] != '.')
        {
            d[i] = TM1637_CharToSeg(str[j]);
            i++;
            ++j;
        }
        else
        {
            d[i - 1] |= SEG_DP;
            ++j;
        }
    }

    tm1637_display_raw(d[0], d[1], d[2]);
}

void tm1637_display_current(float current)
{
    unsigned int int_part;
    unsigned int dec_part;
    unsigned char d0, d1, d2;

    if (current < 0)
        current = 0;
    if (current > 99.99)
        current = 99.99;

    int_part = (unsigned int)current;
    dec_part = (unsigned int)((current - int_part) * 100);

    d0 = digit_segments[int_part / 10];
    d1 = digit_segments[int_part % 10];
    d2 = digit_segments[dec_part / 10];

    d1 |= SEG_DP;

    tm1637_display_raw(d0, d1, d2);
}

void tm1637_display_number(unsigned int num)
{
    unsigned char d[3];

    if (num > 999)
        num = 999;

    d[0] = digit_segments[num / 100];
    d[1] = digit_segments[(num / 10) % 10];
    d[2] = digit_segments[(num / 1) % 10];

    tm1637_display_raw(d[0], d[1], d[2]);
}

//------------led_controll-----------------------------------

void set_status_led(uint8_t led, bit state)
{
    if (state)
        status_led_bitmap |= (1 << led);
    else
        status_led_bitmap &= ~(1 << led);

    // FIXED ADDRESS MODE
    tm1637_start();
    tm1637_write_byte(0x44); // command: fixed address
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0xC3);      // 4th digit address
    tm1637_write_byte(status_led_bitmap); // only LED data
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0x8F); // display ON + brightness
    tm1637_stop();
}
//-------------------------------------------------------

//----------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------

void handle_trip(char n)
{
    if (startup_delay_active == 0)
    {
        ++trip_elapsed_ticks;

        /* Reuse common alternating voltage_control_error/tag display pattern to save code size. */
        #define SHOW_TRIP_ALT(VAL, tag_code)                    \
            do                                                       \
            {                                                        \
                ++trip_display_cycle;                                 \
                if (trip_display_cycle < 6)                           \
                {                                                     \
                    tm1637_display_number(VAL);                      \
                }                                                    \
                else if (trip_display_cycle < 12)                     \
                {                                                    \
                    tm1637_display_text(tag_code);                  \
                }                                                    \
                else                                                 \
                {                                                    \
                    trip_display_cycle = 0;                           \
                }                                                    \
            } while (0)

        switch (n)
        {

        case 1:
            P17 = 0;
            P01 = 1;
            P12 = 1;
            EA = 0;
            while (BTN_UP == 1)
            {
                tm1637_display_text(1);
                P00 = 1;
                Timer2_Delay(24000000, 13, 500);
                P00 = 0;
                if (voltage_control_error < 0)
                {
                    tm1637_display_text("INC");
                }
                else
                {
                    tm1637_display_text("DEC");
                }
                Timer2_Delay(24000000, 11, 700);
            }
            start_manual_restart_delay();
            EA = 1;
            break;

        case 2:
            
           
					if(trip_elapsed_ticks > 5) {
						set_status_led(LED_HILO, 1);
						SHOW_TRIP_ALT(input_voltage, "IPH");
					}
            if (trip_elapsed_ticks > hi_lo_trip_ticks)
            { // 40 9sec
                input_high_latched = 1;
                relay_enabled = 0;
                P00 = 0;
            }
            else
            {
                buzzer_on();
            }
            break;

        case 3:
            set_status_led(LED_OVL, 1);
            if (trip_elapsed_ticks > overload_trip_ticks)
            {
                overload_trip_current_display = (unsigned int)(output_current_rms + 0.5f);
                if (g_config.olr == 1)
                {
                    start_overload_auto_recovery(1);
                    output_current_rms = 0;
                    P00 = 0;
                    relay_enabled = 0;
                    display_page_index = 1;
                }
                else
                {
                    while (BTN_UP == 1)
                    {
                        tm1637_display_text("OL1");
                        buzzer_on();
                    }
                    start_manual_restart_delay();
                    output_current_rms = 0;
                    P00 = 0;
                    relay_enabled = 0;
                    display_page_index = 1;
                }
            }
            else
            {
                buzzer_on();
            }
            break;

        case 4:
           
           
					if(trip_elapsed_ticks > 5) {
						 set_status_led(LED_HILO, 1);
						 SHOW_TRIP_ALT(input_voltage, "IPL");
					}
            if (trip_elapsed_ticks > hi_lo_trip_ticks)
            { // 40 9sec
                input_low_latched = 1;
                relay_enabled = 0;
                P00 = 0;
            }
            else
            {
                buzzer_on();
            }
            break;
            break;

        case 5:
            
           
				
				if(trip_elapsed_ticks > 5) {
					 set_status_led(LED_HILO, 1);
				SHOW_TRIP_ALT(output_voltage, "OPL");
				}
				    
            if (trip_elapsed_ticks > hi_lo_trip_ticks)
            { // 40 9sec
                P00 = 0;
                output_low_latched = 1;
                relay_enabled = 0;
            }
            else
            {
                buzzer_on();
            }
            break;

        case 6:
            
           
					if(trip_elapsed_ticks > 5) {
						 set_status_led(LED_HILO, 1);
						SHOW_TRIP_ALT(output_voltage, "OPH");
					}
            if (trip_elapsed_ticks > hi_lo_trip_ticks)
            { // 40 9sec
                P00 = 0;
                output_high_latched = 1;
                relay_enabled = 0;
            }
            else
            {
                buzzer_on();
            }
            break;

        case 7:
   if (trip_elapsed_ticks > 4)
            {
            P17 = 0;
            P00 = 0;
            P01 = 1;
            P12 = 1;
            // EA = 0;
            set_status_led(LED_OVL, 1);
            overload_trip_current_display = (unsigned int)(output_current_rms + 0.5f);
            if (g_config.olr == 1)
            {
                start_overload_auto_recovery(2);
                output_current_rms = 0;
                relay_enabled = 0;
                display_page_index = 1;
            }
            else

            {
                while (BTN_UP == 1)
                {
                    tm1637_display_text("OL2");
                    buzzer_on();
                }
                start_manual_restart_delay();
                output_current_rms = 0;
                startup_delay_active = 1;
                relay_enabled = 0;
                display_page_index = 1;
            }
					}
            break;

            case 8:
            ++trip_display_cycle;

            
            if (trip_elapsed_ticks > 28)
            { // 40 9sec
                P17 = 0;
							//set_status_led(LED_OUT,0);
                P00 = 0;
                P01 = 1;
                P12 = 1;
               tm1637_display_text("C.F");

               while (BTN_UP == 1)
            {
                startup_timer_ticks = 0;
                while (startup_timer_ticks < 100)
                {
                    P00 = 1;
                }
                startup_timer_ticks = 0;
                while (startup_timer_ticks < 100)
                {
                    P00 = 0;
                }
            }
            start_manual_restart_delay();
            output_current_rms = 0;
            display_page_index = 1;
            }
            else if(trip_elapsed_ticks > 12)
            {
                buzzer_on();
            }
            break;
						
						case 9:
            SHOW_TRIP_ALT("E06", "E.F");
            if (trip_elapsed_ticks > hi_lo_trip_ticks)
            { // 40 9sec
                P00 = 0;
                relay_enabled = 0;
            }
            else
            {
                buzzer_on();
            }
            break;
        }

        #undef SHOW_TRIP_ALT
    }
}
//-----------------------HELPER FUNCTION-----------------------------------------
// write 15771

bit is_up_key_pressed()
{
    if (BTN_UP == 1 && up_key_latched == 0)
    {
        Timer2_Delay(24000000, 1, 100);
        up_key_latched = 1;
        menu_timeout_counter = 0;
        return 1;
    }
    if (BTN_UP == 0)
    {
        up_key_latched = 0;
        up_press_counter = 0;
    }
    return 0;
}

bit is_down_key_pressed()
{
    if (BTN_DOWN == 1 && down_key_latched == 0)
    {
        Timer2_Delay(24000000, 1, 100);
        down_key_latched = 1;
        menu_timeout_counter = 0;
        return 1;
    }
    if (BTN_DOWN == 0)
    {
        down_key_latched = 0;
        down_press_counter = 0;
    }
    return 0;
}
bit is_set_key_pressed()
{
    if (BTN_SET == 1 && set_key_latched == 0)
    {
        Timer2_Delay(24000000, 1, 100);
        set_key_latched = 1;
        menu_timeout_counter = 0;
        return 1;
    }
    if (BTN_SET == 0)
    {
        set_key_latched = 0;
    }
    return 0;
}

//--------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------

void update_display_cycle_selection()
{
    switch (g_config.display_mode)
    {

    case 1:
        display_page_start = 0;
        display_page_end = 0;
        break;
    case 2:
        display_page_start = 1;
        display_page_end = 1;
        break;
    case 3:
        display_page_start = 2;
        display_page_end = 2;
        break;
    case 4:
        display_page_start = 0;
        display_page_end = 2;
        break;
    }
}

//----------------------Buzer Count-------------------------------------------------

void buzzer_on()
{

    if (buzzer_cycle_ticks < ticks_per_second)
    {
        // printf("buzzer_output on \n");
        buzzer_output = 1;
    }

    else
    {
        buzzer_output = 0;
    }
    ++buzzer_cycle_ticks;

    if (buzzer_cycle_ticks > (ticks_per_second * 2))
    {
        buzzer_cycle_ticks = 0;
    }

    P00 = buzzer_output;
}

//----------------------------------------------------------------------------------

//-------------Timer0 Innetrupt At Every 20ms--------------------------------------
void timer0_init_2ms(void)
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
void timer0_isr(void) interrupt 1
{
    // Reload timer for next 2ms
    TH0 = 0xF0;
    TL0 = 0x60;

    display_tick_counter++;

    if (BTN_SET == 1) // assuming active low button
    {
        ++set_key_hold_ticks;
    }
    else
    {
        set_key_hold_ticks = 0;
    }

    if (set_key_hold_ticks > (ticks_per_second * 4))
    {
        display_update_ticks = 0;
        set_key_hold_ticks = 0;
        edit_value = 999;
        set_key_latched = 1;
        g_ui_state = AUT;
    }

    if (display_tick_counter >= display_update_ticks) // 1 ?? 2ms = 2ms
    {
        // printf("---------ok--------\n");
        // printf("ms counter = %d\n",display_tick_counter);
        display_tick_counter = 0;
        display_refresh_pending = 1;
    }
    ++startup_timer_ticks;

    if (startup_timer_ticks > power_on_delay_ticks)
    {
        if (startup_complete == 0)
        {
            if (startup_delay_profile == 0)
            {
                startup_complete = 1;
                startup_delay_active = 0;
                startup_delay_profile = 0;
                overload_recovery_seconds = 0;
                P00 = 0;
                // tm1637_display_text("IP");
            }
            else
            {
                start_manual_restart_delay();
            }
        }

        // printf("\n buzzer_output off flag 1 \n");
    }
    else if (startup_complete == 0)
    {
        if (startup_delay_profile != 0 && BTN_UP == 1)
        {
            start_manual_restart_delay();
            return;
        }

        buzzer_on();
        if (((startup_timer_ticks / ticks_per_second) & 1) == 0)
        {
            if (startup_delay_profile == 1)
            {
                tm1637_display_text("OL1");
            }
            else if (startup_delay_profile == 2)
            {
                tm1637_display_text("OL2");
            }
            else
            {
                tm1637_display_text("DLY");
            }
        }
        else
        {
            if (startup_delay_profile == 0)
            {
                tm1637_display_number(startup_countdown_seconds - (startup_timer_ticks / ticks_per_second));
            }
            else
            {
                tm1637_display_number(overload_trip_current_display);
            }
        }
        //(startup_countdown_seconds - (startup_timer_ticks/13))
    }
}

//------------------------update----------------------

void apply_config_to_runtime()
{

    update_display_cycle_selection();
    target_voltage = g_config.target_voltage;
    input_high_limit = g_config.input_high;
    input_low_limit = g_config.input_low;
    output_high_limit = g_config.output_high;
    output_low_limit = g_config.output_low;
    hi_lo_trip_ticks = (int)(g_config.hlt_delay * 4.44);
    regulation_band_high = g_config.regulation;
    regulation_band_low = g_config.regulation * (-1);
    overload_level1 = (g_config.overload_level1)/10.0;
    overload_level2 = ((g_config.overload_level1)/10.0) + 3.0;
    overload_trip_ticks = (int)(g_config.overload_trip_ticks * 4.44);
    power_on_delay_ticks = (int)(g_config.on_delay * 5);
    input_calibration = g_config.input_voltage_calibration;
    output_calibration = g_config.output_voltage_calibration;
    current_calibration = g_config.current_calibration_factor;
	earth_trip_threshold = g_config.earth_trip;
    ir_high = g_config.ir_high;
    ir_low = g_config.ir_low;
    auto_mode_enabled = g_config.auto_mode_status;
    
}

//-----------------------------------------------------------------------------------

//------------------------set_max_min_number-------------------------------------------

void set_edit_bounds(int max, int min)
{
    edit_value_max = max;
    edit_value_min = min;
}

static int16_t clamp_i16(int16_t value, int16_t min_v, int16_t max_v)
{
    if (value < min_v)
    {
        return min_v;
    }
    if (value > max_v)
    {
        return max_v;
    }
    return value;
}

/* Re-apply startup delay before reconnecting after voltage fault recovery. */
void start_reconnect_delay(void)
{
    power_on_delay_ticks = (unsigned int)(g_config.hlt_delay * 5);
    ticks_per_second = 5;
    startup_countdown_seconds = (unsigned char)(power_on_delay_ticks / 5);
    startup_delay_profile = 0;
    startup_complete = 0;
    startup_delay_active = 1;
    startup_timer_ticks = 0;
    relay_enabled = 0;
}

void start_manual_restart_delay(void)
{
    power_on_delay_ticks = (unsigned int)(g_config.on_delay * 5);
    ticks_per_second = 5;
    startup_countdown_seconds = (unsigned char)(power_on_delay_ticks / 5);
    startup_delay_profile = 0;
    startup_complete = 0;
    startup_delay_active = 1;
    startup_timer_ticks = 0;
    relay_enabled = 0;
}

void start_overload_auto_recovery(unsigned char profile)
{
    power_on_delay_ticks = 300U * 12U;
    ticks_per_second = 12;
    overload_recovery_seconds = 300;
    startup_delay_profile = profile;
    startup_complete = 0;
    startup_delay_active = 1;
    startup_timer_ticks = 0;
    relay_enabled = 0;
}

//-------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------

void main(void)
{

    MODIFY_HIRC(HIRC_24);
    // Enable_UART0_VCOM_printf_24M_115200();

    P13_QUASI_MODE;
    P14_QUASI_MODE;
    P12_PUSHPULL_MODE;
    P01_PUSHPULL_MODE;
    P17_PUSHPULL_MODE;
    P00_PUSHPULL_MODE;
    P15_INPUT_MODE;
    P11_INPUT_MODE;
    P10_INPUT_MODE;
	  P02_INPUT_MODE;
    P30_INPUT_MODE;

    P12 = 1;
    P01 = 1;
    P17 = 0;
    P00 = 0;

    if (load_config_from_flash() == 0)
    {
        // first time defaults
        g_config.target_voltage = 230;
        g_config.on_delay = 10;
        g_config.input_low = 190;
        g_config.input_high = 270;
        g_config.output_low = 210;
        g_config.output_high = 270;
        g_config.display_mode = 4;
        g_config.hlt_delay = 5;
        g_config.contactor_monitor_enabled = 0;
        g_config.regulation = 3;
        g_config.olr = 1;
        g_config.overload_level1 = 100.0;
        g_config.overload_trip_ticks = 10;
        g_config.overload_level2 = 130.0;
        g_config.input_voltage_calibration = 1.0; // 0.92
        g_config.output_voltage_calibration = 1.0; // 0.92
        g_config.current_calibration_factor = 1.0;
        g_config.auto_mode_status = 1;
			  g_config.contactor_monitor_enabled = 1;
			g_config.earth_monitor_enabled = 0;
			g_config.earth_trip = 15;
                g_config.ir_high = 270;
                g_config.ir_low = 150;

        save_config_to_flash(); // store defaults
    }
    apply_config_to_runtime();
    startup_countdown_seconds = (int)(power_on_delay_ticks / 5);
    input_voltage = (int)(measure_ac_voltage_rms(1) * input_calibration);
    timer0_init_2ms();

    // tm1637_display_text("IP");

    while (1)
    {
        EA = 0;
        output_voltage = (int)(measure_ac_voltage_rms(0) * output_calibration);

        if (output_voltage < 100 || output_voltage > 1000)
        {
            output_voltage = 0;
        }
        voltage_control_error = (int)(output_voltage - target_voltage);

        if (voltage_control_error >= regulation_band_high && auto_mode_enabled)
        {
            P12 = 1;
            P01 = 0;
            error_flag = 1;
        }
        else if (voltage_control_error <= regulation_band_low && auto_mode_enabled)
        {
            P01 = 1;
            P12 = 0;
            error_flag = 1;
        }
        else if (auto_mode_enabled == 1)
        {
            P12 = 1;
            P01 = 1;
            error_flag = 0;
            // error_count_ticks = 0;
        }
        if ((error_flag == 0 || voltage_control_error < (-220) || error_count_ticks > 50) && startup_delay_active == 0)
        {
            power_on_delay_ticks = (int)(g_config.on_delay * 5);
            ticks_per_second = 5;
            startup_countdown_seconds = (int)(power_on_delay_ticks / 5);
            if (voltage_scan_step == 1)
            {
                input_voltage = (int)(measure_ac_voltage_rms(1) * input_calibration);
                if (input_voltage < 100 || input_voltage > 1000)
                {
                    input_voltage = 0;
                }
                ++voltage_scan_step;
            }

            else if (voltage_scan_step == 2)
            {
                contactor_voltage = (int)(measure_ac_voltage_rms(2));
                
                ++voltage_scan_step;
            }
            else if (voltage_scan_step == 3)
            {
                earth_voltage = (int)(measure_ac_voltage_rms(3));
                
                voltage_scan_step = 1;
            }

            output_current_rms = (measure_ac_current_rms()) * current_calibration;

            if (output_current_rms < 0.8)
            {
                output_current_rms=0;
            }

            EA = 1;

            if (error_count_ticks > 110 && input_voltage > ir_low && input_voltage < ir_high)
            {
                handle_trip(1); // motor side detection
                error_count_ticks = 0;
            }

            if (error_count_ticks > 50)
            {

                ++error_count_ticks;
            }
            else
            {
                error_count_ticks = 0;
            }
            // error_count_ticks = 0;
        }
        else if (startup_delay_active == 1)
        {
            if (startup_delay_profile == 0)
            {
                power_on_delay_ticks = (int)(g_config.on_delay * 12);
                ticks_per_second = 12;
                startup_countdown_seconds = (int)(power_on_delay_ticks / 12);
            }
            else
            {
                ticks_per_second = 5;
            }
            EA = 1;
        }
        else
        {
            if (error_count_ticks > 20)
                P00 = 0;
            ++error_count_ticks;
        }

        // printf("The voltage is %d \n", output_voltage); // For output voltage print
        //  printf("The current is %0.3f \n",output_current_rms);
        //  printf("The voltage_control_error is %d \n",(int)voltage_control_error); //For Error Print
        //  if (output_voltage > OUTPUT_LOW && output_voltage < OUTPUT_HIGH && input_voltage < INPUT_HIGH && input_voltage > INPUT_LOW && output_current_rms < OVERCURRENT && startup_complete==1)

        //-----------------------manual------------

        if (auto_mode_enabled == 0 && g_ui_state == UI_NORMAL)
        {
            ++manual_led_tick;
            if (manual_led_tick >= 20)
            {
                manual_led_tick = 0;
            }

            set_status_led(LED_DELAY, manual_led_tick < 10);

            if (BTN_UP == 1)
            {
                P01 = 1;
                P12 = 0;
            }

            else if (BTN_DOWN == 1)
            {
                P12 = 1;
                P01 = 0;
            }

            else
            {
                P12 = 1;
                P01 = 1;
            }
        }
        else
        {
            manual_led_tick = 0;
            set_status_led(LED_DELAY, 0);
        }

        //------------------------------------

        if (output_voltage > output_high_limit && output_high_latched == 0)
        {
            error_active = 1;
            handle_trip(6);
        }
        else if (output_voltage < output_low_limit && output_low_latched == 0)
        {
            error_active = 1;
            handle_trip(5);
        }
        else if (input_voltage > input_high_limit && input_high_latched == 0)
        {
            error_active = 1;
            handle_trip(2);
        }
        else if (input_voltage < input_low_limit && input_low_latched == 0)
        {
            error_active = 1;
            handle_trip(4);
        }
        
        else if (output_current_rms > overload_level2)
        {
            handle_trip(7);
        }
        else if (output_current_rms > overload_level1)
        {
            handle_trip(3);
        }
        else if(startup_complete == 1 && relay_enabled == 1 && output_voltage > 180 && contactor_voltage < 100 && g_config.contactor_monitor_enabled == 1) {

              handle_trip(8);
        }
				else if(g_config.earth_monitor_enabled == 1 && earth_voltage > earth_trip_threshold) {

              handle_trip(9);
        }
        else if (startup_complete == 1)
        {
            trip_elapsed_ticks = 0;
            error_active = 0;
            P00 = 0;

            if (input_high_latched == 1)
            {

                if (input_voltage < (input_high_limit - 20))
                {
                    start_reconnect_delay();
                    set_status_led(LED_HILO, 0);
                    input_high_latched = 0;
                }
            }

            else if (input_low_latched == 1)
            {
                if (input_voltage > (input_low_limit + 25))
                {
                    start_reconnect_delay();
                    set_status_led(LED_HILO, 0);
                    input_low_latched = 0;
                }
            }

            else if (output_high_latched == 1)
            {

                if (output_voltage < (output_high_limit - 10))
                {
                    start_reconnect_delay();
                    set_status_led(LED_HILO, 0);
                    output_high_latched = 0;
                }
            }

            else if (output_low_latched == 1)
            {

                if (output_voltage > (output_low_limit + 10))
                {
                    start_reconnect_delay();
                    set_status_led(LED_HILO, 0);
                    output_low_latched = 0;
                }
            }
            else
            {
                relay_enabled = 1;
							   set_status_led(LED_HILO, 0);
							    set_status_led(LED_OVL, 0);
            }
        }

        else
        {
            error_active = 0;
            trip_elapsed_ticks = 0;
        }

        P17 = relay_enabled;
        //set_status_led(LED_OUT, relay_enabled);

        if (display_refresh_pending && startup_delay_active == 0)
        {
            display_refresh_pending = 0;

            if (g_ui_state == UI_NORMAL && error_active == 0)
            {
                set_status_led(LED_AUTO, g_config.auto_mode_status);

                switch (display_page_index)
                {

                
                case 0:	  
								     set_status_led(LED_OP,0);
								     set_status_led(LED_OA, 0);
								     set_status_led(LED_IP, 1);
                    tm1637_display_number((unsigned int)input_voltage);
                    break;
               
                case 1:
                     
								     set_status_led(LED_OA, 0);
								     set_status_led(LED_IP, 0);
								     set_status_led(LED_OP,1);
                    tm1637_display_number((unsigned int)output_voltage);
                    break;
              
                case 2:
									   set_status_led(LED_IP, 0);
								     set_status_led(LED_OP,0);
									   set_status_led(LED_OA, 1);
								     
                    tm1637_display_current(output_current_rms);
                    break;
                }

                display_page_index++;
                if (display_page_index > display_page_end)
                {
                    display_page_index = display_page_start;
                }
            }
            else
            {
                ++menu_timeout_counter;
							
							if(BTN_HOME == 1 && home_state == 0) {
                                home_state = 1;
							    g_ui_state = UI_NORMAL;
                                save_config_to_flash();
                                display_update_ticks = 20;
							
							}
                            if(BTN_HOME == 0) {
                                home_state = 0;
                            }
                switch (g_ui_state)
                {

                case AUT:
                    tm1637_display_text("AUT");
                    if (is_set_key_pressed())
                    {

                        g_ui_state = AUT_ON;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = US;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = UI_EXIT;
                    }
                    break;
                case AUT_ON:
                    tm1637_display_text(STR_ON);
                    if (is_set_key_pressed())
                    {

                        g_config.auto_mode_status = 1;
                        auto_mode_enabled = 1;
                        set_status_led(LED_AUTO, g_config.auto_mode_status);
                        g_ui_state = AUT;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = AUT_OFF;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = AUT_OFF;
                    }
                    break;

                case AUT_OFF:
                    tm1637_display_text(STR_OFF);
                    if (is_set_key_pressed())
                    {

                        g_config.auto_mode_status = 0;
                        auto_mode_enabled = 0;
                        set_status_led(LED_AUTO, g_config.auto_mode_status);
                        g_ui_state = AUT;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = AUT_ON;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = AUT_ON;
                    }
                    break;

                case US:
                    tm1637_display_text("U.ST"); // U.St
                    if (is_set_key_pressed())
                    {

                        edit_value = 0;
                        set_edit_bounds(1000, 0);
                        g_ui_state = US_PASSWORD;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = FS;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = AUT;
                    }
                    break;

                case US_PASSWORD:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed() && edit_value == 5)
                    {

                        g_ui_state = SOP;
                    }
                    break;

                case SOP:
                    tm1637_display_text("S.OP"); // S.OP
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.target_voltage;
                        set_edit_bounds(MAX_TARGET_VOLTAGE, MIN_TARGET_VOLTAGE);
                        g_ui_state = TARGET_VOLTAGE;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = POD;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = UI_US_EXIT;
                    }
                    break;

                case TARGET_VOLTAGE:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {
                        int16_t old_target = target_voltage;
                        int16_t new_target = edit_value;
                        int16_t delta = new_target - old_target;

                        g_config.target_voltage = new_target;
                        target_voltage = new_target;

                        g_config.input_low = clamp_i16(g_config.input_low + delta,
                                                    MIN_INPUT_LOW,
                                                    target_voltage - MAX_INPUT_LOW);

                        g_config.input_high = clamp_i16(g_config.input_high + delta,
                                                     target_voltage + MIN_INPUT_HIGH,
                                                     MAX_INPUT_HIGH);

                        g_config.output_low = clamp_i16(g_config.output_low + delta,
                                                     MIN_OUTPUT_LOW,
                                                     target_voltage - MAX_OUTPUT_LOW);

                        g_config.output_high = clamp_i16(g_config.output_high + delta,
                                                      target_voltage + MIN_OUTPUT_HIGH,
                                                      MAX_OUTPUT_HIGH);

                        input_low_limit = g_config.input_low;
                        input_high_limit = g_config.input_high;
                        output_low_limit = g_config.output_low;
                        output_high_limit = g_config.output_high;
                        g_ui_state = SOP;
                    }
                    break;

                case POD:
                    tm1637_display_text("DLY"); // DLY
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.on_delay;
                        set_edit_bounds(MAX_ON_DELAY, MIN_ON_DELAY);
                        g_ui_state = ON_DELAY_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = IPL;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = SOP;
                    }
                    break;

                case ON_DELAY_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.on_delay = edit_value;
                        power_on_delay_ticks = (int)(g_config.on_delay * 4.16);
                        g_ui_state = POD;
                    }
                    break;

                case IPL:
                    tm1637_display_text("IP.L"); // IP.L
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.input_low;
                        set_edit_bounds(target_voltage - MAX_INPUT_LOW, MIN_INPUT_LOW);
                        g_ui_state = INPUT_LOW_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = IN_HIGH;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = POD;
                    }
                    break;

                case INPUT_LOW_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.input_low = edit_value;
                        input_low_limit = edit_value;
                        g_ui_state = IPL;
                    }
                    break;

                case IN_HIGH:
                    tm1637_display_text("IP.H"); // IP.H
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.input_high;
                        set_edit_bounds(MAX_INPUT_HIGH, target_voltage + MIN_INPUT_HIGH);
                        g_ui_state = INPUT_HIGH_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OPL;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = IPL;
                    }
                    break;

                case INPUT_HIGH_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.input_high = edit_value;
                        input_high_limit = edit_value;
                        g_ui_state = IN_HIGH;
                    }
                    break;

                case OPL:
                    tm1637_display_text("OP.L"); // OP.L
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.output_low;
                        set_edit_bounds(target_voltage - MAX_OUTPUT_LOW, MIN_OUTPUT_LOW);
                        g_ui_state = OUTPUT_LOW_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OPH;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = IN_HIGH;
                    }
                    break;

                case OUTPUT_LOW_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.output_low = edit_value;
                        output_low_limit = edit_value;
                        g_ui_state = OPL;
                    }
                    break;

                case OPH:
                    tm1637_display_text("OP.H"); // OP.H
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.output_high;
                        set_edit_bounds(MAX_OUTPUT_HIGH, target_voltage + MIN_OUTPUT_HIGH);
                        g_ui_state = OUTPUT_HIGH_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = HLT;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OPL;
                    }
                    break;

                case OUTPUT_HIGH_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.output_high = edit_value;
                        output_high_limit = edit_value;
                        g_ui_state = OPH;
                    }
                    break;

                case HLT:
                    tm1637_display_text("HL.T"); // HL.T
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.hlt_delay;
                        set_edit_bounds(60, 0);
                        g_ui_state = HLT_DELAY_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = DIS;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OPH;
                    }
                    break;

                case HLT_DELAY_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.hlt_delay = edit_value;
                        hi_lo_trip_ticks = (int)(edit_value * 4.44);
                        g_ui_state = HLT;
                    }
                    break;

                case DIS:
                    tm1637_display_text("DIS");
                    if (is_set_key_pressed())
                    {

                        edit_value = 5;
                        g_ui_state = INPUT_VOLTAGE;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = EARTH;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = HLT;
                    }
                    break;

                case INPUT_VOLTAGE:
                    tm1637_display_text("IP");
                    if (is_set_key_pressed())
                    {

                        g_config.display_mode = 1;
                        update_display_cycle_selection();
                        g_ui_state = DIS;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OP;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = ALL;
                    }
                    break;

                case OP:
                    tm1637_display_text("OP");
                    if (is_set_key_pressed())
                    {

                        g_config.display_mode = 2;
                        update_display_cycle_selection();
                        g_ui_state = DIS;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OA;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = IP;
                    }
                    break;

                case OA:
                    tm1637_display_text("CUR"); // CUR
                    if (is_set_key_pressed())
                    {

                        g_config.display_mode = 3;
                        update_display_cycle_selection();
                        g_ui_state = DIS;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = ALL;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OP;
                    }
                    break;

                case ALL:
                    tm1637_display_text("ALL");
                    if (is_set_key_pressed())
                    {

                        g_config.display_mode = 4;
                        update_display_cycle_selection();
                        g_ui_state = DIS;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = INPUT_VOLTAGE;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OA;
                    }
                    break;

                case FS:
                    tm1637_display_text("F.ST"); // F.ST
                    if (is_set_key_pressed())
                    {

                        edit_value = 999;
                        set_edit_bounds(1000, 0);
                        g_ui_state = FS_PASSWORD;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = UI_EXIT;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = US;
                    }
                    break;

                case FS_PASSWORD:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed() && edit_value == 998)
                    {

                        g_ui_state = REG;
                    }
                    break;

                case REG:
                    tm1637_display_text("S.RG"); // S.RG
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.regulation;
                        set_edit_bounds(MAX_REG, MIN_REG);
                        g_ui_state = REGULATION_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OLR;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = UI_FS_EXIT;
                    }
                    break;

                case REGULATION_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.regulation = edit_value;
                        regulation_band_high = edit_value;
                        regulation_band_low = (-1) * edit_value;
                        g_ui_state = REG;
                    }
                    break;

                case OLR:
                    tm1637_display_text("OL.R"); // OL.R
                    if (is_set_key_pressed())
                    {

                        g_ui_state = OLR_ON;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OL1;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = REG;
                    }
                    break;
                case OLR_ON:
                    tm1637_display_text("AT");
                    if (is_set_key_pressed())
                    {

                        g_config.olr = 1;
                        g_ui_state = OLR;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OLR_OFF;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OLR_OFF;
                    }
                    break;
                case OLR_OFF:
                    tm1637_display_text("MN");
                    if (is_set_key_pressed())
                    {

                        g_config.olr = 0;
                        g_ui_state = OLR;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OLR_ON;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OLR_ON;
                    }
                    break;

                case OL1:
                    tm1637_display_text("S.OL"); // S.OL
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.overload_level1;
                        set_edit_bounds(MAX_OL1, MIN_OL1);
                        g_ui_state = OL1_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OLT;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OLR;
                    }
                    break;

                case OL1_SET:
                    tm1637_display_current((float)edit_value / 10);
                    if (is_set_key_pressed())
                    {

                        g_config.overload_level1 = edit_value;
                        overload_level1 = (float)edit_value/10.0;
                        g_config.overload_level2 = edit_value + 30;
                        g_ui_state = OL1;
                    }
                    break;

                case OLT:
                    tm1637_display_text("OL.T"); // OL.T
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.overload_trip_ticks;
                        set_edit_bounds(MAX_OL1_TIME, MIN_OL1_TIME);
                        g_ui_state = OLT_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OL2;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OL1;
                    }
                    break;

                case OLT_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.overload_trip_ticks = edit_value;
                        overload_trip_ticks = (int)(g_config.overload_trip_ticks * 4.44);
                        g_ui_state = OLT;
                    }
                    break;

                case OL2:
                    tm1637_display_text("OL2");
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.overload_level2;
                        g_ui_state = OL2_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = IPC;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OLT;
                    }
                    break;

                case OL2_SET:
                    tm1637_display_current((float)edit_value / 10);
                    if (is_set_key_pressed())
                    {

                        g_config.overload_level2 = edit_value;
                        overload_level2 = (float)edit_value/10.0;
                        g_ui_state = OL2;
                    }
                    break;

                case IPC:
                    tm1637_display_text("C.IP"); // C.IP
                    if (is_set_key_pressed())
                    {

                        input_voltage_raw = (int)(measure_ac_voltage_rms(1));
                        edit_value = input_voltage_raw;
                        set_edit_bounds(1000, 0);
                        g_ui_state = IPC_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OPC;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OL2;
                    }
                    break;

                case IPC_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.input_voltage_calibration = (((float)edit_value) / input_voltage_raw);
                        input_calibration = g_config.input_voltage_calibration;
                        g_ui_state = IPC;
                    }
                    break;

                case OPC:
                    tm1637_display_text("C.OP"); // C.OP
                    if (is_set_key_pressed())
                    {

                        output_voltage_raw = (int)(measure_ac_voltage_rms(0));
                        edit_value = 230;
                        set_edit_bounds(1000, 0);
                        g_ui_state = OPC_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = OAC;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = IPC;
                    }
                    break;

                case OPC_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.output_voltage_calibration = (float)(((float)edit_value) / output_voltage_raw);
                        output_calibration = g_config.output_voltage_calibration;
                        g_ui_state = OPC;
                    }
                    break;

                case OAC:
                    tm1637_display_text("C.CU"); // C.CU
                    if (is_set_key_pressed())
                    {

                        edit_value = measure_ac_current_rms();
                        set_edit_bounds(1000, 0);
                        g_ui_state = OAC_SET;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = CON;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OPC;
                    }
                    break;

                case OAC_SET:
                    // tm1637_display_number((unsigned int)edit_value);
                    tm1637_display_current((float)edit_value / 10);
                    if (is_set_key_pressed())
                    {
                        if (output_current_rms > 0.01)
                        {
                            g_config.current_calibration_factor = (float)edit_value / (output_current_rms * 10);
                        }
                        current_calibration = g_config.current_calibration_factor;
                        g_ui_state = OAC;
                    }
                    break;

                case CON:
                    tm1637_display_text("CON");
                    if (is_set_key_pressed())
                    {

                        g_ui_state = CON_ON;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = EARTH_HIGH;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = OAC;
                    }
                    break;
                case CON_ON:
                    tm1637_display_text(STR_ON);
                    if (is_set_key_pressed())
                    {

                        g_config.contactor_monitor_enabled = 1;
                        g_ui_state = CON;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = CON_OFF;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = CON_OFF;
                    }
                    break;
                case CON_OFF:
                    tm1637_display_text(STR_OFF);
                    if (is_set_key_pressed())
                    {

                        g_config.contactor_monitor_enabled = 0;
                        g_ui_state = CON;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = CON_ON;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = CON_ON;
                    }
                    break;

					case EARTH:
                    tm1637_display_text("S.ER");
                    if (is_set_key_pressed())
                    {

                        g_ui_state = EARTH_ON;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = UI_US_EXIT;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = DIS;
                    }
                    break;
                case EARTH_ON:
                    tm1637_display_text(STR_ON);
                    if (is_set_key_pressed())
                    {

                        g_config.earth_monitor_enabled = 1;
                        g_ui_state = EARTH;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = EARTH_OFF;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = EARTH_OFF;
                    }
                    break;
                case EARTH_OFF:
                    tm1637_display_text(STR_OFF);
                    if (is_set_key_pressed())
                    {

                         g_config.earth_monitor_enabled = 0;
                        g_ui_state = EARTH;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = EARTH_ON;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = EARTH_ON;
                    }
                    break;
					case EARTH_HIGH:
                    tm1637_display_text("E.HI"); // OP.H
                    if (is_set_key_pressed())
                    {

                        edit_value = g_config.earth_trip;
                        set_edit_bounds(50,10);
                        g_ui_state = EARTH_HIGH_SET;
                    }
                    if (is_up_key_pressed())
                    {

                            g_ui_state = IR_HIGH;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = CON;
                    }
                    break;

                case EARTH_HIGH_SET:
                    tm1637_display_number((unsigned int)edit_value);
                    if (is_set_key_pressed())
                    {

                        g_config.earth_trip = edit_value;
                        earth_trip_threshold = edit_value;
                        g_ui_state = EARTH_HIGH;
                    }
                    break;

                    case IR_HIGH:
                        tm1637_display_text("IR.H");
                        if (is_set_key_pressed())
                        {

                            edit_value = g_config.ir_high;
                            set_edit_bounds(300,250);
                            g_ui_state = IR_HIGH_SET;
                        }
                        if (is_up_key_pressed())
                        {

                            g_ui_state = IR_LOW;
                        }
                        if (is_down_key_pressed())
                        {

                            g_ui_state = EARTH_HIGH;
                        }
                        break;

                    case IR_HIGH_SET:
                        tm1637_display_number((unsigned int)edit_value);
                        if (is_set_key_pressed())
                        {

                            g_config.ir_high = edit_value;
                            ir_high = edit_value;
                            g_ui_state = IR_HIGH;
                        }
                        break;

                    case IR_LOW:
                        tm1637_display_text("IR.L");
                        if (is_set_key_pressed())
                        {

                            edit_value = g_config.ir_low;
                            set_edit_bounds(190, 140);
                            g_ui_state = IR_LOW_SET;
                        }
                        if (is_up_key_pressed())
                        {

                            g_ui_state = UI_FS_EXIT;
                        }
                        if (is_down_key_pressed())
                        {

                            g_ui_state = IR_HIGH;
                        }
                        break;

                    case IR_LOW_SET:
                        tm1637_display_number((unsigned int)edit_value);
                        if (is_set_key_pressed())
                        {

                            g_config.ir_low = edit_value;
                            ir_low = edit_value;
                            g_ui_state = IR_LOW;
                        }
                        break;

                case UI_EXIT:
                    tm1637_display_text("-E-");
                    if (is_set_key_pressed())
                    {

                        settings_page_active = 0;
                        save_config_to_flash();
                        g_ui_state = UI_NORMAL;
                        display_update_ticks = 20;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = AUT;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = FS;
                    }
                    break;

                case UI_US_EXIT:
                    tm1637_display_text(STR_END);
                    if (is_set_key_pressed())
                    {

                        g_ui_state = US;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = SOP;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = DIS;
                    }
                    break;

                case UI_FS_EXIT:
                    tm1637_display_text(STR_END);
                    if (is_set_key_pressed())
                    {

                        g_ui_state = FS;
                    }
                    if (is_up_key_pressed())
                    {

                        g_ui_state = REG;
                    }
                    if (is_down_key_pressed())
                    {

                        g_ui_state = IR_LOW;
                    }
                }

                if (BTN_UP == 1)
                {
                    ++up_press_counter;
                    menu_timeout_counter = 0;
                    if (up_press_counter > 30 && edit_value < edit_value_max)
                    {
                        ++edit_value;
                    }
                    else if (up_press_counter % 2 == 0 && edit_value < edit_value_max)
                    {
                        ++edit_value;
                    }
                }
                else if (BTN_DOWN == 1)
                {
                    ++down_press_counter;
                    menu_timeout_counter = 0;
                    if (down_press_counter > 30 && edit_value > edit_value_min)
                    {
                        --edit_value;
                    }
                    else if (down_press_counter % 2 == 0 && edit_value > edit_value_min)
                    {
                        --edit_value;
                    }
                }
                if (BTN_UP == 0)
                {
                    up_press_counter = 0;
                }
                if (BTN_DOWN == 0)
                {
                    down_press_counter = 0;
                }
                if (menu_timeout_counter > 130)
                {
                    menu_timeout_counter = 0;
                    save_config_to_flash();
                    display_update_ticks = 20;
                    g_ui_state = UI_NORMAL;
                }
            }
        }

        // Timer0_Delay(24000000,300,1000);
    }
}
