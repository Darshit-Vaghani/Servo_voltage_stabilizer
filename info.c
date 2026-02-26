#include "FreeRTOS.h"
#include "LiquidCrystal_F28E12x.h"
#include "board.h"
#include "c2000_freertos.h"
#include "device.h"
#include "driverlib.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <i2cLib_FIFO_polling.h>
#include <string.h>
#include <stdbool.h>

// ================= EEPROM CONFIG =================
#define EEPROM_I2C_ADDR     0x50
#define EEPROM_PAGE_SIZE   8      // AT24C02 = 8 bytes
#define EEPROM_START_ADDR  0x00
#define chek_status 5
// =================================================

// ================= I2C GLOBALS =================
struct I2CHandle EEPROM;

uint8_t TX_MsgBuffer[MAX_BUFFER_SIZE];
uint8_t RX_MsgBuffer[MAX_BUFFER_SIZE];
uint32_t ControlAddr;


//----------------------Defines-------------------------------------------------

#define ADC_MAX 4095.0
#define ADC_REF_VOLT 3.33
#define ADC_OFFSET 1.67
#define GAIN_FACTOR 0.002222

#define CURRENT_ADC_OFFSET 1.68
#define CURRENT_GAIN_FACTOR 0.01508 // 2388:1 CT + 36Ω

#define SAMPLE_COUNT 369 // 738 2 cycle
#define OSR 7
#define inc 3
#define dec 2
#define buzzer 4
#define relay 5
#define V_ZC 6 // Zero-cross input (HIGH in negative cycle)
#define C_ZC 7
#define ZC_TIMEOUT 10000 // adjust based on CPU speed

#define UP 12
#define DOWN 24
#define HOME 242
#define SET 13

int target = 230,v_in,v_o;
float c;
int error = 0;
char d1 = 0, d2 = 0, d3 = 0, d4 = 0;
bool onoff_temp = 0,buzer_state = 0, start_flag = 0,relay_flag=0,v_in_high=0,v_in_low=0,op_high=0,op_low=0;
uint64_t time = 0;
unsigned int buzer_count = 0;
uint8_t display_error = 0;

//------------------------------UI-------------------------------------------------------------------

typedef enum {
  NORMAL,
  TRIP_LOGS,
  MANUAL_MODE,
  SETTING,
  FACT_SETTING,
  CALIBRATION,
  REGULATION,
  OP_CURRENT,
  OVERLOAD,
  OVLD_TIME,
  OVLD_IMD,
  OVLD_RESET,
  EARTH_FAIL,
  EARTH_HIGH,
  EPOINT,
  MOTOR_FAULT,
  FACT_PASSWORD,
  EDIT,
  EDIT_ONOFF,
  SETTING_PASSWORD,
  SET_VOLTAGE,
  ON_DELAY,
  IP_LOW,
  IP_HIGH,
  OP_LOW,
  OP_HIGH,
  HI_LO_TIME,
  CONTACTOR_FAIL,
  OVLD_RESET_AUTO,
  IP_CALIBRATION,
  OP_CALIBRATION,
  EARTH_CALIBRATION,
  OA_CALIBRATION,
  CALIBRATION_PASSWORD
} UI_state;

UI_state pre_state = NORMAL;
UI_state display_state = NORMAL;
//----------------------------------------------------------------------------------------------------

//------------------MUTEX---------------------------

typedef struct {
  int voltage_out;
  float current;
  int Frequncy;
  int voltage_in;
  int motor_current;
  int earth_voltage;
  int contactor_v;
} StabilizerData_t;

typedef struct {

  uint8_t reg;
  bool auto_onoff;
  int set_volt;
  uint8_t ip_cal;
  uint8_t op_cal;
  uint8_t oa_cal;
} parameter_required_controlltask_t;

parameter_required_controlltask_t required_parameter;
StabilizerData_t gData;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t parameter_mutex;

//---------------------------------------------------------------

//-------------------data-store----------------------------------

typedef struct {
  bool Mode;
  uint8_t regulation_voltage;
  bool op_current;
  uint8_t overload_current;
  int ovld_time_s;
  uint8_t ovld_imd_current;
  bool ovld_reset_onoff;
  bool earth_fail_onoff;
  bool contactor_fail_onoff;
  uint8_t earth_high_voltage;
  bool epoint_fail;
  bool motor_fault_onoff;
  uint16_t target_output_voltage;
  uint16_t upper_target_voltage;
  uint8_t ondelay_s;
  uint16_t ip_low_voltage;
  uint16_t ip_high_voltage;
   uint16_t op_high_voltage;
  uint16_t op_low_voltage;
  uint16_t upper_ip_low_voltage;
  uint16_t upper_ip_high_voltage;
   uint16_t upper_op_high_voltage;
  uint16_t upper_op_low_voltage;
  uint8_t hi_lo_time_s;
  uint8_t input_calibration;
  uint8_t output_calibration;
  uint8_t current_calibration;
  uint8_t confirmation;
} parameters;

parameters store_data;

// ================= DEFAULT PARAMETERS =================
void load_default_parameters(parameters *p)
{
    p->Mode = 1;
    p->regulation_voltage = 3;
    p->op_current = 1;
    p->overload_current = 10;
    p->ovld_time_s = 5;
    p->ovld_imd_current = 15;
    p->ovld_reset_onoff = true;
    p->earth_fail_onoff = 0;
    p->contactor_fail_onoff = true;
    p->earth_high_voltage = 25;
    p->epoint_fail = false;
    p->motor_fault_onoff = false;
    p->target_output_voltage = 230;
    p->ondelay_s = 10;
    p->ip_low_voltage = 180;
    p->ip_high_voltage = 250;
    p->op_high_voltage = 250;
    p->op_low_voltage = 180;
    p->hi_lo_time_s = 5;
    p->input_calibration = 100;
    p->output_calibration = 104;
    p->current_calibration = 100;
    p->confirmation = chek_status;
    p->upper_ip_high_voltage = 30;
    p->upper_op_high_voltage = 30;
    p->upper_ip_low_voltage = 0;
    p->upper_op_low_voltage =0;
    p->upper_target_voltage = 0;
}
uint16_t i ;
// ================= EEPROM SAVE =================
int save_parameters(parameters *p, uint16_t startAddr)
{
    uint16_t total = sizeof(parameters);
    memcpy(TX_MsgBuffer, p, total);

    uint16_t offset = 0;
    uint16_t status;

    while (offset < total)
    {
        uint16_t chunk = (total - offset > EEPROM_PAGE_SIZE)
                        ? EEPROM_PAGE_SIZE
                        : total - offset;

        ControlAddr = startAddr + offset;
        EEPROM.pTX_MsgBuffer = &TX_MsgBuffer[offset];
        EEPROM.NumOfDataBytes = chunk;

        //uartPrintf("\nWRITE addr=0x%02X len=%u\n", ControlAddr, chunk);

        for (i= 0; i < chunk; i++)
            uartPrintf("  TX[%u] = %u\n", offset + i, TX_MsgBuffer[offset + i]);

        status = I2C_ControllerTransmitter(&EEPROM);
        //uartPrintf("  status = %u\n", status);

        DEVICE_DELAY_US(EEPROM.WriteCycleTime_in_us);
        offset += chunk;
    }
    return 0;
}

// ================= EEPROM LOAD =================
int load_parameters(parameters *p, uint16_t startAddr)
{
    uint16_t total = sizeof(parameters);
    uint16_t offset = 0;
    uint16_t status;

    while (offset < total)
    {
        uint16_t chunk = (total - offset > EEPROM_PAGE_SIZE)
                        ? EEPROM_PAGE_SIZE
                        : total - offset;

        ControlAddr = startAddr + offset;

        EEPROM.NumOfDataBytes = 0;
        status = I2C_ControllerTransmitter(&EEPROM);
        DEVICE_DELAY_US(50);

        EEPROM.NumOfDataBytes = chunk;
        EEPROM.pRX_MsgBuffer = &RX_MsgBuffer[offset];
        status = I2C_ControllerReceiver(&EEPROM);

        uartPrintf("\nREAD addr=0x%02X len=%u\n", ControlAddr, chunk);

        for (i = 0; i < chunk; i++)
            uartPrintf("  RX[%u] = %u\n", offset + i, RX_MsgBuffer[offset + i]);

        offset += chunk;
    }

    memcpy(p, RX_MsgBuffer, total);

    if (p->confirmation != chek_status)
    {
        uartPrintf("\nINVALID EEPROM DATA → loading defaults\n");
        load_default_parameters(p);
        return 1;
    }

    uartPrintf("\nEEPROM DATA VALID\n");
    return 0;
}

// ================= I2C INIT =================
void I2C_GPIO_init(void)
{
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(0, GPIO_QUAL_ASYNC);

    GPIO_setDirectionMode(1, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(1, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_0_I2CA_SDA);
    GPIO_setPinConfig(GPIO_1_I2CA_SCL);
}

void I2Cinit(void)
{
    I2C_disableModule(I2CA_BASE);
    I2C_initController(I2CA_BASE, DEVICE_SYSCLK_FREQ, 100000, I2C_DUTYCYCLE_50);
    I2C_setTargetAddress(I2CA_BASE, EEPROM_I2C_ADDR);
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);
    I2C_enableFIFO(I2CA_BASE);
    I2C_enableModule(I2CA_BASE);

    EEPROM.base = I2CA_BASE;
    EEPROM.TargetAddr = EEPROM_I2C_ADDR;
    EEPROM.pControlAddr = &ControlAddr;
    EEPROM.NumOfAddrBytes = 1;
    EEPROM.pTX_MsgBuffer = TX_MsgBuffer;
    EEPROM.pRX_MsgBuffer = RX_MsgBuffer;
    EEPROM.NumOfAttempts = 5;
    EEPROM.Delay_us = 1000;
    EEPROM.WriteCycleTime_in_us = 10000;
}

/*
parameters store_data = {.Mode = true,
                         .regulation_voltage = 2,
                         .op_current = true,
                         .overload_current = 10,
                         .ovld_time_s = 5,
                         .ovld_imd_current = 15,
                         .earth_fail_onoff = true,
                         .earth_high_voltage = 30,
                         .epoint_fail = false,
                         .motor_fault_onoff = true,
                         .target_output_voltage = 230,
                         .ondelay_s = 3,
                         .ip_low_voltage = 220,
                         .ip_high_voltage = 290,
                         .op_high_voltage = 250,
                         .op_low_voltage = 190,
                         .hi_lo_time_s = 15,
                         .contactor_fail_onoff=0,
                         .input_calibration = 1.0,
                         .output_calibration=1.0,
                         .current_calibration = 1.0,
                         .ovld_reset_onoff = 1.0
                         };
*/
//---------------------------------------------------------------

//----------------------------------------------------------------------------------------------

//---------------------------Function
//Prototype--------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);
//------------------------------------------------------------------------------------------------

//---------------------------------ADC
//Init-------------------------------------------------------
void initADCA0(void) {
  ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_8_0);
  ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
  ADC_enableConverter(ADCA_BASE);
  DEVICE_DELAY_US(1000);

  // Configure 8 SOCs
  ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0,
               15);
  
  // ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY,
  // ADC_CH_ADCIN1, 80); ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2,
  // ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN2, 80); ADC_setupSOC(ADCA_BASE,
  // ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 80);
  // ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
  // ADC_CH_ADCIN4, 80); ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5,
  // ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5, 80); ADC_setupSOC(ADCA_BASE,
  // ADC_SOC_NUMBER6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN6, 80);
  // ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_SW_ONLY,
  // ADC_CH_ADCIN7, 80);
  ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN11,
               80);
               ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN4,
               15);
               ADC_setupSOC(ADCA_BASE,ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,ADC_CH_ADCIN7,
               15);
  ADC_setupSOC(ADCA_BASE,ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,ADC_CH_ADCIN8,15);
  ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN5,
               80);
               
}

//--------------------------------------------------------------------------------------------------

//--------------------------Read ADC
//Value-------------------------------------------------------
uint16_t readADC(uint16_t socNumber) {

  ADC_forceSOC(ADCA_BASE, socNumber);
  while (ADC_isBusy(ADCA_BASE))
    ;
  return ADC_readResult(ADCARESULT_BASE, socNumber);
}
//--------------------------------------------------------------------------------------------------

//-----------------------------ADC
//Oversample----------------------------------------------------------
uint16_t readADCA0_oversampled(uint16_t ADC_NUMBER) {
  uint32_t sum = 0;
  int i;
  for (i = 0; i < OSR; i++) {

    sum += readADC(ADC_NUMBER);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
  }

  return (uint16_t)(sum / OSR);
}

//--------------------------------------------------------------------------------------------------------

//-------------------------------------ZERO CROSS
//Dtector---------------------------------------------------

int waitForZeroCross(uint8_t pin) {
  uint32_t timeout = 0;

  // Wait for HIGH (negative half cycle)
  while (GPIO_readPin(pin) == 0) {
    if (++timeout > ZC_TIMEOUT)
      return 0; // No signal
  }

  timeout = 0;

  // Wait for falling edge
  while (GPIO_readPin(pin) == 1) {
    if (++timeout > ZC_TIMEOUT)
      return 0; // No signal
  }

  return 1; // Zero-cross detected
}

//----------------------------------------------------------------------------------------------------

//----------------------------------------AC Voltage
//OUTPUT------------------------------------------
float ac_voltage_rms(uint16_t ADC_NO) {
  float sum_squares = 0.0f;
  float temp;
  uint16_t adc_val;

  waitForZeroCross(V_ZC);
    

  int i;
  for (i = 0; i < SAMPLE_COUNT; i++) {
    adc_val = readADCA0_oversampled(ADC_NO);

    /* Convert ADC to actual voltage step-by-step using same temp variable */
    temp = ((float)adc_val * ADC_REF_VOLT) / ADC_MAX;
    temp -= ADC_OFFSET;
    temp /= GAIN_FACTOR;

    sum_squares += temp * temp;
  }

  return sqrtf(sum_squares / SAMPLE_COUNT);
}

//------------------------------------------------------------------------------------------------

//---------------------------AC
//Current-----------------------------------------------------------
float ac_current_rms(void) {
  float sum_squares = 0.0;

  // waitForZeroCross(C_ZC);
  int i = 0;
  for (i = 0; i < SAMPLE_COUNT; i++) {
    uint16_t adc_val = readADCA0_oversampled(ADC_SOC_NUMBER1);
    float v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
    float current_signal = v_adc - CURRENT_ADC_OFFSET;
    float actual_current = current_signal / CURRENT_GAIN_FACTOR;

    sum_squares += actual_current * actual_current;
  }

  return sqrt(sum_squares / SAMPLE_COUNT);
}

//---------------------------MOTOR
//Current-----------------------------------------------------------
float ac_current_motor(void) {
  float sum_squares = 0.0;

  // waitForZeroCross(C_ZC);
  int i = 0;
  for (i = 0; i < SAMPLE_COUNT; i++) {
    uint16_t adc_val = readADCA0_oversampled(ADC_SOC_NUMBER5);
    float v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
    float current_signal = v_adc - CURRENT_ADC_OFFSET;
    float actual_current = current_signal / 0.0225;

    sum_squares += actual_current * actual_current;
  }

  return sqrt(sum_squares / SAMPLE_COUNT);
}

//----------------------------------------------------------------------------------------------

//-----------------------------UART
//Function-----------------------------------------------------

void initUART0(void) {
  Device_init();
  Device_initGPIO();

  GPIO_setPinConfig(GPIO_35_SCIA_RX);
  GPIO_setPinConfig(GPIO_37_SCIA_TX);

  SCI_setConfig(
      SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200,
      (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));

  SCI_resetChannels(SCIA_BASE);
  SCI_resetTxFIFO(SCIA_BASE);
  SCI_resetRxFIFO(SCIA_BASE);
  SCI_enableFIFO(SCIA_BASE);
  SCI_enableModule(SCIA_BASE);
}
void uartPrint(const char *msg) {
  while (*msg)
    SCI_writeCharBlockingFIFO(SCIA_BASE, *msg++);
}

void uartPrintf(const char *format, ...) {
  char buffer[128];
  va_list args;

  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  uartPrint(buffer);
}
// uartPrintf("vout=%d\n", v_out);
//--------------------------------------------------------------------------------------------------

//----------------------------------------------Frequncy
//Measurment---------------------------------

float measureFrequency(uint32_t zcPin) {
  uint32_t pulseCount = 0;
  uint32_t elapsed_us = 0;

  uint8_t lastState = GPIO_readPin(zcPin);

  while (elapsed_us < 300000) // 300 ms
  {
    uint8_t currentState = GPIO_readPin(zcPin);

    if (currentState == 1 && lastState == 0) {
      pulseCount++;
    }

    lastState = currentState;

    DEVICE_DELAY_US(100);
    elapsed_us += 100;
  }

  float measurement_time = 0.3; // 300 ms

  float frequency = pulseCount / measurement_time;

  // If 2 pulses per cycle:
  // frequency = frequency / 2.0;

  return frequency;
}

//------------------------------------------------------------------------------------------------

void main(void) {
  initUART0();
  initADCA0();
  Device_initGPIO();
  I2C_GPIO_init();
    I2Cinit();
  lcd_begin(16, 2);

  GPIO_setPadConfig(inc, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(inc, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(dec, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(dec, GPIO_DIR_MODE_OUT);

  GPIO_writePin(inc, 1);
  GPIO_writePin(dec, 1);

  GPIO_setPadConfig(buzzer, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(buzzer, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(relay, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(relay, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(UP, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(UP, GPIO_DIR_MODE_IN);

  GPIO_setPadConfig(DOWN, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(DOWN, GPIO_DIR_MODE_IN);

  GPIO_setPadConfig(SET, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(SET, GPIO_DIR_MODE_IN);

  GPIO_setPinConfig(GPIO_227_GPIO227);
GPIO_setAnalogMode(227, GPIO_ANALOG_DISABLED);
GPIO_setQualificationMode(227, GPIO_QUAL_SYNC);

  GPIO_setPadConfig(V_ZC, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(V_ZC, GPIO_DIR_MODE_IN);
  GPIO_setPadConfig(C_ZC, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(C_ZC, GPIO_DIR_MODE_IN);
  buzer_state = 1;

  // Initializes PIE and clears PIE registers. Disables CPU interrupts.
  Interrupt_initModule();
  Device_initGPIO();

  // Disable all CPU interrupts and clear all CPU interrupt flags.
  DINT;
  IER = 0x0000;
  IFR = 0x0000;

  Interrupt_initVectorTable(); // Initializes the PIE vector table with pointers
                               // to the shell Interrupt

    int res = load_parameters(&store_data, EEPROM_START_ADDR);
    //uartPrintf("Load result = %d\n", res);


  Board_init();
  dataMutex = xSemaphoreCreateMutex();
  parameter_mutex = xSemaphoreCreateMutex();
  FreeRTOS_init(); // Configure FreeRTOS
}

int calculate_num(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  return (a * 1000) + (b * 100) + (c * 10) + d;
}

void set_number(int raw) {
  d1 = (raw / 1000) % 10;
  d2 = (raw / 100) % 10;
  d3 = (raw / 10) % 10;
  d4 = raw % 10;
}



//----------------------------------------ControllTask----------------------------------------------------------

void Control_Task(void *pvParameters) {
  uint8_t output_regulation = 3,flag=0,ip_c=100,op_c=100,oa_c=100;
  bool mode_auto_onoff = 1;
  int v_input=0,contactor_voltage=0,v_earth=0,v_out=0,motor_cur=0;
  float cur;
  for (;;) {
    v_out = (int)(ac_voltage_rms(ADC_SOC_NUMBER0));
    v_out = v_out * op_c /100;
    //uartPrintf("op = %d\n",ip_c);
    
      
       cur = ac_current_rms();
       cur = cur * oa_c / 100;
    // int fre = (int)(measureFrequency(V_ZC));
    if (v_out < 100 || v_out > 1000)
      v_out = 0;

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    gData.voltage_out = v_out;
    gData.current = cur;
    gData.voltage_in = v_input;
    gData.Frequncy = 50;
    gData.contactor_v = contactor_voltage;
    gData.motor_current = motor_cur;
    gData.earth_voltage = v_earth;
    xSemaphoreGive(dataMutex);
//v_out = (int)((0.9857f * (v_out)) - 18.67f);
    xSemaphoreTake(parameter_mutex, portMAX_DELAY);

    output_regulation = required_parameter.reg;
    mode_auto_onoff = required_parameter.auto_onoff;
    target = required_parameter.set_volt;
    ip_c = required_parameter.ip_cal;
    op_c = required_parameter.op_cal;
    oa_c = required_parameter.oa_cal;

    xSemaphoreGive(parameter_mutex);

    //uartPrintf("v_earth= %d\n",output_regulation);  // 78 v earth   0.2888
    //uartPrintf("cur = %d\n",motor_cur);
    // uartPrintf("cur = %d\n",cur);
    //uartPrintf("reg = %d\n",output_regulation);
    error = v_out - target;
    // uartPrintf("fre = %d\n",(int)fre);

    if (error >= 3 && mode_auto_onoff) {
      GPIO_writePin(inc, 1);
      GPIO_writePin(dec, 0);
    } else if (error <= -3 && mode_auto_onoff) {
      GPIO_writePin(dec, 1);
      GPIO_writePin(inc, 0);
    } else if (mode_auto_onoff) {
      GPIO_writePin(inc, 1);
      GPIO_writePin(dec, 1);
      // measure

      if(flag == 0) {
       v_input = (int)ac_voltage_rms(ADC_SOC_NUMBER2);
       v_input = v_input * ip_c/100;
      }

      else if(flag==1)
       contactor_voltage = (int)ac_voltage_rms(ADC_SOC_NUMBER3);

      else if(flag == 2)
       v_earth = ac_voltage_rms(ADC_SOC_NUMBER4);

      else if(flag==3) motor_cur = (int)ac_current_motor();

      else{ flag =0;}

      ++flag;
    }
    else {
      if (GPIO_readPin(UP) == 0 && display_state == NORMAL) {
        GPIO_writePin(dec, 1);
        GPIO_writePin(inc, 0);
      } else if (GPIO_readPin(DOWN) == 0 && display_state == NORMAL) {
        GPIO_writePin(inc, 1);
        GPIO_writePin(dec, 0);
      } else {
        GPIO_writePin(inc, 1);
        GPIO_writePin(dec, 1);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // control every 20ms
  }
}
const char *getonoff(uint8_t value) {
  if (value == 1)
    return "ON";
  else
    return "OFF";
}

void buzer_off() {

  buzer_state = 0;
  GPIO_writePin(buzzer, 0);
}
//------------------------------------------------------------------------------------------------------

//--------------store function-----------------------------------------------

void edit_data_store(int new_interger_data,float new_floater_data) {

switch(pre_state) {

 case REGULATION:
 store_data.regulation_voltage = new_interger_data;
 break;

 case MANUAL_MODE:
 store_data.Mode = new_interger_data;
 break;

 case OP_CURRENT:
 store_data.op_current = new_interger_data;
 break;

 case OVERLOAD:
 store_data.overload_current = new_interger_data;
 break;

 case OVLD_TIME:
 store_data.ovld_time_s = new_interger_data;
 break;

 case OVLD_IMD:
 store_data.ovld_imd_current = new_interger_data;
 break;

 case EARTH_FAIL:
 store_data.earth_fail_onoff = new_interger_data;
 break;

 case EARTH_HIGH:
 store_data.earth_high_voltage = new_interger_data;
 break;

 case EPOINT:
 store_data.epoint_fail = new_interger_data;
 break;

  case MOTOR_FAULT:
  store_data.motor_fault_onoff = new_interger_data;
  break;


  case SET_VOLTAGE:
  if(new_interger_data > 250) {
    store_data.target_output_voltage = 250;
    store_data.upper_target_voltage = (new_interger_data - 250);
  }
  else {
    store_data.target_output_voltage = new_interger_data;
    store_data.upper_target_voltage = 0;
  }
  break;

  case ON_DELAY:
  store_data.ondelay_s = new_interger_data;
  break;

  case IP_LOW:
  if(new_interger_data > 250) {
    store_data.ip_low_voltage = 250;
    store_data.upper_ip_low_voltage = (new_interger_data - 250);
  }
  else {
    store_data.ip_low_voltage = 250;
    store_data.upper_ip_low_voltage = 0;
  }
  
  break;

  case IP_HIGH:
   if(new_interger_data > 250) {
    store_data.ip_high_voltage = 250;
    store_data.upper_ip_high_voltage = (new_interger_data - 250);
  }
  else {
    store_data.ip_high_voltage = 250;
    store_data.upper_ip_high_voltage = 0;
  }
  break;

  case OP_LOW:
   if(new_interger_data > 250) {
    store_data.op_low_voltage = 250;
    store_data.upper_op_low_voltage = (new_interger_data - 250);
  }
  else {
    store_data.op_low_voltage = 250;
    store_data.upper_op_low_voltage = 0;
  }
  break;

  case OP_HIGH:
  if(new_interger_data > 250) {
    store_data.op_high_voltage = 250;
    store_data.upper_op_high_voltage = (new_interger_data - 250);
  }
  else {
    store_data.op_high_voltage = 250;
    store_data.upper_op_high_voltage = 0;
  }
  break;

  case HI_LO_TIME:
  store_data.hi_lo_time_s = new_interger_data;
  break;

  case CONTACTOR_FAIL:
  store_data.contactor_fail_onoff = new_interger_data;
  break;

  case OVLD_RESET:
  store_data.ovld_reset_onoff = new_interger_data;
  break;

  case IP_CALIBRATION:
  store_data.input_calibration = (uint8_t)(((float)new_interger_data/v_in) * 100);
  break;

   case OP_CALIBRATION:
   store_data.output_calibration = (uint8_t)(((float)new_interger_data/v_o) * 100);
   break;

   case OA_CALIBRATION:
  store_data.current_calibration = (uint8_t)(((float)new_interger_data/c) * 100);
  break;

  default:
  return;
}


}

//---------------------------------------------------------------------------
//---------------------------------------------------UITask---------------------------------------------

void UI_Task(void *pvParameters) {
  

//--gama------
  int earth_vol,t_start=5,f,parameter_pre_temp = 4,hlt_timer=0,contactor_vol=0,motor_c=0;
  uint8_t temp = 0, edit_count = 0;
  bool edit_screen = 0, blink_flag = 0;

  for (;;) {

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    v_in = gData.voltage_in;
    c = gData.current;
    v_o = gData.voltage_out;
    f = gData.Frequncy;
    contactor_vol = gData.contactor_v;
    motor_c = gData.motor_current;
    earth_vol = gData.earth_voltage;
    xSemaphoreGive(dataMutex);

    xSemaphoreTake(parameter_mutex, portMAX_DELAY);
    required_parameter.reg = store_data.regulation_voltage;
    required_parameter.auto_onoff = store_data.Mode;
    required_parameter.set_volt = store_data.target_output_voltage + store_data.upper_target_voltage;
    required_parameter.ip_cal = store_data.input_calibration;
    required_parameter.op_cal = store_data.output_calibration;
    required_parameter.oa_cal = store_data.current_calibration;
    xSemaphoreGive(parameter_mutex);

   if(c<0.95) {c=0;}

   if(GPIO_readPin(227) == 0) {
    edit_screen = 0;
    display_state = NORMAL;
   }
   
    if (edit_screen == 0) {
      lcd_clear();
      lcd_setCursor(0, 0);
    } else {
      lcd_setCursor(0, 1);
      lcd_print("                ");
    }

    switch (display_state) {

    case NORMAL:
      lcd_printf("IP:%3d OP:%3d", v_in, v_o);
      lcd_setCursor(0, 1);
      lcd_printf("OA:%d.%d F:%d", (int)c,(int)((c-(int)c)*10), f);
      if (GPIO_readPin(SET) == 0) {
        display_state = TRIP_LOGS;
      }
      break;

    case TRIP_LOGS:
      lcd_printf("TRIP LOG:");
      lcd_setCursor(0, 1);
      lcd_printf("<--   M:01   -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = MANUAL_MODE;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = CALIBRATION;
      }

      break;

    case MANUAL_MODE:
      lcd_printf("AUTO MODE: ");
      lcd_printf(getonoff(store_data.Mode));
      lcd_setCursor(0, 1);
      lcd_printf("<--   M:02   -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = SETTING;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = TRIP_LOGS;
      }
      if (GPIO_readPin(SET) == 0) {
        pre_state = display_state;
        onoff_temp = store_data.Mode;
        display_state = EDIT_ONOFF;
        edit_screen = 1;
      }
      break;

    case SETTING:
      lcd_printf("SETTING :");
      lcd_setCursor(0, 1);
      lcd_printf("<--   M:03   -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = FACT_SETTING;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = MANUAL_MODE;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        display_state = SETTING_PASSWORD;
      }
      break;

    case FACT_SETTING:
      lcd_printf("FACT SETTING:");
      lcd_setCursor(0, 1);
      lcd_printf("<--   M:04   -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = CALIBRATION;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = SETTING;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        display_state = FACT_PASSWORD;
      }
      break;

    case CALIBRATION:
      lcd_printf("CALIBRATION:");
      lcd_setCursor(0, 1);
      lcd_printf("<--   M:05   -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = TRIP_LOGS;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = FACT_SETTING;
      }
       if (GPIO_readPin(SET) == 0) {
       edit_screen = 1;
        display_state = CALIBRATION_PASSWORD;
      }
      break;

    case REGULATION:
      lcd_printf("REGULATION: ");
      lcd_printf("%4d", store_data.regulation_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P01 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OP_CURRENT;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = MOTOR_FAULT;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.regulation_voltage);
        display_state = EDIT;
      }
      break;

    case OP_CURRENT:
      lcd_printf("OP CURRENT: ");
      lcd_printf(getonoff(1));
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P02 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OVERLOAD;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = REGULATION;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        onoff_temp = store_data.op_current;
        display_state = EDIT_ONOFF;
      }
      break;

    case OVERLOAD:
      lcd_printf("OVERLOAD:");
       lcd_printf("%4d", store_data.overload_current);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P03 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OVLD_TIME;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OP_CURRENT;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.overload_current);
        display_state = EDIT;
      }
      break;

    case OVLD_TIME:
      lcd_printf("OVLD TIME:");
       lcd_printf("%4d",store_data.ovld_time_s);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P04 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OVLD_IMD;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OVERLOAD;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.ovld_time_s);
        display_state = EDIT;
      }
      break;

    case OVLD_IMD:
      lcd_printf("OVLD IMD:");
       lcd_printf("%4d", store_data.ovld_imd_current);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P05 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OVLD_RESET;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OVLD_TIME;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.ovld_imd_current);
        display_state = EDIT;
      }
      break;

      case OVLD_RESET:
      lcd_printf("OVLD RESET AUTO: ");
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P06 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = CONTACTOR_FAIL;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OVLD_IMD;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        onoff_temp = store_data.ovld_reset_onoff;
        display_state = EDIT_ONOFF;
      }
      break;


      case CONTACTOR_FAIL:
      lcd_printf("CONTACTOR FAIL: ");
      lcd_printf(getonoff(store_data.contactor_fail_onoff));
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P07 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = EARTH_FAIL;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OVLD_RESET;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        onoff_temp = store_data.contactor_fail_onoff;
        display_state = EDIT_ONOFF;
      }
      break;

    case EARTH_FAIL:
      lcd_printf("EARTH FAIL: ");
      lcd_printf(getonoff(store_data.earth_fail_onoff));
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P08 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = EARTH_HIGH;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = CONTACTOR_FAIL;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        onoff_temp = store_data.earth_fail_onoff;
        display_state = EDIT_ONOFF;
      }
      break;

    case EARTH_HIGH:
      lcd_printf("EARTH HIGH: ");
      lcd_printf("%4d", store_data.earth_high_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P09 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = EPOINT;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = EARTH_FAIL;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.earth_high_voltage);
        display_state = EDIT;
      }
      break;

    case EPOINT:
      lcd_printf("EPOINT FAIL:");
      lcd_printf(getonoff(store_data.epoint_fail));
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P10 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = MOTOR_FAULT;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state =EARTH_HIGH;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        onoff_temp = store_data.epoint_fail;
        display_state = EDIT_ONOFF;
      }
      break;

    case MOTOR_FAULT:
      lcd_printf("MOTOR FAULT: ");
      lcd_printf(getonoff(store_data.motor_fault_onoff));
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P11 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = REGULATION;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = EPOINT;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        onoff_temp = store_data.motor_fault_onoff;
        display_state = EDIT_ONOFF;
      }
      break;

    case FACT_PASSWORD:
      lcd_printf(" ENTER PASSWORD");
      edit_screen = 1;
      pre_state = display_state;
      set_number(0);
      display_state = EDIT;
      break;

    case SETTING_PASSWORD:
      lcd_printf(" ENTER PASSWORD");
      edit_screen = 1;
      pre_state = display_state;
      set_number(0);
      display_state = EDIT;
      break;

      case CALIBRATION_PASSWORD:
      lcd_printf(" ENTER PASSWORD");
      edit_screen = 1;
      pre_state = display_state;
      set_number(0);
      display_state = EDIT;
      break;

    case SET_VOLTAGE:
      lcd_printf("SET VOLTAGE: ");
      lcd_printf("%3d", store_data.target_output_voltage + store_data.upper_target_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P01 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = ON_DELAY;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = HI_LO_TIME;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.target_output_voltage + store_data.upper_target_voltage);
        display_state = EDIT;
      }
      break;

    case ON_DELAY:
      lcd_printf("ON DELAY: ");
      lcd_printf("%4d", store_data.ondelay_s);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P02 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = IP_LOW;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = SET_VOLTAGE;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.ondelay_s);
        display_state = EDIT;
      }
      break;

    case IP_LOW:
      lcd_printf("IP LOW: ");
      lcd_printf("%4d",store_data.ip_low_voltage + store_data.upper_ip_low_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P03 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = IP_HIGH;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = ON_DELAY;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.ip_low_voltage + store_data.upper_ip_low_voltage);
        display_state = EDIT;
      }
      break;

    case IP_HIGH:
      lcd_printf("IP HIGH: ");
      lcd_printf("%4d",store_data.ip_high_voltage + store_data.upper_ip_high_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P04 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OP_LOW;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = IP_LOW;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.ip_high_voltage + store_data.upper_ip_high_voltage);
        display_state = EDIT;
      }
      break;

    case OP_LOW:
      lcd_printf("OP LOW: ");
      lcd_printf("%4d",store_data.op_low_voltage + store_data.upper_op_low_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P05 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OP_HIGH;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = IP_HIGH;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.op_low_voltage + store_data.upper_op_low_voltage);
        display_state = EDIT;
      }
      break;

    case OP_HIGH:
      lcd_printf("OP HIGH: ");
      lcd_printf("%4d",store_data.op_high_voltage + store_data.upper_op_high_voltage);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P06 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = HI_LO_TIME;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OP_LOW;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.op_high_voltage + store_data.upper_op_high_voltage);
        display_state = EDIT;
      }
      break;

    case HI_LO_TIME:
      lcd_printf("HI-LO TIME: ");
      lcd_printf("%3d", store_data.hi_lo_time_s);
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P07 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = SET_VOLTAGE;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OP_HIGH;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(store_data.hi_lo_time_s);
        display_state = EDIT;
      }
      break;

      case IP_CALIBRATION:
      lcd_printf("IP CALIBRATION: ");
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P01 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OP_CALIBRATION;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OA_CALIBRATION;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(v_in);
        display_state = EDIT;
      }
      break;

      case OP_CALIBRATION:
      lcd_printf("OP CALIBRATION: ");
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P02 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = OA_CALIBRATION;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = IP_CALIBRATION;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number(v_o);
        display_state = EDIT;
      }
      break;

      case OA_CALIBRATION:
      lcd_printf("OA CALIBRATION: ");
      lcd_setCursor(0, 1);
      lcd_printf("<-- EDIT:P03 -->");
      if (GPIO_readPin(UP) == 0) {
        display_state = IP_CALIBRATION;
      }
      if (GPIO_readPin(DOWN) == 0) {
        display_state = OP_CALIBRATION;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 1;
        pre_state = display_state;
        set_number((int)c);
        display_state = EDIT;
      }
      break;

    case EDIT:
      lcd_setCursor(0, 1);
      lcd_printf("EDIT:       %d%d%d%d", d1, d2, d3, d4);

      if (blink_flag == 0) {
        lcd_setCursor(12 + edit_count, 1);
        lcd_printf("*");
        blink_flag = 1;
      } else {
        blink_flag = 0;
      }
      if (GPIO_readPin(SET) == 0) {
        ++edit_count;
        if (edit_count > 3) {
          edit_screen = 0;
          edit_count = 0;

          if (pre_state == FACT_PASSWORD || pre_state == SETTING_PASSWORD || pre_state == CALIBRATION_PASSWORD) {

            if (pre_state == FACT_PASSWORD &&
                calculate_num(d1, d2, d3, d4) == 101) {
              display_state = REGULATION;
            } else if (pre_state == SETTING_PASSWORD &&
                       calculate_num(d1, d2, d3, d4) == 1) {
              display_state = SET_VOLTAGE;
            }
            else if (pre_state == CALIBRATION_PASSWORD &&
                       calculate_num(d1, d2, d3, d4) == 1) {
              display_state = IP_CALIBRATION;
            }
             else {
              lcd_clear();
              lcd_setCursor(0,0);
              lcd_printf(" WRONG PASSWORD ");
              vTaskDelay(pdMS_TO_TICKS(100));
              display_state = NORMAL;
            }
          }

          else {
            edit_data_store(calculate_num(d1, d2, d3, d4),0);
             save_parameters(&store_data, EEPROM_START_ADDR);
            display_state = pre_state;
          }
        }
      }
      if (GPIO_readPin(UP) == 0) {
        switch (edit_count) {
        case 0:
          ++d1;
          if (d1 > 9)
            d1 = 0;
          break;
        case 1:
          ++d2;
          if (d2 > 9)
            d2 = 0;
          break;
        case 2:
          ++d3;
          if (d3 > 9)
            d3 = 0;
          break;
        case 3:
          ++d4;
          if (d4 > 9)
            d4 = 0;
          break;
        }
      }
      if (GPIO_readPin(DOWN) == 0) {
        switch (edit_count) {
        case 0:
          --d1;
          if (d1 < 0)
            d1 = 9;
          break;
        case 1:
          --d2;
          if (d2 < 0)
            d2 = 9;
          break;
        case 2:
          --d3;
          if (d3 < 0)
            d3 = 9;
          break;
        case 3:
          --d4;
          if (d4 < 0)
            d4 = 9;
          break;
        }
      }
      break;

    case EDIT_ONOFF:
      lcd_setCursor(0, 1);
      lcd_printf("EDIT:       ");
      lcd_printf(getonoff(onoff_temp));
      if (GPIO_readPin(DOWN) == 0 || GPIO_readPin(UP) == 0) {
        onoff_temp = !onoff_temp;
      }
      if (GPIO_readPin(SET) == 0) {
        edit_screen = 0;
        edit_data_store(onoff_temp, 0);
         save_parameters(&store_data, EEPROM_START_ADDR);
        display_state = pre_state;
      }
      break;
    }
    if (GPIO_readPin(UP) == 0) {
      ++temp;
    } else if (GPIO_readPin(DOWN) == 0) {
      --temp;
    }

    //------------------LOOP---------------------------------------------


    //-------------------alpha---------------------------------

if(v_in > (store_data.ip_high_voltage + store_data.upper_ip_high_voltage) && v_in_high == 0) {
  display_error = 1;
++hlt_timer;
if(hlt_timer > (store_data.hi_lo_time_s * 5)) {
relay_flag = 0;
buzer_off();
v_in_high = 1;
}
else {
  buzer_state = 1;
}
}
else if(v_in < (store_data.ip_low_voltage + store_data.upper_ip_low_voltage)&& v_in_low == 0) {
  //uartPrintf("input Low %d\n",hlt_timer);
display_error = 2;
++hlt_timer;
if(hlt_timer > (store_data.hi_lo_time_s * 5)) {
   //uartPrintf("Trip\n");
relay_flag = 0;
buzer_off();
v_in_low = 1;
}
else {
  buzer_state = 1;
}
}
else if(v_o > (store_data.op_high_voltage + store_data.upper_op_high_voltage)&& op_high == 0){
  display_error = 3;
++hlt_timer;
if(hlt_timer > (store_data.hi_lo_time_s * 5)) {
relay_flag = 0;
buzer_off();
op_high = 1;
}
else {
  buzer_state = 1;
}
}
else if(v_o < (store_data.op_low_voltage + store_data.upper_op_low_voltage) && op_low == 0) {
  display_error = 4;
++hlt_timer;
if(hlt_timer > (store_data.hi_lo_time_s * 5)) {
relay_flag = 0;
buzer_off();
op_low = 1;
}
else {
  buzer_state = 1;
}
}

else if(relay_flag == 1 && contactor_vol < 120 && v_in > 150 & store_data.contactor_fail_onoff) {
  display_error = 6;
++hlt_timer;
if(hlt_timer > (5 * 5)) {
relay_flag = 0;
buzer_off();
}
else {
  buzer_state = 1;
}
}
else if(motor_c > 17 && store_data.motor_fault_onoff) {
display_error = 7;
++hlt_timer;
if(hlt_timer > (5 * 5)) {
relay_flag = 0;
buzer_off();
}
else {
  buzer_state = 1;
}
}
else if((earth_vol - 35) > store_data.earth_high_voltage && store_data.earth_fail_onoff) {
display_error = 8;
++hlt_timer;
if(hlt_timer > (5 * 5)) {
relay_flag = 0;
buzer_off();
}
else {
  buzer_state = 1;
}
}

else if(c > store_data.overload_current) {
  display_error = 5;
++hlt_timer;
if(hlt_timer > (store_data.ovld_time_s * 5)) {
relay_flag = 0;
buzer_off();
int timer_count = 0;
GPIO_writePin(relay,relay_flag);
lcd_clear();
lcd_setCursor(0,0);
lcd_print(" WAIT FOR 5 MIN ");
while(timer_count < 1500)   // 1500 × 200ms = 300,000ms = 5 minutes
{
    vTaskDelay(pdMS_TO_TICKS(200));
    timer_count++;
}
time = 0;
relay_flag = 1;
start_flag = 0;
}
else {
  buzer_state = 1;
}

}

else if(c > store_data.ovld_imd_current) {
  
relay_flag = 0;
int timer_count = 0;
GPIO_writePin(relay,relay_flag);
lcd_clear();
lcd_setCursor(0,0);
lcd_print(" WAIT FOR 5 MIN ");
while(timer_count < 1500)   // 1500 × 200ms = 300,000ms = 5 minutes
{
    vTaskDelay(pdMS_TO_TICKS(200));
    timer_count++;
}
time = 0;
relay_flag = 1;
start_flag = 0;
  }
  
else {

  hlt_timer = 0;
  display_error = 0;
  if(start_flag == 1) {buzer_off();}
  
}

if(v_in_high || v_in_low || op_low || op_high) {

if(v_in_high && v_in < ((store_data.ip_high_voltage + store_data.upper_ip_high_voltage) - 15)) {
relay_flag = 1;
v_in_high = 0;
}
if(v_in_low && v_in > ((store_data.ip_low_voltage + store_data.upper_ip_low_voltage )+ 15)) {
relay_flag = 1;
v_in_low = 0;
}
if(op_high && v_o < ((store_data.op_high_voltage+ store_data.upper_op_high_voltage) - 15)) {
relay_flag = 1;
op_high = 0;
}
if(op_low && v_o > ((store_data.op_low_voltage + store_data.upper_op_low_voltage)+ 15)) {
relay_flag = 1;
op_low = 0;
}

}
//--------------------------------------------------------


    if (buzer_count < 6 && buzer_state) {
      GPIO_writePin(buzzer, 1);

      if(display_error > 0) {
         //uartPrintf("inside errro display\n");
        lcd_clear();
        lcd_setCursor(0,0);

        switch(display_error) {

          case 1:
          lcd_printf("   INPUT HIGH   ");
          break;
          case 2:
          lcd_printf("   INPUT LOW   ");
          break;
          case 3:
          lcd_printf("   OUTPUT HIGH  ");
          break;
          case 4:
          lcd_printf("   OUTPUT LOW  ");
          break;
          case 5:
          lcd_printf("   OVERLOAD-1   ");
          break;
          case 6:
          lcd_printf(" CONTACTOR FAIL ");
          break;
          case 7:
          lcd_printf("  MOTOR FAULT   ");
          break;
          case 8:
          lcd_printf("   EARTH HIGH   ");
          break;
        }   
      }
    } else {
      GPIO_writePin(buzzer, 0);
    }
    if (buzer_count > 12) {
      buzer_count = 0;
    }
    ++buzer_count;

    if (time > 25 && start_flag == 0) {
      buzer_off();
      start_flag = 1;
      relay_flag = 1;
    }

   if(start_flag == 0) relay_flag=0;
    GPIO_writePin(relay, relay_flag);
  
    ++time;

    

    //--------------------------------------------------------------------
    vTaskDelay(pdMS_TO_TICKS(140)); // UI slower
  }
}

//--------------------------------------------------------------------------------------------------------

//----------------------------Stakeoverflow
//Hook--------------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
  (void)pcTaskName;
  (void)pxTask;

  uartPrintf("Stackoverflow\n");
  /* Run time stack overflow checking is performed if
   * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook function is
   * called if a stack overflow is detected. */
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}

//---------------------------------------------------------------------------------------------------------

//----------------------------------Malloc
//Hook------------------------------------------------------------

void vApplicationMallocFailedHook(void) {
  /* vApplicationMallocFailedHook() will only be called if
  configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
  function that will get called if a call to pvPortMalloc() fails.
  pvPortMalloc() is called internally by the kernel whenever a task, queue,
  timer or semaphore is created.  It is also called by various parts of the
  demo application.  If heap_1.c or heap_2.c are used, then the size of the
  heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
  FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
  to query the size of free heap space that remains (although it does not
  provide information on how the remaining heap might be fragmented). */
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}
