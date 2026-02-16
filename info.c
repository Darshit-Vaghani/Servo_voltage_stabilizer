//-----------------------------Header File-----------------------------------

#include "LiquidCrystal_F28E12x.h"
#include "board.h"
#include "device.h"
#include "driverlib.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

//-----------------------------------------------------------------------------

//----------------------Defines-------------------------------------------------

#define ADC_MAX 4095.0
#define ADC_REF_VOLT 3.33
#define ADC_OFFSET 2.08
#define GAIN_FACTOR 0.002222

#define CURRENT_ADC_OFFSET 2.08
#define CURRENT_GAIN_FACTOR 0.01508 // 2388:1 CT + 36Ω

#define SAMPLE_COUNT 369 // 738 2 cycle
#define OSR 7
#define inc 16
#define dec 13
#define V_ZC 6 // Zero-cross input (HIGH in negative cycle)
#define C_ZC 7

#define ZC_TIMEOUT 5000 // adjust based on CPU speed
//-------------------------------------------------------------------------------

//------------------------------UART
//Functions-----------------------------------

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

//-----------------------------------------------------------------------------

//-------------------------ADC
//Configuration---------------------------------------

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
}

//-----------------------------------------------------------------------------------

//---------------------------Read
//ADC-------------------------------------------------

uint16_t readADC(uint16_t socNumber) {
  ADC_forceSOC(ADCA_BASE, socNumber);

  while (ADC_isBusy(ADCA_BASE))
    ;

  return ADC_readResult(ADCARESULT_BASE, socNumber);
}

//-------------------------------------------------------------------------------------

//-------------------ADC
//Oversample-----------------------------------------------------

uint16_t readADCA0_oversampled(uint16_t ADC_NUMBER) {
  uint32_t sum = 0;
  int i;
  for (i = 0; i < OSR; i++) {

    sum += readADC(ADC_NUMBER);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
  }

  return (uint16_t)(sum / OSR);
}

//---------------------------------------------------------------------------------------

//--------------------------Zero Crossing
//detector---------------------------------------

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

//------------------------------------------------------------------------------

//-------------------OUTPUT Current
//Measurment-----------------------------------

float ac_current_rms(void) {
  float sum_squares = 0.0;

  waitForZeroCross(C_ZC);
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

//------------------------------------------------------------------------

//-----------------OUTPUT Voltage Measurment---------------------------------

float ac_voltage_rms(void) {
  uint16_t adc_val;
  float sum_squares = 0.0;
  float v_adc, ac_signal, actual_voltage;

  waitForZeroCross(V_ZC);
  GPIO_writePin(inc, 1);
  int i;
  for (i = 0; i < SAMPLE_COUNT; i++) {
    adc_val = readADCA0_oversampled(ADC_SOC_NUMBER0);

    v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
    ac_signal = v_adc - ADC_OFFSET;
    actual_voltage = ac_signal / GAIN_FACTOR;

    sum_squares += actual_voltage * actual_voltage;
  }
  GPIO_writePin(inc, 0);

  return sqrt(sum_squares / SAMPLE_COUNT);
}

//-----------------------------------------------------------------------------

void main(void) {
  initUART0();
  initADCA0();
  Device_initGPIO();
  lcd_begin(16, 2);

  GPIO_setPadConfig(inc, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(inc, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(dec, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(dec, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(V_ZC, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(V_ZC, GPIO_DIR_MODE_IN);
  GPIO_setPadConfig(C_ZC, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(C_ZC, GPIO_DIR_MODE_IN);

  while (1) {
    // int v_out = (int)(ac_voltage_rms());

    int v_out = (int)((0.9857f * ((int)(ac_voltage_rms()))) - 18.67f);
    if (v_out < 100 || v_out > 1000)
      v_out = 0;

    uartPrintf("%d V\n", v_out);
    uartPrintf("%d mA\n", (int)(ac_current_rms() * 1000));
    DEVICE_DELAY_US(1000000);
  }
}
