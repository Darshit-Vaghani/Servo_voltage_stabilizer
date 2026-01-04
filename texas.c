#include "board.h"
#include "device.h"
#include "driverlib.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

/* ---------------- CONFIG ---------------- */

#define ADC_MAX        4095.0
#define ADC_REF_VOLT   3.3
#define ADC_OFFSET     1.645
#define GAIN_FACTOR    0.000202020

#define SAMPLE_COUNT   1500
#define OSR            16

#define inc     16
#define dec     13
#define ZC_PIN  32     // Zero-cross input (HIGH in negative cycle)

int target = 230;
int error = 0;

/* ---------------- UART ---------------- */

void initUART0(void)
{
    Device_init();
    Device_initGPIO();

    GPIO_setPinConfig(GPIO_35_SCIA_RX);
    GPIO_setPinConfig(GPIO_37_SCIA_TX);

    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200,
        (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));

    SCI_resetChannels(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);
}

void uartPrint(const char *msg)
{
    while (*msg)
        SCI_writeCharBlockingFIFO(SCIA_BASE, *msg++);
}

void uartPrintf(const char *format, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    uartPrint(buffer);
}

/* ---------------- ADC ---------------- */

void initADCA0(void)
{
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_8_0);
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);

    ADC_setupSOC(ADCA_BASE,
                 ADC_SOC_NUMBER0,
                 ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN0,
                 15);

    ADC_setInterruptSource(ADCA_BASE,
                           ADC_INT_NUMBER1,
                           ADC_SOC_NUMBER0);

    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

/* -------- Oversampled ADC Read -------- */

uint16_t readADCA0_oversampled(void)
{
    uint32_t sum = 0;
    int i;
    for(i = 0; i < OSR; i++)
    {
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);
        while(!ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1));

        sum += ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    return (uint16_t)(sum / OSR);
}

/* -------- Zero Cross Wait -------- */

void waitForZeroCross(void)
{
    // Wait until negative cycle
    while(GPIO_readPin(ZC_PIN) == 0);

    // Wait for zero-cross (falling edge)
    while(GPIO_readPin(ZC_PIN) == 1);
}

/* ---------------- RMS ---------------- */

float ac_voltage_rms(void)
{
    uint16_t adc_val;
    float sum_squares = 0.0;
    float v_adc, ac_signal, actual_voltage;

    // 🔥 Start exactly at zero crossing
    waitForZeroCross();
    int i;
    for(i = 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = readADCA0_oversampled();

        v_adc = (adc_val * ADC_REF_VOLT) / ADC_MAX;
        ac_signal = v_adc - ADC_OFFSET;
        actual_voltage = ac_signal / GAIN_FACTOR;

        sum_squares += actual_voltage * actual_voltage;
    }

    return sqrt(sum_squares / SAMPLE_COUNT);
}

/* ---------------- MAIN ---------------- */

void main(void)
{
    initUART0();
    initADCA0();
    Device_initGPIO();

    GPIO_setPadConfig(inc, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(inc, GPIO_DIR_MODE_OUT);

    GPIO_setPadConfig(dec, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(dec, GPIO_DIR_MODE_OUT);

    GPIO_setPadConfig(ZC_PIN, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(ZC_PIN, GPIO_DIR_MODE_IN);

    while(1)
    {
        int v_out = (int)ac_voltage_rms();

        if(v_out < 100 || v_out > 1000)
            v_out = 0;

        error = v_out - target;
        uartPrintf("vout=%d\n",v_out);
        if(error >= 6)
        {
            GPIO_writePin(inc, 0);
            GPIO_writePin(dec, 1);
        }
        else if(error <= -6)
        {
            GPIO_writePin(dec, 0);
            GPIO_writePin(inc, 1);
        }
        else
        {
            GPIO_writePin(inc, 0);
            GPIO_writePin(dec, 0);
        }
    }
}
