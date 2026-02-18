#include "driverlib.h"
#include "device.h"
#include "FreeRTOS.h"
#include "board.h"
#include "c2000_freertos.h"
#include <stdarg.h>
#include "LiquidCrystal_F28E12x.h"
#include <math.h>
#include <stdio.h>

//----------------------Defines-------------------------------------------------

#define ADC_MAX 4095.0
#define ADC_REF_VOLT 3.33
#define ADC_OFFSET 2.08
#define GAIN_FACTOR 0.002222

#define CURRENT_ADC_OFFSET 1.68
#define CURRENT_GAIN_FACTOR 0.01508 // 2388:1 CT + 36Ω

#define SAMPLE_COUNT 369 // 738 2 cycle
#define OSR 7
#define inc 16
#define dec 13
#define V_ZC 6 // Zero-cross input (HIGH in negative cycle)
#define C_ZC 7
#define ZC_TIMEOUT 5000 // adjust based on CPU speed

#define UP 12
#define DOWN 24
#define HOME 242
#define SET 13

int target = 230;
int error = 0;

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
    EARTH_FAIL,
    EARTH_HIGH,
    EPOINT,
    MOTOR_FAULT,
    FACT_PASSWORD,
    EDIT

} UI_state;


//----------------------------------------------------------------------------------------------------

//------------------MUTEX---------------------------

typedef struct
{
    int voltage_out;
    float current;
    int Frequncy;
    int voltage_in;
   
} StabilizerData_t;

StabilizerData_t gData;
SemaphoreHandle_t dataMutex;

//------------------------------------------------

//----------------------------------------------------------------------------------------------

//---------------------------Function Prototype--------------------------------------------------
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationMallocFailedHook( void );
//------------------------------------------------------------------------------------------------

//---------------------------------ADC Init-------------------------------------------------------
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

//--------------------------------------------------------------------------------------------------

//--------------------------Read ADC Value-------------------------------------------------------
uint16_t readADC(uint16_t socNumber) {

  ADC_forceSOC(ADCA_BASE, socNumber); 
  while (ADC_isBusy(ADCA_BASE));
  return ADC_readResult(ADCARESULT_BASE, socNumber);
  
}
//--------------------------------------------------------------------------------------------------

//-----------------------------ADC Oversample----------------------------------------------------------
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

//-------------------------------------ZERO CROSS Dtector---------------------------------------------------

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

//----------------------------------------AC Voltage OUTPUT------------------------------------------
float ac_voltage_rms(void)
{
    float sum_squares = 0.0f;
    float temp;
    uint16_t adc_val;

    if(!waitForZeroCross(V_ZC))
       return 0.0f;   // Safety: no signal

    GPIO_writePin(inc, 1);
   int i;
    for(i= 0; i < SAMPLE_COUNT; i++)
    {
        adc_val = readADCA0_oversampled(ADC_SOC_NUMBER0);

        /* Convert ADC to actual voltage step-by-step using same temp variable */
        temp  = ((float)adc_val * ADC_REF_VOLT) / ADC_MAX;
        temp -= ADC_OFFSET;
        temp /= GAIN_FACTOR;

        sum_squares += temp * temp;
    }

    GPIO_writePin(inc, 0);

    return sqrtf(sum_squares / SAMPLE_COUNT);
}

//------------------------------------------------------------------------------------------------

//---------------------------AC Current-----------------------------------------------------------
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

//----------------------------------------------------------------------------------------------

//-----------------------------UART Function-----------------------------------------------------

void initUART0(void) {
  Device_init();
  Device_initGPIO();

  GPIO_setPinConfig(GPIO_35_SCIA_RX);
  GPIO_setPinConfig(GPIO_37_SCIA_TX);

  SCI_setConfig(
      SCIA_BASE, DEVICE_LSPCLK_FREQ,115200,
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
//uartPrintf("vout=%d\n", v_out);
//--------------------------------------------------------------------------------------------------


//----------------------------------------------Frequncy Measurment---------------------------------

float measureFrequency(uint32_t zcPin)
{
    uint32_t pulseCount = 0;
    uint32_t elapsed_us = 0;

    uint8_t lastState = GPIO_readPin(zcPin);

    while(elapsed_us < 300000)   // 300 ms
    {
        uint8_t currentState = GPIO_readPin(zcPin);

        if(currentState == 1 && lastState == 0)
        {
            pulseCount++;
        }

        lastState = currentState;

        DEVICE_DELAY_US(100);
        elapsed_us += 100;
    }

    float measurement_time = 0.3;  // 300 ms

    float frequency = pulseCount / measurement_time;

    // If 2 pulses per cycle:
    // frequency = frequency / 2.0;

    return frequency;
}

//------------------------------------------------------------------------------------------------

void main(void)
{
  initUART0();
  initADCA0();
  Device_initGPIO();
  lcd_begin(16, 2);

  GPIO_setPadConfig(inc, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(inc, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(dec, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(dec, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(UP, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(UP, GPIO_DIR_MODE_IN);

  GPIO_setPadConfig(DOWN, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(DOWN, GPIO_DIR_MODE_IN);


  GPIO_setPadConfig(SET, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(SET, GPIO_DIR_MODE_IN);

  GPIO_setPadConfig(HOME, GPIO_PIN_TYPE_PULLUP);
  GPIO_setDirectionMode(HOME, GPIO_DIR_MODE_IN);


  GPIO_setPadConfig(V_ZC, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(V_ZC, GPIO_DIR_MODE_IN);
  GPIO_setPadConfig(C_ZC, GPIO_PIN_TYPE_STD);
  GPIO_setDirectionMode(C_ZC, GPIO_DIR_MODE_IN);

    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();
    Device_initGPIO();

    // Disable all CPU interrupts and clear all CPU interrupt flags.
    DINT;
    IER = 0x0000;
    IFR = 0x0000;

    
    Interrupt_initVectorTable(); // Initializes the PIE vector table with pointers to the shell Interrupt
    Board_init();
    dataMutex = xSemaphoreCreateMutex();
    FreeRTOS_init();  // Configure FreeRTOS
}


//----------------------------------------Controll Task----------------------------------------------------------

void Control_Task(void *pvParameters)
{
   
    for(;;)
    {
        int v_out = (int)((0.9857f * ((int)(ac_voltage_rms()))) - 18.67f);
         float cur = ac_current_rms();
        //int fre = (int)(measureFrequency(V_ZC));
        if (v_out < 100 || v_out > 1000)
            v_out = 0;

          xSemaphoreTake(dataMutex, portMAX_DELAY);

        gData.voltage_out = v_out;
        gData.current = cur;
        gData.voltage_in = 240;
        gData.Frequncy = 50;
       

        xSemaphoreGive(dataMutex);

       uartPrintf("voltage = %d\n",(int)v_out);
       uartPrintf("cur = %d\n",cur);
       //uartPrintf("fre = %d\n",(int)fre);
       



        vTaskDelay(pdMS_TO_TICKS(20));  // control every 20ms
    }
}
const char* getonoff(uint8_t value)
{
    if(value == 1)
        return "ON";
    else
        return "OFF";
}

//------------------------------------------------------------------------------------------------------


//---------------------------------------------------UI Task---------------------------------------------

void UI_Task(void *pvParameters)
{
    UI_state display_state = NORMAL;

    int v_in,v_o,t,f,parameter_pre_temp=4;
       float c;
       uint8_t temp =0,d1=0,d2=0,d3=0,d4=0,edit_count=0;
       bool edit_screen = 0,blink_flag=0;
    
    for(;;)
    {
       
      

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        v_in = gData.voltage_in;
        c = gData.current;
        v_o = gData.voltage_out;
        f = gData.Frequncy;
        xSemaphoreGive(dataMutex);

        if(edit_screen == 0) {
        lcd_clear();
        lcd_setCursor(0,0);
        }
        else {
          lcd_setCursor(0,1);
          lcd_print("                ");
        }
        
         switch (display_state) {

          case NORMAL:
          lcd_printf("IP:%3d OP:%3d",v_in,v_o);
         lcd_setCursor(0,1);
         lcd_printf("OA:%d F:%d",(int)c*10,f);
         if(GPIO_readPin(SET) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case TRIP_LOGS:
         lcd_printf("TRIP LOG:");
         lcd_setCursor(0,1);
         lcd_printf("<--   M:01   -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = MANUAL_MODE;
         } 
         if(GPIO_readPin(SET) == 0) {
          display_state = EDIT;
          edit_screen =1;
         } 
         break;

         case MANUAL_MODE:
         lcd_printf("MANUAL MODE: ");
         lcd_printf(getonoff(1));
         lcd_setCursor(0,1);
         lcd_printf("<--   M:02   -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = SETTING;
         } 
         break;
         case SETTING:
         lcd_printf("SETTING :");
         lcd_setCursor(0,1);
         lcd_printf("<--   M:03   -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = FACT_SETTING;
         } 
         break;
         case FACT_SETTING:
         lcd_printf("FACT SETTING:");
         lcd_setCursor(0,1);
         lcd_printf("<--   M:04   -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = CALIBRATION;
         } 
         break;

         case CALIBRATION:
         lcd_printf("CALIBRATION:");
         lcd_setCursor(0,1);
         lcd_printf("<--   M:05   -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case REGULATION:
         lcd_printf("REGULATION: ");
         lcd_printf("%4d",parameter_pre_temp);
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P01 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case OP_CURRENT:
         lcd_printf("OP CURRENT: ");
         lcd_printf(getonoff(1));
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P02 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case OVERLOAD:
         lcd_printf("OVERLOAD:");
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P03 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case OVLD_TIME:
         lcd_printf("OVLD TIME:");
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P04 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case OVLD_IMD:
         lcd_printf("OVLD IMD:");
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P05 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case EARTH_FAIL:
         lcd_printf("EARTH FAIL: ");
         lcd_printf(getonoff(1));
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P06 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case EARTH_HIGH:
         lcd_printf("EARTH HIGH: ");
         lcd_printf("%4d",parameter_pre_temp);
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P07 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case EPOINT:
         lcd_printf("EPOINT FAIL:");
         lcd_printf(getonoff(1));
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P08 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case MOTOR_FAULT:
         lcd_printf("MOTOR FAULT: ");
         lcd_printf(getonoff(1));
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P09 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case FACT_PASSWORD:
         lcd_printf(" ENTER PASSWORD");
         lcd_setCursor(0,1);
         lcd_printf("<-- EDIT:P0 -->");
         if(GPIO_readPin(UP) == 0) {
          display_state = TRIP_LOGS;
         } 
         break;

         case EDIT:
         lcd_setCursor(0,1);
         lcd_printf("EDIT:  %d%d%d%d",d1,d2,d3,d4);
          
          if(blink_flag == 0) {
            lcd_setCursor(7+edit_count,1);
            lcd_printf("*");
            blink_flag=1;
          }
          else {
            blink_flag=0;
          }
          if(GPIO_readPin(SET) == 0) {
           ++edit_count;
            if(edit_count >= 4) {
              edit_screen = 0;
              display_state = NORMAL;
            }
         }
         if(GPIO_readPin(UP) == 0) {
          switch (edit_count) {
           case 0: ++d1;
           break;
           case 1: ++d2;
           break;
           case 2: ++d3;
           break;
           case 3: ++d4;
           break;
          }
         }
          break;
         }
    
         if(GPIO_readPin(UP) == 0) {
          ++temp;
         }
         else if(GPIO_readPin(DOWN) == 0) {
          --temp;
         }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // UI slower
    }
}

//--------------------------------------------------------------------------------------------------------


//----------------------------Stakeoverflow Hook--------------------------------------------------------
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

     uartPrintf("Stackoverflow\n");
    /* Run time stack overflow checking is performed if configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}

//---------------------------------------------------------------------------------------------------------

//----------------------------------Malloc Hook------------------------------------------------------------

void vApplicationMallocFailedHook( void )
{
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
    for( ;; );
}


