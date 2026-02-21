//#############################################################################
//
// FILE:   F28E12x_EEPROM_DataStorage.c
// DESCRIPTION: ULTRA SIMPLE EEPROM test - just write and read a pattern
//#############################################################################

#include "EEPROM_Config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//------------------------------UART Functions-----------------------------------
void initUART0(void) {
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

//------------------------------SIMPLE TEST STRUCTURE-----------------------------------
typedef struct {
    uint16 magic;           // Should be 0xA55A
    uint32 counter;         // Incrementing counter
    float voltage;          // Test voltage
    uint16 checksum;        // Simple checksum
} TestData_t;

//------------------------------Global Variables-----------------------------------
TestData_t g_test_data;
uint16 g_write_buffer[20];  // Buffer for EEPROM operations

// Forward declarations
extern void Example_Done(void);
extern void Sample_Error(void);

//------------------------------SIMPLE WRITE TEST-----------------------------------
void SimpleWriteTest(void) {
    uint16 i;
    
    uartPrintf("\n\n=== SIMPLE EEPROM WRITE TEST ===\r\n");
    
    // Prepare test data
    g_test_data.magic = 0xA55A;
    g_test_data.counter = 12345;
    g_test_data.voltage = 24.5f;
    g_test_data.checksum = 0xFFFF;
    
    uartPrintf("Writing to EEPROM:\r\n");
    uartPrintf("  Magic: 0x%04X\r\n", g_test_data.magic);
    uartPrintf("  Counter: %lu\r\n", g_test_data.counter);
    uartPrintf("  Voltage: %d.%d V\r\n", (int)g_test_data.voltage, 
               (int)((g_test_data.voltage - (int)g_test_data.voltage)*10));
    
    // Copy to buffer
    memcpy(g_write_buffer, &g_test_data, sizeof(TestData_t));
    
    // Show what we're writing
    uartPrintf("Write buffer (first 8 words): ");
    for (i = 0; i < 8; i++) {
        uartPrintf("0x%04X ", g_write_buffer[i]);
    }
    uartPrintf("\r\n");
    
    // Write to EEPROM
    #ifdef _64_BIT_MODE
    uartPrintf("Writing in 64-bit mode...\r\n");
    EEPROM_Program_64_Bits(4, &g_write_buffer[0]);
    #else
    uartPrintf("Writing in page mode...\r\n");
    EEPROM_Write(g_write_buffer);
    #endif
    
    uartPrintf("Write complete!\r\n");
}

//------------------------------SIMPLE READ TEST-----------------------------------
void SimpleReadTest(void) {
    TestData_t read_data;
    uint16 i;
    
    uartPrintf("\n=== SIMPLE EEPROM READ TEST ===\r\n");
    
    // Clear buffer
    memset(g_write_buffer, 0, sizeof(g_write_buffer));
    
    // Read from EEPROM
    #ifdef _64_BIT_MODE
    uartPrintf("Reading in 64-bit mode...\r\n");
    EEPROM_Read(&g_write_buffer[0]);
    #else
    uartPrintf("Reading in page mode...\r\n");
    EEPROM_Read(g_write_buffer);
    #endif
    
    // Show what we read
    uartPrintf("Read buffer (first 8 words): ");
    for (i = 0; i < 8; i++) {
        uartPrintf("0x%04X ", g_write_buffer[i]);
    }
    uartPrintf("\r\n");
    
    // Copy to structure
    memcpy(&read_data, g_write_buffer, sizeof(TestData_t));
    
    uartPrintf("Read from EEPROM:\r\n");
    uartPrintf("  Magic: 0x%04X\r\n", read_data.magic);
    uartPrintf("  Counter: %lu\r\n", read_data.counter);
    uartPrintf("  Voltage: %d.%d V\r\n", (int)read_data.voltage, 
               (int)((read_data.voltage - (int)read_data.voltage)*10));
    
    // Verify
    if (read_data.magic == 0xA55A && read_data.counter == 12345) {
        uartPrintf("\n*** SUCCESS! EEPROM IS WORKING! ***\r\n");
    } else {
        uartPrintf("\n*** FAILED! EEPROM NOT WORKING! ***\r\n");
        uartPrintf("Expected magic: 0xA55A, Got: 0x%04X\r\n", read_data.magic);
    }
}

//------------------------------MAIN-----------------------------------
void main(void) {
    uint16 status;
    uint16 eepromConfigCheck;
    uint16 i;

    // Initialize device
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    
    EINT;
    ERTM;
    
    // Initialize UART
    initUART0();
    
    uartPrintf("\n\n=== F28E12x EEPROM TEST ===\r\n");
    
    // Initialize Flash
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 3);
    
    EALLOW;
    
    // Initialize Flash API
    status = Fapi_initializeAPI(FlashTech_CPU0_BASE_ADDRESS, 150);
    if (status != Fapi_Status_Success) {
        uartPrintf("ERROR: Flash API init failed! Code: %d\r\n", status);
        Sample_Error();
    }
    
    // Configure EEPROM
    eepromConfigCheck = EEPROM_Config_Check();
    if (eepromConfigCheck == 0xFFFF || eepromConfigCheck == 0xEEEE || eepromConfigCheck == 0xCCCC) {
        uartPrintf("ERROR: EEPROM config failed! Code: 0x%04X\r\n", eepromConfigCheck);
        Sample_Error();
    }
    
    uartPrintf("EEPROM configured successfully\r\n");
    
    // First, read what's currently in EEPROM
    uartPrintf("\n--- Initial EEPROM contents ---\r\n");
    SimpleReadTest();
    
    // Now write test data
    uartPrintf("\n--- Writing test data ---\r\n");
    SimpleWriteTest();
    
    // Read back immediately to verify write worked
    uartPrintf("\n--- Reading back immediately ---\r\n");
    SimpleReadTest();
    
    uartPrintf("\n\n=== Test Complete ===\r\n");
    uartPrintf("Press RESET and run again - the data should persist!\r\n");
    
    Example_Done();
}
