#include "Arduino_OpenPLC.h"
#include "defines.h"

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

#ifdef MODBUS_ENABLED
#include "ModbusSlave.h"
#endif

//Include WiFi lib to turn off WiFi radio on ESP32 and ESP8266 boards if we're not using WiFi
#ifndef MBTCP
    #if defined(BOARD_ESP8266)
        #include <ESP8266WiFi.h>
    #elif defined(BOARD_ESP32)
        #include <WiFi.h>
    #endif
#endif

unsigned long __tick = 0;

unsigned long scan_cycle;

#include "arduino_libs.h"

extern uint8_t pinMask_DIN[];
extern uint8_t pinMask_AIN[];
extern uint8_t pinMask_DOUT[];
extern uint8_t pinMask_AOUT[];

void setupCycleDelay(unsigned long long cycle_time)
{
    scan_cycle = (uint32_t)(cycle_time/1000000);
}

void setup() 
{
    //Turn off WiFi radio on ESP32 and ESP8266 boards if we're not using WiFi
    #ifndef MBTCP
        #if defined(BOARD_ESP8266) || defined(BOARD_ESP32)
            WiFi.mode(WIFI_OFF);
        #endif
    #endif
    config_init__();
    glueVars();
    hardwareInit();
    rp2040.enableDoubleResetBootloader();
    Serial.begin();
	#ifdef MODBUS_ENABLED
        #ifdef MBSERIAL
	        //Config Modbus Serial (port, speed, rs485 tx pin)
            #ifdef MBSERIAL_TXPIN
                //Disable TX pin from OpenPLC hardware layer
                for (int i = 0; i < NUM_DISCRETE_INPUT; i++)
                {
                    if (pinMask_DIN[i] == MBSERIAL_TXPIN)
                        pinMask_DIN[i] = 255;
                }
                for (int i = 0; i < NUM_ANALOG_INPUT; i++)
                {
                    if (pinMask_AIN[i] == MBSERIAL_TXPIN)
                        pinMask_AIN[i] = 255;
                }
                for (int i = 0; i < NUM_DISCRETE_OUTPUT; i++)
                {
                    if (pinMask_DOUT[i] == MBSERIAL_TXPIN)
                        pinMask_DOUT[i] = 255;
                }
                for (int i = 0; i < NUM_ANALOG_OUTPUT; i++)
                {
                    if (pinMask_AOUT[i] == MBSERIAL_TXPIN)
                        pinMask_AOUT[i] = 255;
                }
                mbconfig_serial_iface(&MBSERIAL_IFACE, MBSERIAL_BAUD, MBSERIAL_TXPIN);
            #else
                mbconfig_serial_iface(&MBSERIAL_IFACE, MBSERIAL_BAUD, -1);
            #endif
	
	        //Set the Slave ID
	        modbus.slaveid = MBSERIAL_SLAVE;
        #endif
    
        #ifdef MBTCP
        uint8_t mac[] = { MBTCP_MAC };
        uint8_t ip[] = { MBTCP_IP };
        uint8_t dns[] = { MBTCP_DNS };
        uint8_t gateway[] = { MBTCP_GATEWAY };
        uint8_t subnet[] = { MBTCP_SUBNET };
        
        if (sizeof(ip)/sizeof(uint8_t) < 4)
            mbconfig_ethernet_iface(mac, NULL, NULL, NULL, NULL);
        else if (sizeof(dns)/sizeof(uint8_t) < 4)
            mbconfig_ethernet_iface(mac, ip, NULL, NULL, NULL);
        else if (sizeof(gateway)/sizeof(uint8_t) < 4)
            mbconfig_ethernet_iface(mac, ip, dns, NULL, NULL);
        else if (sizeof(subnet)/sizeof(uint8_t) < 4)
            mbconfig_ethernet_iface(mac, ip, dns, gateway, NULL);
        else
            mbconfig_ethernet_iface(mac, ip, dns, gateway, subnet);
        #endif
        
        //Add all modbus registers
        init_mbregs(MAX_ANALOG_OUTPUT, MAX_DIGITAL_OUTPUT, MAX_ANALOG_INPUT, MAX_DIGITAL_INPUT);
        mapEmptyBuffers();
	#endif

    setupCycleDelay(common_ticktime__);
    rp2040.wdt_begin( scan_cycle * 5 );
    xTaskCreate( cycleTask, "CycleTask", 256, NULL, 7, NULL );

    #ifdef MODBUS_ENABLED
    xTaskCreate( modbusProcTask, "Modbus", 256, NULL, 4, NULL );
    #endif
}

void setup1() {
    delay(1000);
}

#ifdef MODBUS_ENABLED
void mapEmptyBuffers()
{
    //Map all empty I/O buffers to Modbus registers
    for (int i = 0; i < MAX_DIGITAL_OUTPUT; i++)
    {
        if (bool_output[i/8][i%8] == NULL)
        {
			bool_output[i/8][i%8] = (IEC_BOOL *)malloc(sizeof(IEC_BOOL));
			*bool_output[i/8][i%8] = 0;
        }
    }
    for (int i = 0; i < MAX_ANALOG_OUTPUT; i++)
    {
        if (int_output[i] == NULL)
        {
			int_output[i] = (IEC_UINT *)(modbus.holding + i);
        }
    }
    for (int i = 0; i < MAX_DIGITAL_INPUT; i++)
    {
        if (bool_input[i/8][i%8] == NULL)
        {
            bool_input[i/8][i%8] = (IEC_BOOL *)malloc(sizeof(IEC_BOOL));
			*bool_input[i/8][i%8] = 0;
        }
    }
    for (int i = 0; i < MAX_ANALOG_INPUT; i++)
    {
        if (int_input[i] == NULL)
        {
			int_input[i] = (IEC_UINT *)(modbus.input_regs + i);
        }
    }
}
void syncModbusBuffers()
{
    //Sync OpenPLC Buffers with Modbus Buffers	
    for (int i = 0; i < MAX_DIGITAL_OUTPUT; i++)
    {
        if (bool_output[i/8][i%8] != NULL)
        {
            write_discrete(i, COILS, (bool)*bool_output[i/8][i%8]);
        }
    }
    for (int i = 0; i < MAX_ANALOG_OUTPUT; i++)
    {
        if (int_output[i] != NULL)
        {
            modbus.holding[i] = *int_output[i];
        }
    }
    for (int i = 0; i < MAX_DIGITAL_INPUT; i++)
    {
        if (bool_input[i/8][i%8] != NULL)
        {
            write_discrete(i, INPUTSTATUS, (bool)*bool_input[i/8][i%8]);
        }
    }
    for (int i = 0; i < MAX_ANALOG_INPUT; i++)
    {
        if (int_input[i] != NULL)
        {
            modbus.input_regs[i] = *int_input[i];
        }
    }
    
    //Write changes back to OpenPLC Buffers
    for (int i = 0; i < MAX_DIGITAL_OUTPUT; i++)
    {
        if (bool_output[i/8][i%8] != NULL)
        {
            *bool_output[i/8][i%8] = get_discrete(i, COILS);
        }
    }
    for (int i = 0; i < MAX_ANALOG_OUTPUT; i++)
    {
        if (int_output[i] != NULL)
        {
            *int_output[i] = modbus.holding[i];
        }
    }
}

void modbusProcTask( void *pvParameters )
{
    for (;;) {
        mbtask();
        vTaskDelay(1);
    }
}
#endif


void loop() 
{
}

void cycleTask( void *pvParameters ) 
{
    TickType_t xLastWakeTime;
    for (;;) {
        xLastWakeTime = xTaskGetTickCount();
        #ifdef MODBUS_ENABLED
        if( mbmutex_get( 0 ) ) {
        #endif
            rp2040.wdt_reset();
            updateInputBuffers();
            config_run__(__tick++);
            updateOutputBuffers();
            updateTime();
            #ifdef MODBUS_ENABLED
            syncModbusBuffers();
            mbmutex_release();
            #endif
            vTaskDelayUntil( &xLastWakeTime, scan_cycle );
        #ifdef MODBUS_ENABLED
        }
        #endif
    }
}