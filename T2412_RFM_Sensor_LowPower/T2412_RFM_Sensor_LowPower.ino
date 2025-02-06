/**
T2311_RFM69_Modem 
HW: Adafruit M0 RFM69 Feather or Arduino Pro Mini + RFM69

Send and receive data via UART

*******************************************************************************
https://github.com/infrapale/T2412_RFM_Sensor_LowPower
*******************************************************************************
https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio
https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide/all
*******************************************************************************


*******************************************************************************
Sensor Radio Message:   {"Z":"OD_1","S":"Temp","V":23.1,"R":"-"}
                        {"Z":"Dock","S":"T_dht22","V":"8.7","R":"-"}

*******************************************************************************
**/

#include <Arduino.h>
#include <Wire.h>
#include "main.h"
#include "watchdog.h"
#include "secrets.h"
#include <RH_RF69.h>
#include "atask.h"
#include "json.h"
#include "rfm69.h"
#include "rfm_send.h"
#include "io.h"
#include "xi2c.h"
#include "sensor.h"
#include "watchdog.h"

//*********************************************************************************************
#define SERIAL_BAUD   9600
#define ENCRYPTKEY    RFM69_KEY   // defined in secret.h
#define SLEEP_TIME_MS  3000


void debug_print_task(void);
void run_100ms(void);

atask_st debug_print_handle         = {"Debug Print    ", 5000,0, 0, 255, 0, 1, debug_print_task};
atask_st clock_handle               = {"Tick Task      ", 100, 0, 0, 255, 0, 1, run_100ms};


void initialize_tasks(void)
{
  atask_initialize();
  atask_add_new(&debug_print_handle);
  atask_add_new(&clock_handle);
}


void setup() 
{
    io_initialize();
    //while (!Serial); // wait until serial console is open, remove if not tethered to computer
    delay(2000);
    Serial.begin(9600);
    Serial.print(F("T2412_RFM_Sensor_LowPower")); Serial.print(F(" Compiled: "));
    Serial.print(F(__DATE__)); Serial.print(F(" "));
    Serial.print(F(__TIME__)); Serial.println();
    Serial.flush();
    // watchdog_initialize();
    #ifdef  ADA_M0_RFM69
        SerialX.begin(9600);
    #endif
    // Serial.println(F("sensor_initialize() ..."));
    // sensor_initialize();
    // Serial.println(F("sensor_initialize() is ready"));
    Wire.begin();

    #ifdef SLEEP_CTRL
      xi2c_initialize(SLEEP_CTRL_I2C_ADDR);
      delay(10);
      xi2c_set_wd_timeout(60000);
      delay(10);
    #endif

    Wire.begin();

    watchdog_clear_local();
    rfm69_initialize();
    initialize_tasks();
    sensor_initialize();
    Serial.println(F("Setup() is ready"));
}

void loop() 
{
    atask_run();  
    // io_toggle_clear_wd();

}

void run_100ms(void)
{
    static uint8_t ms100 = 0;

    if (++ms100 >= 10 )
    {
        io_toggle_clear_wd();
        ms100 = 0;
    }
    io_run_100ms();
}

void debug_print_task(void)
{
  atask_print_status(true);
}

