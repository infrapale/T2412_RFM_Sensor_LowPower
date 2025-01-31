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

RH_RF69         rf69(RFM69_CS, RFM69_INT);
RH_RF69         *rf69p;

#define NBR_TEST_MSG  4
#define LEN_TEST_MSG  32

void debug_print_task(void);
void run_100ms(void);

atask_st debug_print_handle         = {"Debug Print    ", 5000,0, 0, 255, 0, 1, debug_print_task};
//atask_st clock_handle               = {"Tick Task      ", 100, 0, 0, 255, 0, 1, run_100ms};

void initialize_tasks(void)
{
  atask_initialize();
  atask_add_new(&debug_print_handle);
  //atask_add_new(&clock_handle);
  Serial.print(F("Tasks initialized ")); Serial.println(TASK_NBR_OF);
}


void setup() 
{
    //while (!Serial); // wait until serial console is open, remove if not tethered to computer
    delay(2000);
    Serial.begin(9600);
    Serial.print(F("T2412_RFM_Sensor_LowPower")); Serial.print(F(" Compiled: "));
    Serial.print(F(__DATE__)); Serial.print(F(" "));
    Serial.print(F(__TIME__)); Serial.println();
    Serial.flush();
    watchdog_initialize();
    #ifdef  ADA_M0_RFM69
        SerialX.begin(9600);
    #endif
    Wire.begin();

    watchdog_clear_local();

    //send_p = rfm_send_get_data_ptr();
    rf69p = &rf69;
    rfm69_initialize(&rf69);
    io_initialize();
    initialize_tasks();
    sensor_initialize();

    Serial.println(F("Setup() is ready"));
}

void loop() 
{
    atask_run();  
}

void run_100ms(void)
{
}

void debug_print_task(void)
{
  atask_print_status(true);
}


#ifdef SEND_TEST_MSG
    void send_test_data_task(void)
    {
        static uint8_t indx = 0;
        static uint8_t cntr = 0;
        switch(send_test_data_handle.state)
        {
          case 0:
            send_test_data_handle.state = 10;
            break;
          case 10:
            rfm_send_clr_flag_msg_was_sent();
            //uart_p->rx.str  = test_msg[indx];
            //uart_p->rx.avail = true;
            rfm_send_radiate_msg( test_msg[indx] );
            send_test_data_handle.state = 11;
            break;
          case 11:
            if (rfm_send_get_flag_msg_was_sent())
            {
              if (++indx >= 1 )
              {
                send_test_data_handle.state = 100;
                xi2c_set_sleep_time(60);
                
                //xi2c_set_wd_timeout(0x2345);
              }
              else
              {
                send_test_data_handle.state = 12;
                cntr = 0;
              }
            }
            break;
          case 12:
            if(++cntr > 4) send_test_data_handle.state = 10;
            break;
          case 100:
            break;
      
        }
    }
#endif
