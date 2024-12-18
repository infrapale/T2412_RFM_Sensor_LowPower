#include "main.h"
#include "sensor.h"
#include "atask.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme(&Wire); // I2C


typedef struct 
{
    uint8_t indx;
    uint8_t bme680_addr;
    uint8_t ldr1_pin;
    uint8_t ldr2_pin;
    float   temperature;
    float   pressure;
    float   humidity;
    float   gas;
    float   co2;
    float   ldr1;
} sensor_ctrl_st;

void sensor_task(void);
atask_st sensor_handle     = {"Sensor Task    ", 1000,0, 0, 255, 0, 1, &sensor_task};
sensor_ctrl_st sensor_ctrl;

#ifdef SENSOR_UNDEFINED
  #define NBR_OF_VALUES 3
  #define USE_BME680
  sensor_t sensor[NBR_OF_VALUES] = 
  {
    {"Undef", "Temp", true, 0.0, 10000},
    {"Undef", "Hum",  true, 0.0, 10000},
    {"Undef", "LDR",  true, 0.0, 10000},
  };
#endif

#ifdef SENSOR_VA_TUPA
  #define NBR_OF_VALUES 2
  sensor_st sensor[NBR_OF_VALUES] = 
  {
    {"VA_Tupa", "Temp", true, 0.0, 10000},
    {"VA_Tupa", "Hum",  true, 0.0, 10000},
  }; 
#endif

void sensor_initialize(void)
{
  sensor_ctrl.bme680_addr = 0x00;
  sensor_ctrl.ldr1_pin = 255;
  sensor_ctrl.ldr2_pin = 255;

  #ifdef SENSOR_UNDEFINED
      sensor_ctrl.bme680_addr = 0x40;
  #endif

  #ifdef  SENSOR_VA_TUPA
      sensor_ctrl.bme680_addr = 0x44;
      sensor_ctrl.ldr1_pin = 2;
  #endif

  #ifdef USE_BME680
      if (!bme.begin()) {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        while (1);
      }
      sensor_ctrl.
        // Set up oversampling and filter initialization
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150); // 320*C for 150 ms
  #endif
  atask_add_new(&sensor_handle);
  sensor_ctrl.indx = 0;
}

bool sensor_read_bme680(void)
{
    bool bme680_ok = false;
    if (bme.performReading()) bme680_ok = true;

    if (bme680_ok ) 
    {
        sensor_ctrl.temperature = bme.temperature;
        sensor_ctrl.pressure = bme.pressure ;
        sensor_ctrl.humidity = bme.humidity ;
        sensor_ctrl.gas = bme.gas_resistance ;
    }
    else
    {
      Serial.println("Failed to perform reading :(");
    }  
    return bme680_ok;
}

void sensor_read_ldr1(void)
{
  
}


void sensor_task(void)
{

    switch(sensor_handle.state)
    {
        case 0:
          sensor_handle.state = 10;
          break;
        case 10:
          if (sensor_ctrl.bme680_addr != 0x00)
          {
            sensor_read_bme680();
          }
          if (sensor_ctrl.ldr1_pin < 255) sensor_read_ldr1();

          break;

    }

}
