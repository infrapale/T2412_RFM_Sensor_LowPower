#include "main.h"
#include "sensor.h"
#include "atask.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_SCD30.h>
#include <Adafruit_PCT2075.h>
#include "xi2c.h"
#include "json.h"
#include "rfm_send.h"

#define SEALEVELPRESSURE_HPA (1013.25)


typedef struct 
{
    uint8_t indx;
    char    radio_msg[MAX_MESSAGE_LEN];
    uint32_t  next_radiate;
} sensor_ctrl_st;

typedef struct
{
  char                  label[SENSOR_NAME_LEN];
  sensor_value_type_et  type;
  uint8_t               addr;
  float                 value;
  bool                  updated;
  uint32_t              next_ms;
  uint32_t              interval_ms;          
 } value_st;

typedef struct
{
  char        zone[ZONE_NAME_LEN];
  sensor_et   sensor_type;
  value_st   values[SENSOR_MAX_VALUES];
 // uint32_t  interval;

} sensor_st;


extern atask_st sensor_handle; 
sensor_ctrl_st sensor_ctrl;

#ifdef ZONE_VA_TUPA
  #define NBR_OF_SENSORS 2
  #define BME680_ADDR     0x76
  #define USE_BME680
  
  sensor_st sensor[NBR_OF_SENSORS] =
  {
    {
      "VA_Tupa",
      .sensor_type = SENSOR_BME680,
      {
        {"Temp", SENSOR_VALUE_TEMPERATURE,  BME680_ADDR, 0.0, false, 00000, 30000 },
        {"Hum",  SENSOR_VALUE_HUMIDITY,     BME680_ADDR, 0.0, false, 20000, 60000 },
        {"Pres", SENSOR_VALUE_PRESSURE,     BME680_ADDR, 0.0, false, 30000, 120000 },
        {"Gas",  SENSOR_VALUE_GAS,          BME680_ADDR, 0.0, false, 40000, 60000 },
      }
    },
    {
      "VA_Tupa",
      .sensor_type = SENSOR_LDR,
      {
        {"LDR1",  SENSOR_VALUE_LDR, A0, 0.0, false, 50000, 60000 },
        {"LDR2",  SENSOR_VALUE_LDR, A1, 0.0, false, 55000, 60000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false, 0, 60000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false , 0, 60000},
      }
    }
  };
    
#endif

#ifdef ZONE_TEST
  #define NBR_OF_SENSORS 1
  #define SCD30_ADDR     0x61
  #define USE_SCD30
  
  sensor_st sensor[NBR_OF_SENSORS] =
  {
    {
      "VA_Varasto",
      .sensor_type = SENSOR_PCT2075,
      {
        {"Temp", SENSOR_VALUE_TEMPERATURE,  SCD30_ADDR, 0.0, false, 10000, 60000 },
        {"Hum",  SENSOR_VALUE_HUMIDITY,     SCD30_ADDR, 0.0, false, 20000, 60000 },
        {"CO2",  SENSOR_VALUE_CO2,          SCD30_ADDR, 0.0, false, 30000, 60000 },
        {"-",    SENSOR_VALUE_UNDEFINED,    SCD30_ADDR, 0.0, false, 40000, 60000 },
      }
    }
  };
    
#endif

#ifdef ZONE_VA_VARASTO
  #define NBR_OF_SENSORS 1
  #define PCT2075_ADDR   0x37
  #define USE_PCT2075
  
  sensor_st sensor[NBR_OF_SENSORS] =
  {
    {
      "Test",
      .sensor_type = SENSOR_SCD30,
      {
        {"Temp", SENSOR_VALUE_TEMPERATURE,  PCT2075_ADDR, 0.0, false, 10000, 60000 },
        {"-",    SENSOR_VALUE_UNDEFINED,    PCT2075_ADDR, 0.0, false, 20000, 60000 },
        {"-",    SENSOR_VALUE_UNDEFINED,    PCT2075_ADDR, 0.0, false, 30000, 60000 },
        {"-",    SENSOR_VALUE_UNDEFINED,    PCT2075_ADDR, 0.0, false, 40000, 60000 },
      }
    }
  };
    
#endif

#ifdef USE_BME680
Adafruit_BME680 bme(&Wire); // I2C
#endif

#ifdef USE_SCD30
  Adafruit_SCD30  scd30;
#endif

#ifdef USE_PCT2075
  Adafruit_PCT2075 PCT2075;
#endif


void sensor_initialize(void)
{
  sensor_ctrl.next_radiate = millis() + 20000;
  Serial.println(F("Sensor Setup"));
  for(uint8_t sindx = 0; sindx < NBR_OF_SENSORS; sindx++)
  {
      if(sensor[sindx].sensor_type != SENSOR_UNDEFINED)
      {
          for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
          {
            Serial.print(sensor[sindx].zone);
            Serial.print(F(" "));
            Serial.println(sensor[sindx].values[vindx].label);
          }
      }
  }


  Serial.println(F(""));

  #ifdef USE_BME680
      if (!bme.begin(BME680_ADDR)) {
        Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
        while (1);
      }
      delay(1000);
        // Set up oversampling and filter initialization
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(320, 150); // 320*C for 150 ms
  #endif

  #ifdef USE_PCT2075
      if (!PCT2075.begin(PCT2075_ADDR)) 
      {
        Serial.println(F("Could not find a valid PCT2075 sensor, check wiring!"));
        while (1) delay(10);
      }

  #endif

  #ifdef USE_SCD30
    if (!scd30.begin(SCD30_ADDR)) 
    {
      Serial.println(F("Failed to find SCD30 chip"));
      while (1) { delay(10);} 
    }
    Serial.println(F("SCD30 Found!"));
    Serial.print(F("Measurement Interval: ")); 
    Serial.print(scd30.getMeasurementInterval()); 
    Serial.println(F(" seconds"));


  #endif
  // Initialize LDR
  for(uint8_t sindx = 0; sindx < NBR_OF_SENSORS; sindx++)
  {
      if(sensor[sindx].sensor_type == SENSOR_LDR)
      {
          for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
          {
            uint8_t ldr_pin = sensor[sindx].values[vindx].addr;
            if (sensor[sindx].values[vindx].value != NULL) pinMode(ldr_pin, INPUT);
          }
      }
  }
  sensor_ctrl.indx = 0;
}

bool sensor_read_bme680(uint8_t indx)
{
    bool bme680_ok = false;
    #ifdef USE_BME680
      if (bme.performReading()) bme680_ok = true;

      if (bme680_ok ) 
      { 
          for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
          {
              switch(sensor[indx].values[vindx].type)
              {
                  case SENSOR_VALUE_TEMPERATURE:
                      sensor[indx].values[vindx].value = bme.temperature;
                      sensor[indx].values[vindx].updated = true;
                      break;
                  case SENSOR_VALUE_HUMIDITY:
                      sensor[indx].values[vindx].value = bme.humidity;
                      sensor[indx].values[vindx].updated = true;
                      break;
                  case SENSOR_VALUE_PRESSURE:
                      sensor[indx].values[vindx].value = bme.pressure;
                      sensor[indx].values[vindx].updated = true;
                      break;
                  case SENSOR_VALUE_GAS:
                      sensor[indx].values[vindx].value = bme.gas_resistance;
                      sensor[indx].values[vindx].updated = true;
                      break;
              }
          }
      }
      else
      {
        Serial.println("Failed to perform reading :(");
      }  
    #endif
    return bme680_ok;
}

bool sensor_read_scd30(uint8_t indx)
{
    bool scd30_ok = false;;
    #ifdef USE_SCD30
      if (scd30.dataReady())
      {
        Serial.println("Data available!");

        if (!scd30.read())
        { 
          Serial.println("Error reading sensor data");
        }
        else 
        {
          for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
          {
          switch(sensor[indx].values[vindx].type)
          {
              case SENSOR_VALUE_TEMPERATURE:
                  sensor[indx].values[vindx].value = scd30.temperature;
                  sensor[indx].values[vindx].updated = true;
                  break;
              case SENSOR_VALUE_HUMIDITY:
                  sensor[indx].values[vindx].value = scd30.relative_humidity;
                  sensor[indx].values[vindx].updated = true;
                  break;
              case SENSOR_VALUE_CO2:
                  sensor[indx].values[vindx].value = scd30.CO2;
                  sensor[indx].values[vindx].updated = true;
                  break;
          }
        }

      }
    }
    #endif  
    return scd30_ok;
}

void sensor_read_pct2075(uint8_t sindx)
{
    for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
    {
        switch(sensor[sindx].values[vindx].type)
        {
            case SENSOR_VALUE_TEMPERATURE:
                uint8_t ldr_pin = sensor[sindx].values[vindx].addr;
                sensor[sindx].values[vindx].value = PCT2075.getTemperature();
                sensor[sindx].values[vindx].updated = true;
                break;
        }    
    }
}

void sensor_read_ldr(uint8_t sindx)
{
    for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
    {
        switch(sensor[sindx].values[vindx].type)
        {
            case SENSOR_VALUE_LDR:
                uint8_t ldr_pin = sensor[sindx].values[vindx].addr;
                sensor[sindx].values[vindx].value = (float) analogRead(ldr_pin);
                sensor[sindx].values[vindx].updated = true;
                break;
        }    
    }
}


void sensor_read_all(void)
{
    for(uint8_t sindx = 0; sindx < NBR_OF_SENSORS; sindx++)
    {
        switch(sensor[sindx].sensor_type)
        {
            case SENSOR_BME680:
                sensor_read_bme680(sindx);
                break;
            case SENSOR_LDR:
                sensor_read_ldr(sindx);
                break;
            case SENSOR_SCD30:
                sensor_read_scd30(sindx);
                break;
            case SENSOR_PCT2075:
                sensor_read_pct2075(sindx);
                break;
        }
    }
}

void sensor_print_value(uint8_t sindx, uint8_t vindx)
{
    if((sensor[sindx].sensor_type != SENSOR_VALUE_UNDEFINED)  && 
      (sensor[sindx].values[vindx].type != SENSOR_VALUE_UNDEFINED) && 
      (millis() > sensor_ctrl.next_radiate)) 
    {
        if (millis() > sensor[sindx].values[vindx].next_ms)
        {
            sensor[sindx].values[vindx].next_ms = millis() + sensor[sindx].values[vindx].interval_ms;
            Serial.print("Sensor :"); Serial.print(sindx);
            Serial.print(" value# "); Serial.print(vindx);
            Serial.print(" -> ");
            Serial.print(sensor[sindx].zone); Serial.print(" ");
            Serial.print(sensor[sindx].values[vindx].label); Serial.print(" ");
            Serial.print(sensor[sindx].values[vindx].value);
            Serial.println();


            json_convert_sensor_float_to_json(
                sensor_ctrl.radio_msg, 
                sensor[sindx].zone, 
                sensor[sindx].values[vindx].label, 
                sensor[sindx].values[vindx].value, 
                "-" );


            rfm_send_radiate_msg(sensor_ctrl.radio_msg);
            sensor_ctrl.next_radiate = millis() + 2000;

        }
    }
}

void sensor_print_all(void)
{
    for(uint8_t sindx = 0; sindx < NBR_OF_SENSORS; sindx++)
    {
        for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
        {
             sensor_print_value(sindx, vindx);
        }
    }

}

void sensor_task(void)
{
    static uint32_t next_send_ms = 0;
    static uint8_t  sensor_index = 0;
    static uint8_t  value_index = 0;
    bool all_done;

    switch(sensor_handle.state)
    {
        case 0:
          sensor_handle.state = 10;
          break;
        case 10:
          sensor_read_all();
          sensor_handle.state = 20;
          break;
        case 20:
          if(millis() > next_send_ms ) sensor_handle.state = 30;
          break;
        case 30:
          sensor_print_value(sensor_index, value_index);
          all_done = false;
          if (++value_index >= SENSOR_MAX_VALUES)
          {
            if (++sensor_index >= NBR_OF_SENSORS)
            {
                sensor_index = 0;
                all_done = true;
                next_send_ms = millis() + 10000;
                sensor_handle.state = 10;
                xi2c_set_sleep_time(10000);
            }
            value_index = 0;
          }
          if (!all_done)
          {
            if (sensor[sensor_index].values[value_index].type == SENSOR_VALUE_UNDEFINED) 
            {
              next_send_ms = millis();
            }
            else 
            {
              next_send_ms = millis() + 2000;
              sensor_handle.state = 20;
            }
          }
          break;

    }

}
