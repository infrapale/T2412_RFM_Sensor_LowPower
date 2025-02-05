/*
https://learn.adafruit.com/adafruit-veml7700/overview
https://learn.adafruit.com/adafruit-mcp9808-precision-i2c-temperature-sensor-guide
*/
#include "Arduino.h"
#include "main.h"
#include "watchdog.h"
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
#include "Adafruit_MCP9808.h"
#include "Adafruit_VEML7700.h"
#include "io.h"


// Create the MCP9808 temperature sensor object

#define SEALEVELPRESSURE_HPA (1013.25)


typedef struct 
{
    uint8_t indx;
    char    radio_msg[MAX_MESSAGE_LEN];
    //uint32_t  next_action;

    uint8_t read_task_indx;
    uint8_t send_task_indx;
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


atask_st sensor_read_handle         = {"Sensor Read    ", 1000, 0, 0, 255, 0, 1, &sensor_read_task};
atask_st sensor_send_handle         = {"Sensor Send    ", 1000, 0, 0, 255, 0, 1, &sensor_send_task};


sensor_ctrl_st sensor_ctrl;

#ifdef ZONE_VA_TUPA
  #define NBR_OF_SENSORS 2
  #define BME680_ADDR     0x76
  #define USE_BME680
  #define LDR_PIN         A0
  
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
        {"LDR1",  SENSOR_VALUE_LDR, LDR_PIN, 0.0, false, 50000, 60000 },
        {"LDR2",  SENSOR_VALUE_LDR, A1, 0.0, false, 55000, 60000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false, 0, 60000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false , 0, 60000},
      }
    }
  };   
#endif


#ifdef ZONE_VA_OD
  #define NBR_OF_SENSORS 2
  #define MCP9808_ADDR      0x18
  #define VEML7700_ADDR     0x10
  
  sensor_st sensor[NBR_OF_SENSORS] =
  {
    {
      "VA_OD",
      .sensor_type = SENSOR_MCP9808,
      {
        {"Temp", SENSOR_VALUE_TEMPERATURE,  MCP9808_ADDR, 0.0, false, 10000, 300000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false, 0, 60000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false, 0, 60000 },
        {"-",     SENSOR_VALUE_UNDEFINED, 0u, 0.0, false, 0, 60000 },
      }
    },    
    {
      "VA_OD",
      .sensor_type = SENSOR_VEML7700,
      {
        {"Lux",   SENSOR_VALUE_LUX,       VEML7700_ADDR, 0.0, false, 10000, 300000 },
        {"ALS",   SENSOR_VALUE_ALS,       VEML7700_ADDR, 0.0, false, 10000, 300000 },
        {"White", SENSOR_VALUE_WHITE,     VEML7700_ADDR, 0.0, false, 10000, 300000 },
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
        {"Temp", SENSOR_VALUE_TEMPERATURE,  SCD30_ADDR, 0.0, false, 10000, 300000 },
        {"Hum",  SENSOR_VALUE_HUMIDITY,     SCD30_ADDR, 0.0, false, 20000, 300000 },
        {"CO2",  SENSOR_VALUE_CO2,          SCD30_ADDR, 0.0, false, 30000, 300000 },
        {"-",    SENSOR_VALUE_UNDEFINED,    SCD30_ADDR, 0.0, false, 40000, 300000 },
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
      "Varasto",
      .sensor_type = SENSOR_PCT2075,
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

#ifdef MCP9808_ADDR
  Adafruit_MCP9808 mcp9808_sensor = Adafruit_MCP9808();
#endif

#ifdef VEML7700_ADDR
Adafruit_VEML7700 veml = Adafruit_VEML7700();
#endif


void sensor_initialize(void)
{
  //io_pwr_on_1(true);
  delay(2000);

  //sensor_ctrl.next_radiate = millis() + 20000;
  Serial.println(F("Sensor Setup"));
  // for(uint8_t sindx = 0; sindx < NBR_OF_SENSORS; sindx++)
  // {
  //     if(sensor[sindx].sensor_type != SENSOR_UNDEFINED)
  //     {
  //         for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
  //         {
  //           Serial.print(sensor[sindx].zone);
  //           Serial.print(F(" "));
  //           Serial.println(sensor[sindx].values[vindx].label);
  //         }
  //     }


  Serial.println();

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

  #ifdef MCP9808_ADDR
    if (!mcp9808_sensor.begin(MCP9808_ADDR)) {
      Serial.println(F("Couldn't find MCP9808! Check your connections and verify the address is correct."));
      while (1);
    }     
    Serial.println(F("Found MCP9808!"));
    mcp9808_sensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
    // Mode Resolution SampleTime
    //  0    0.5°C       30 ms
    //  1    0.25°C      65 ms
    //  2    0.125°C     130 ms
    //  3    0.0625°C    250 ms

  #endif

  #ifdef VEML7700_ADDR
    if (!veml.begin()) {
      Serial.println(F("Sensor not found"));
      while (1);
    }
    Serial.println(F("Sensor found"));

    // == OPTIONAL =====
    // Can set non-default gain and integration time to
    // adjust for different lighting conditions.
    // =================
    // veml.setGain(VEML7700_GAIN_1_8);
    // veml.setIntegrationTime(VEML7700_IT_100MS);

   
    Serial.print(F("Gain: "));
    switch (veml.getGain()) {
      case VEML7700_GAIN_1: Serial.println("1"); break;
      case VEML7700_GAIN_2: Serial.println("2"); break;
      case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
      case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
    }

    Serial.print(F("Integration Time (ms): "));
    switch (veml.getIntegrationTime()) {
      case VEML7700_IT_25MS: Serial.println("25"); break;
      case VEML7700_IT_50MS: Serial.println("50"); break;
      case VEML7700_IT_100MS: Serial.println("100"); break;
      case VEML7700_IT_200MS: Serial.println("200"); break;
      case VEML7700_IT_400MS: Serial.println("400"); break;
      case VEML7700_IT_800MS: Serial.println("800"); break;
    }

    veml.setLowThreshold(10000);
    veml.setHighThreshold(20000);
    // veml.interruptEnable(true);

  #endif

  #ifdef LDR_PIN
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
  #endif

  sensor_ctrl.indx = 0;
  sensor_ctrl.read_task_indx = atask_add_new(&sensor_read_handle);
  sensor_ctrl.send_task_indx = atask_add_new(&sensor_send_handle);
}

#ifdef USE_BME680
bool sensor_read_bme680(uint8_t indx)
{
    bool bme680_ok = false;
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
        Serial.printlnF(("Failed to perform reading :("));
      }  
    return bme680_ok;
}
#endif

#ifdef USE_SCD30
bool sensor_read_scd30(uint8_t indx)
{
    bool scd30_ok = false;;
      if (scd30.dataReady())
      {
        Serial.println(F("Data available!"));

        if (!scd30.read())
        { 
          Serial.println(F("Error reading sensor data"));
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
    return scd30_ok;
}
#endif  

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

#ifdef LDR_PIN
bool sensor_read_ldr(uint8_t sindx)
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
  return true;
}
#endif


#ifdef MCP9808_ADDR
void sensor_read_mcp9808(uint8_t sindx)
{
    for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
    {
        switch(sensor[sindx].values[vindx].type)
        {
            case SENSOR_VALUE_TEMPERATURE:
                sensor[sindx].values[vindx].value = mcp9808_sensor.readTempC();
                sensor[sindx].values[vindx].updated = true;
                break;
        }    
    }
    return true;
}
#endif

#ifdef VEML7700_ADDR
void sensor_read_veml7700(uint8_t sindx)
{
    for(uint8_t vindx= 0; vindx < SENSOR_MAX_VALUES; vindx++)
    {
        switch(sensor[sindx].values[vindx].type)
        {
            case SENSOR_VALUE_LUX:
                sensor[sindx].values[vindx].value = veml.readLux();
                sensor[sindx].values[vindx].updated = true;
                break;
            case SENSOR_VALUE_ALS:
                sensor[sindx].values[vindx].value = veml.readALS();
                sensor[sindx].values[vindx].updated = true;
                break;
            case SENSOR_VALUE_WHITE:
                sensor[sindx].values[vindx].value = veml.readWhite();
                sensor[sindx].values[vindx].updated = true;
                break;
        }    
    }
    uint16_t irq = veml.interruptStatus();
    if (irq & VEML7700_INTERRUPT_LOW) Serial.println(F("** Low threshold"));
    if (irq & VEML7700_INTERRUPT_HIGH) Serial.println(F("** High threshold"));
    return true;
}
#endif


void sensor_read_all(void)
{
    for(uint8_t sindx = 0; sindx < NBR_OF_SENSORS; sindx++)
    {
        // Serial.print(F("Read Sensor: ")); Serial.print(sindx);
        // Serial.print(F(" Type: ")); Serial.println(sensor[sindx].sensor_type);
        switch(sensor[sindx].sensor_type)
        {
            #ifdef USE_BME680
            case SENSOR_BME680:
                sensor_read_bme680(sindx);
                break;
            #endif

            #ifdef LDR_PIN
            case SENSOR_LDR:
                sensor_read_ldr(sindx);
                break;
            #endif

            #ifdef USE_SCD30
            case SENSOR_SCD30:
                sensor_read_scd30(sindx);
                break;
            #endif

            #ifdef USE_PCT2075
            case SENSOR_PCT2075:
                sensor_read_pct2075(sindx);
                break;
            #endif  

            #ifdef MCP9808_ADDR  
            case SENSOR_MCP9808:
                sensor_read_mcp9808(sindx);
                break;
            #endif    

            #ifdef VEML7700_ADDR
            case SENSOR_VEML7700:
                sensor_read_veml7700(sindx);
                break;
            #endif
        }
    }
}

void sensor_print_value(uint8_t sindx, uint8_t vindx)
{

    // if((sensor[sindx].sensor_type != SENSOR_VALUE_UNDEFINED)  && 
    //   (sensor[sindx].values[vindx].type != SENSOR_VALUE_UNDEFINED) && 
    //   (millis() > sensor_ctrl.next_radiate)) 
    // {
    //     if (millis() > sensor[sindx].values[vindx].next_ms)
    //     {
    //         sensor[sindx].values[vindx].next_ms = millis() + sensor[sindx].values[vindx].interval_ms;
    //         Serial.print(F("Sensor :")); Serial.print(sindx);
    //         Serial.print(F(" value# ")); Serial.print(vindx);
    //         Serial.print(F(" -> "));
    //         Serial.print(sensor[sindx].zone); Serial.print(" ");
    //         Serial.print(sensor[sindx].values[vindx].label); Serial.print(" ");
    //         Serial.print(sensor[sindx].values[vindx].value);
    //         Serial.println();


    //         json_convert_sensor_float_to_json(
    //             sensor_ctrl.radio_msg, 
    //             sensor[sindx].zone, 
    //             sensor[sindx].values[vindx].label, 
    //             sensor[sindx].values[vindx].value, 
    //             "-" );


    //         rfm_send_radiate_msg(sensor_ctrl.radio_msg);
    //         sensor_ctrl.next_radiate = millis() + 2000;

    //     }
    // }
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

void sensor_read_task(void)
{
  /*
    static uint8_t  sensor_index = 0;
    static uint8_t  value_index = 0;
    bool all_done;

    watchdog_clear_local();
    switch(sensor_read_handle.state)
    {
        case 0:
            sensor_read_handle.state = 10;
            io_pwr_on_1(false);
            atask_delay(sensor_ctrl.read_task_indx,4000);
            break;
        case 10:
            io_pwr_on_1(true);
            atask_delay(sensor_ctrl.read_task_indx,4000);
            sensor_read_handle.state = 15;
            break;      
        case 15: 
            sensor_read_all();
            sensor_read_handle.state = 20;
            io_pwr_on_1(false);
            atask_delay(sensor_ctrl.read_task_indx,15000);
          break;
        case 20:
            sensor_read_handle.state = 10;
            break;
      }
*/
}

void sensor_send_task(void)
{
    static uint8_t sensor_index = 0;
    static uint8_t value_index = 0;
    
    // Serial.print(F("Send Sensor# ")); Serial.print(sensor_index);
    // Serial.print(F(" Value#: ")); Serial.println(value_index);
    switch(sensor_send_handle.state)
    {
        case 0:
          sensor_read_all();
          sensor_send_handle.state = 10;
          break;
        case 10:
          if((sensor[sensor_index].values[value_index].type != SENSOR_VALUE_UNDEFINED) && 
            (sensor[sensor_index].values[value_index].updated) &&
            millis() > (sensor[sensor_index].values[value_index].next_ms))
          {
            if (++sensor_index >= NBR_OF_SENSORS)
            {
                sensor_index = 0;
                // next_send_ms = millis() + 10000;
                sensor_send_handle.state = 10;
                xi2c_set_sleep_time(10000);
            }
            value_index = 0;

            json_convert_sensor_float_to_json(
                sensor_ctrl.radio_msg, 
                sensor[sensor_index].zone, 
                sensor[sensor_index].values[value_index].label, 
                sensor[sensor_index].values[value_index].value, 
                "-" );
            rfm_send_radiate_msg(sensor_ctrl.radio_msg);
            sensor[sensor_index].values[value_index].updated = false;        
            sensor[sensor_index].values[value_index].next_ms = millis() +
              sensor[sensor_index].values[value_index].interval_ms;
            atask_delay(sensor_ctrl.send_task_indx,5000);
          }
          sensor_send_handle.state = 20;
          break;
        case 20:
          if(++value_index >= SENSOR_MAX_VALUES )
          {
            value_index = 0;
            if(++sensor_index >= NBR_OF_SENSORS) sensor_index = 0;
          }
          sensor_send_handle.state = 10;
          break;
    }
}
