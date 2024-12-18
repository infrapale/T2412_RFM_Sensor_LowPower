#ifndef __SENSOR_H__
#define __SENSOR_H__

#define  SENSOR_SENSOR_LEN    10
#define  SENSOR_NAME_LEN    10

//#define SENSOR_UNDEFINED 
#define SENSOR_VA_TUPA
//#define SENSOR_VA_MH1
//#define SENSOR_VA_MH2
//#define SENSOR_VA_PARVI
//#define SENSOR_VA_KHH
//#define SENSOR_VA_OD
//#define SENSOR_LA_H
//#define SENSOR_LA_STUDIO
//#define SENSOR_DOCK

typedef enum
{
  SENSOR_BME680_TEMP = 0,
  SENSOR_BME680_HUM,
  SENSOR_BME680 = 0,
  SENSOR_CO2,
  SENSOR_NBR_OF
} sensor_et;

typedef struct
{
  char      zone[SENSOR_SENSOR_LEN];
  char      label[SENSOR_NAME_LEN];
  bool      active;
  float     value;
  uint32_t  interval;

} sensor_st;

#endif




