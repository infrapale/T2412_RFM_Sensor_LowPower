#ifndef __SENSOR_H__
#define __SENSOR_H__

#define  SENSOR_ZONE_LEN    10
#define  SENSOR_NAME_LEN    10

typedef enum
{
  ZONE_UNDEFINED = 0, 
  ZONE_VA_TUPA,
  ZONE_LA_H,
  ZONE_LA_STUDIO,
  ZONE_DOCK
} zone_indx_et;

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
  char  zone[SENSOR_ZONE_LEN];
  char  label[SENSOR_NAME_LEN];
  bool  active;
  float value;
  uint32 interval;

} sensor_st;

#endif

sensor_st sensor[SENSOR_NBR_OF] =
{
  //                  1234567890  1234567890
  [SENSOR_BME680] = {"Dock     ", "T_BME680"},

}
