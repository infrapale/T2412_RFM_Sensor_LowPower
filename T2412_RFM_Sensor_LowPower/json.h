#ifndef __JSON_H__
#define __JSON_H__


/// @brief Convert sensor (float) to radio message
/// @note
/// @param  buff  target buffer
/// @param  zone char array
/// @param  sensor
/// @param  value float
/// @param  remark
/// @return
uint16_t json_convert_sensor_float_to_json(
    char *buff, 
    const char *zone, 
    const char *sensor, 
    float value, 
    const char *remark );




#endif