# T2412_RFM_Sensor_LowPower
This system contains the main HW components:
- Arduino Pro Mini 3.3V  reading sensors and sending values when powerd on
- RFM69 Transmit data over 434MHz
- ATTiny412 powering the system via load switches
- BME680 temperature etc sensor

Pro mini is acting as a I2C mater and the ATTiny412 is acting as a I2C slave
