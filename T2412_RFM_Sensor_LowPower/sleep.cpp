#ifdef SLEEP_CTRL
      xi2c_initialize(SLEEP_CTRL_I2C_ADDR);
      delay(100);
      xi2c_set_wd_timeout(0x1234);
      //xi2c_set_sleep_time(5000);
      delay(100);
      //xi2c_set_sleep_time(0x1234);
      xi2c_set_wd_timeout(0x2345);
      delay(100);
      Wire.beginTransmission(SLEEP_CTRL_I2C_ADDR);
      int res = Wire.endTransmission();
      Serial.print(F("Wire Code=")); Serial.println(res);
#endif
    