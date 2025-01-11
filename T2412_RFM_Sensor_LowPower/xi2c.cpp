/******************************************************************************
  EDOOG  IS CURRENTLY NOT IN USE,  HW FIX is pending
*******************************************************************************
https://inductive-kickback.com/2019/04/creating-an-i2c-slave-interface-for-a-sensor-or-peripheral/

https://deepbluembedded.com/arduino-i2c-tutorial-examples/
https://duino4projects.com/mastering-i2c-interfacing-arduino-boards-for-seamless-communication/
https://wiki-content.arduino.cc/en/Tutorial/LibraryExamples/MasterWriter
https://thewanderingengineer.com/2014/02/17/attiny-i2c-slave/
https://www.tastethecode.com/introducing-the-attiny-device-pcb-i2c-slave-devices
******************************************************************************/


#include "Arduino.h"
#include <Wire.h>
#include "xi2c.h"
//#include "helper.h"
//#include "io.h"

uint8_t tarr1[I2C_TX_BUFF_SIZE];
uint8_t tarr2[I2C_TX_BUFF_SIZE];

typedef struct 
{
    eeprom_index_et eeprom_addr_index;
} xi2c_ctrl_st;

i2c_st i2c;
xi2c_ctrl_st xi2c_ctrl;
char s[89];




void helper_print_hex_arr(uint8_t *arr, uint8_t n)
{
    for (uint8_t i = 0; i < n;i++)
    {
      sprintf(s,"%02X ", arr[i]);
      Serial.print(s);
    }
    Serial.println();
}

void xi2c_put_tx_buff_uint32( uint16_t offs, uint32_t u32)
{
    i2c.tx_buff[offs + 0 ] = (u32 >> 24) & 0x000000FF;
    i2c.tx_buff[offs + 1 ] = (u32 >> 16) & 0x000000FF;
    i2c.tx_buff[offs + 2 ] = (u32 >> 8) & 0x000000FF;
    i2c.tx_buff[offs + 3 ] = (u32 >> 0) & 0x000000FF;
}

void xi2c_put_tx_buff_uint16( uint16_t offs, uint16_t u16)
{
    i2c.tx_buff[offs + 0 ] = (u16 >> 8) & 0x000000FF;
    i2c.tx_buff[offs + 1 ] = (u16 >> 0) & 0x000000FF;
}
void xi2c_put_tx_buff_uint8( uint16_t offs, uint8_t u8)
{
    i2c.tx_buff[offs + 0 ] = (u8 >> 0) & 0x000000FF;
}

uint32_t xi2c_get_rx_buff_uint32(uint16_t offs)
{
    uint32_t u32 = 0;
    u32 |= ((uint32_t)i2c.rx_buff[offs + 0 ] << 24) & 0xFF000000UL;
    u32 |= ((uint32_t)i2c.rx_buff[offs + 1 ] << 16) & 0x00FF0000UL;
    u32 |= ((uint32_t)i2c.rx_buff[offs + 2 ] << 8)  & 0x00000FF00UL;
    u32 |= ((uint32_t)i2c.rx_buff[offs + 3 ] << 0)  & 0x0000000FFUL;
    return u32;
}

uint16_t xi2c_get_rx_buff_uint16(uint16_t offs)
{
    uint16_t u16 = 0;
    u16 |= ((uint16_t)i2c.rx_buff[offs + 0 ] << 8)  & 0x00000FF00UL;
    u16 |= ((uint16_t)i2c.rx_buff[offs + 1 ] << 0)  & 0x0000000FFUL;
    return u16;
}

uint8_t xi2c_get_rx_buff_uint8(uint16_t offs)
{
    uint8_t u8 = 0;
    u8 = ((uint8_t)i2c.rx_buff[offs + 0 ] << 0)  & 0x0000000FFUL;
    return u8;
}


void xi2c_build_test_data(uint8_t base_value)
{
  for( uint8_t i = 0; i < 8; i++)
  {
    tarr1[i] = (base_value + i) ;
    tarr2[i] = base_value + i*4;
  }
}

  

void xi2c_initialize(uint8_t i2c_addr)
{
  Serial.println("xi2c_initialize()");
  sprintf(s,"xi2c_initialize(0x%2X)\n",i2c_addr);
  Serial.print(s);
  i2c.addr = i2c_addr;
  xi2c_ctrl.eeprom_addr_index = EEPROM_MAIN_DATA;
  if (false)
  {
      Serial.println("Watchdog is enabled");
      xi2c_set_wd_timeout(10000);
      uint32_t u32 = xi2c_get_wd_timeout();
      sprintf(s,"edog WD timout = %d\n",u32);
      Serial.print(s);

  }
  else
  {
      Serial.println("Watchdog is disabled");
  }
  //xi2c_build_test_data(0x20);
  //xi2c_test_eeprom_write_read();
  //xi2c_set_wd_timeout(5000);
  //delay(10);
  //xi2c_set_sleep_time(2000);
  //xi2c_rd_reg(REG_ADDR_SLEEP_TIME,4);
  //xi2c_read_i2c(4);

}

void clear_local_watchdog(void)
{
    #ifdef PRO_MINI_RFM69
    watchdog.clear();
    #endif
    #ifdef  ADA_M0_RFM69
    
    #endif
      
}


void xi2c_print_rx_buff(void)
{
  Serial.print("i2c.rx: ");
  helper_print_hex_arr(i2c.rx_buff,9);
}

void xi2c_print_tx_buff(void)
{
  Serial.print("i2c.tx: ");
  helper_print_hex_arr(i2c.tx_buff,9);
}

void xi2c_receive_i2c(void)
{
  if(i2c.reg_m2s > 0)
  {
    Wire.beginTransmission(i2c.addr); 
    Wire.write( i2c.tx_buff, i2c.reg_m2s + 1)  ;      
    Wire.endTransmission();
  }  
}

void xi2c_read_i2c(uint8_t bytes)
{
    Wire.requestFrom(i2c.addr, bytes);   
    uint8_t i = 0; 
    while(Wire.available())    
    { 
      int c = Wire.read();
      i2c.rx_buff[i++] = (uint8_t) c;
    }
}

void xi2c_send_i2c(void)
{
  {
    Wire.beginTransmission(i2c.addr); 
    Wire.write( i2c.tx_buff, i2c.reg_m2s + 1)  ;      
    Wire.endTransmission();
  }  
}

// void xi2c_set_read_pos(uint8_t pos)
// {
//   Serial.println("Set read pos");
//   xi2c_build_uint_msg(REG_ADDR_SET_RD_POS, pos, 1, 0);
//   xi2c_send_i2c();
// }

// void xi2c_rd_reg(uint8_t pos, uint8_t len)
// {
//   xi2c_set_read_pos(pos);
//   delay(1);
//   Wire.requestFrom(i2c.addr, len); 
//   Wire.requestFrom(i2c.addr, i2c.reg_s2m);   
//   uint8_t i = 0; 
//   while(Wire.available())    
//   { 
//     int c = Wire.read();
//     i2c.rx_buff[i++] = (uint8_t) c;
//   }
//   xi2c_print_rx_buff();
// }
 

void xi2c_send_receive(void)
{

  Serial.print("send_receive: "); 
  // Serial.print(i2c.addr, HEX); 
  Serial.print(" reg_m2s="); Serial.print(i2c.reg_m2s);
  Serial.print(" reg_s2m="); Serial.print(i2c.reg_s2m);
  Serial.println(" ");
  Serial.flush();

  Wire.beginTransmission(i2c.addr); 
  Wire.write( i2c.tx_buff, i2c.reg_m2s + 1)  ;      
  Wire.endTransmission();

  if(i2c.reg_s2m > 0)
  {
    // delay(1);
    //Wire.beginTransmission(i2c.addr); 
    Wire.requestFrom(i2c.addr, i2c.reg_s2m);   
    uint8_t i = 0; 
    while(Wire.available())    
    { 
      int c = Wire.read();
      i2c.rx_buff[i++] = (uint8_t) c;
    }

    //Wire.endTransmission();
  }
}

void xi2c_build_uint_msg(uint8_t raddr, uint32_t value, uint8_t m2s, uint8_t s2m)
{
  uint8_t aindx = 0;    //[0] is reserved for register address
  i2c.reg_addr = raddr;
  i2c.reg_data = value;
  i2c.reg_m2s = m2s;
  i2c.reg_s2m = s2m;

  i2c.tx_buff[aindx++] = i2c.reg_addr;
  if (i2c.reg_m2s == 4)
  {
    i2c.tx_buff[aindx++] = (i2c.reg_data >> 24) & 0x000000FF;
    i2c.tx_buff[aindx++] = (i2c.reg_data >> 16) & 0x000000FF;
  }
  if (i2c.reg_m2s > 1)
  {
    i2c.tx_buff[aindx++] = (i2c.reg_data >> 8) & 0x000000FF;
  }
  i2c.tx_buff[aindx] = (i2c.reg_data >> 0) & 0x000000FF;
}

void xi2c_build_array_msg(uint8_t raddr, uint8_t *arr, uint8_t m2s, uint8_t s2m)
{
  uint8_t aindx = 0;    //[0] is reserved for register address
  i2c.reg_addr = raddr;
  i2c.reg_m2s = m2s;
  i2c.reg_s2m = s2m;
  // sprintf(s,"i2c.reg_addr %02X %02X\n", i2c.reg_addr, i2c.tx_buff[0]); Serial.print(s); Serial.flush(); 
  i2c.tx_buff[aindx++] = i2c.reg_addr;
  for (uint8_t i = 0; i < m2s; i++)
  {
    i2c.tx_buff[aindx++] = arr[i];
  }
}


void xi2c_set_wd_timeout(uint32_t wd_timeout)
{
  sprintf(s,"Watchdog timeout = %d\n\r", wd_timeout);
  Serial.print(s);
  xi2c_build_uint_msg(CMD_SET_WD_INTERVAL, wd_timeout, 4, 0);
  xi2c_send_receive();
}

uint32_t xi2c_get_wd_timeout(void)
{
  Serial.print("Read WD TIMEOUT\n");
  
  xi2c_build_uint_msg(CMD_GET_WD_INTERVAL, 0, 1, 4);
  xi2c_send_receive();  
  xi2c_print_rx_buff();
  return xi2c_get_rx_buff_uint32(0);
}


void xi2c_set_sleep_time(uint32_t sleep_time)
{
  sprintf(s,"Sleep time = %d\n\r",sleep_time);
  Serial.print(s);
  xi2c_build_uint_msg(CMD_SET_SLEEP_TIME, sleep_time, 4, 0);
  xi2c_send_receive();
}

void xi2c_clear_watchdog(void)
{
  // Serial.println("Clear watchdogSleep time = ");
  xi2c_build_uint_msg(CMD_CLEAR_WATCHDOG, 1, 1, 0);
  xi2c_send_receive();
}

void xi2c_switch_off(void)
{
  Serial.println("Goto Sleep");
  xi2c_build_uint_msg(CMD_SWITCH_OFF, 1, 1, 0);
  xi2c_send_receive();
}

// void xi2c_switch_off_1(uint8_t value)
// {
//   Serial.println("Switch off 1");
//   xi2c_build_uint_msg(REG_ADDR_SWITCH_OFF_1, value, 1, 0);
//   xi2c_send_receive();
// }

void xi2c_load_eeprom(void)
{
  Serial.println("Load EEPROM Data");
  xi2c_build_uint_msg(CMD_EEPROM_LOAD, 1, 1, 0);
  xi2c_send_i2c();
}

void xi2c_save_eeprom(void)
{
  Serial.println("Save EEPROM Data");
  xi2c_build_uint_msg(CMD_EEPROM_SAVE, 1, 1, 0);
  xi2c_send_i2c();
}

void xi2c_select_eeprom_index(eeprom_index_et eeprom_addr_index)
{
  xi2c_ctrl.eeprom_addr_index = eeprom_addr_index;
  sprintf(s, "Select EEPROM index %d\n",xi2c_ctrl.eeprom_addr_index);
  Serial.print(s);
  xi2c_build_uint_msg(CMD_SET_EEPROM_INDEX, (uint8_t)xi2c_ctrl.eeprom_addr_index, 1, 0);
  xi2c_send_i2c();
}


void xi2c_read_eeprom(eeprom_index_et eeprom_addr_index)
{
  sprintf(s,"Read EEPROM @ %d\n", eeprom_addr_index);
  Serial.print(s);

  xi2c_select_eeprom_index(eeprom_addr_index);

  xi2c_build_uint_msg(CMD_EEPROM_LOAD, 0, 0, 0);
  xi2c_send_i2c();
  
  delay(20);
  xi2c_build_uint_msg(CMD_EEPROM_READ, 1, 0, 8);

  xi2c_send_receive();  
  //xi2c_print_rx_buff();
}

void xi2c_write_eeprom(eeprom_index_et eeprom_addr_index, uint8_t *arr)
{
  Serial.print("Write EEPROM \n");
  xi2c_select_eeprom_index(eeprom_addr_index);
 
  delay(5);
  xi2c_build_array_msg(CMD_EEPROM_WRITE, arr, 8, 0);
  xi2c_send_i2c();
  delay(10);

  // xi2c_build_uint_msg(CMD_EEPROM_SAVE, 0, 0, 0);  obsolete!!
  // xi2c_send_i2c();
  // //xi2c_print_tx_buff();
}

// void xi2c_write_eeprom_buff(uint16_t addr)
// {
//   Serial.printf("Write EEPROM @ %04X", addr);
 
//   xi2c_build_uint_msg(REG_ADDR_EEPROM_ADDR, (uint32_t) addr, 2, 0);
//   xi2c_send_i2c();
  
//   delay(1);
//   i2c.reg_addr = REG_ADDR_EEPROM_WRITE;
//   i2c.reg_m2s = 9;
//   i2c.reg_s2m = 0;
//   i2c.tx_buff[0] = i2c.reg_addr;
//   xi2c_send_i2c();
//   delay(1);

//   xi2c_build_uint_msg(REG_ADDR_EEPROM_SAVE, 0, 1, 0);
//   xi2c_send_i2c();
//   delay(10);
//   xi2c_print_tx_buff();
// }


void xi2c_test_eeprom_write_read(void)
{
  eeprom_index_et eeprom_addr_index;

  Serial.print("xi2c_test_eeprom_write_read ------\n");

  // xi2c_set_wd_timeout(0x2345);
  // for( uint32_t i=0x10; i< 0x100; i+=0x10)
  // {
  //     uint32_t ival =  xi2c_get_wd_timeout();
  //     Serial.printf("read ival = %X\n", ival);
  //     Serial.printf("add %X\n",i);
  //     xi2c_set_wd_timeout(ival +i);
  // }

  // while(1);    

  for (uint8_t iter=0; iter<4; iter++)
  {
    sprintf(s,"xi2c_test_eeprom_write_read test iteration %d\n", iter);
    Serial.print(s);

    eeprom_addr_index = EEPROM_USER_0;
    sprintf(s,"> read index= %d\n", eeprom_addr_index);
    Serial.print(s);
    xi2c_read_eeprom(eeprom_addr_index);
    xi2c_print_rx_buff();

    eeprom_addr_index = EEPROM_USER_1;
    sprintf(s,"> read index= %d\n", eeprom_addr_index);
    Serial.print(s);
    xi2c_read_eeprom(eeprom_addr_index);
    xi2c_print_rx_buff();

    eeprom_addr_index = EEPROM_USER_0;
    sprintf(s,"> write index= %d\n", eeprom_addr_index);
    Serial.print(s);
    xi2c_build_test_data(0x40+iter);
    xi2c_write_eeprom(eeprom_addr_index, tarr1);
    xi2c_print_tx_buff();

    eeprom_addr_index = EEPROM_USER_1;
    sprintf(s,"> write index= %d\n", eeprom_addr_index);
    Serial.print(s);
    xi2c_build_test_data(0x50+iter);
    xi2c_write_eeprom(eeprom_addr_index, tarr1);
    xi2c_print_tx_buff();

    delay(100);
  }
}

