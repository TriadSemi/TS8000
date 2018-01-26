/*******************************************************************
    Copyright (C) 2018 Triad Semiconductor

    ts8000.h - Library for configuring the Triad Semiconductor TS8000 Ultrasonic
               to Digital converter.

    Created by: John Seibel
*******************************************************************/

#ifndef ts8000_h
#define ts8000_h

#include <stdint.h>

#define BUS_DRV_DLY     1       //delay in microseconds between bus level changes
#define BUS_CHECK_DLY   2000    //delay in microseconds for the checkBus() function
                                //should be longer than an ultrasonic pulse detection envelope
#define CONFIG_RECOVERY 100     //delay in microseconds after exiting configuration mode
#define UNKNOWN_STATE   0x04    //checkBus() function error code
#define S3_STATE        0x03    //checkBus() function state
#define S2_STATE        0x02    //checkBus() function state
#define S1_STATE        0x01    //checkBus() function state
#define LISTEN_STATE    0x00    //checkBus() function state
#define CFG_WORD        0x082C  //default configuration value (power up value)
#define BUS_FAIL        0x01    //configDevice() function status return value
#define VERIFY_FAIL     0x02    //configDevice() function status return value
#define LISTEN_FAIL     0x03    //configDevice() function status return value
#define CONFIG_PASS     0x04    //configDevice() function status return value

class TS8000 {

  public:
    TS8000(int device_CLK_pin, int device_DATA_pin);
    uint8_t configDevice(uint16_t config_val = CFG_WORD);

  private:
    bool goToListen(void);
    uint8_t checkBus(void);
    void ts_delayUs(unsigned int delay_val);
    void ts_pinMode(int pin, uint8_t mode);
    uint8_t ts_digitalRead(int pin);
    void ts_digitalWrite(int pin, uint8_t write_val);
    void writeConfig(uint16_t config_val);
    uint16_t readConfig(void);
    int CLK_pin;
    int DATA_pin;
    bool configured;
};    

#endif
