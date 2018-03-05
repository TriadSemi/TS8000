/*******************************************************************
    Copyright (c) 2018 Triad Semiconductor
    ts8000.h - Library for calibrating and configuring the Triad
               Semiconductor TS8000 Ultrasonic to Digital converter.

    Created by: John Seibel
*******************************************************************/

#ifndef ts8000_h
#define ts8000_h

#include <stdint.h>

#define BUS_DRV_DLY     1       //delay in microseconds between bus level changes
#define CAL             true    //perform calibration
#define NO_CAL          false   //do not perform calibration
#define CONFIG_RECOVERY 1500    //delay in microseconds after exiting configuration mode
#define CFG_WORD        0x082C  //default configuration value (power up value)

extern volatile uint16_t calibration_count;
extern volatile uint8_t TCC0_int_count;

class TS8000 {

  public:
    TS8000(int device_CLK_pin, int device_DATA_pin, uint16_t ideal_cal_count_val);
    void timer_init(void);
    uint16_t calibrateDevice(uint16_t config_val = CFG_WORD);

  private:
    void ts_delayUs(unsigned int delay_val);
    void ts_pinMode(int pin, uint8_t mode);
    uint8_t ts_digitalRead(int pin);
    void ts_digitalWrite(int pin, uint8_t write_val);
    void writeConfig(uint16_t config_val);
    uint16_t readConfig(bool cal);
    int CLK_pin;
    int DATA_pin;
    uint16_t timer_cal_value;
};    

#endif
