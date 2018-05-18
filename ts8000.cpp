/*******************************************************************
    Copyright (c) 2018 Triad Semiconductor
    ts8000.cpp - Library for calibrating and configuring the Triad
                 Semiconductor TS8000 Ultrasonic to Digital converter.

    Created by: John Seibel
*******************************************************************/

#include "ts8000.h"
#include <Arduino.h>

volatile uint16_t calibration_count;  //global variable used to pass the timer count from the ISR
volatile uint8_t TCC1_int_count;  //global variable used to count ISR executions
uint16_t  mask;
uint16_t  offset1;
uint16_t  offset0;
uint16_t  config_val1;
uint16_t  config_val0;


TS8000::TS8000(int device_CLK_pin, int device_DATA_pin, uint16_t ideal_timer_cal_value) {
  CLK_pin = device_CLK_pin;
  DATA_pin = device_DATA_pin;
  timer_cal_value = ideal_timer_cal_value;
  ts_digitalWrite(CLK_pin, LOW);
  ts_pinMode(CLK_pin, OUTPUT);
  ts_pinMode(DATA_pin, INPUT);
  }

void TS8000::ts_delayUs(unsigned int delay_val) {
  delayMicroseconds(delay_val);
  }

void TS8000::ts_pinMode(int pin, uint8_t mode) {
  pinMode(pin, mode);
  }

uint8_t TS8000::ts_digitalRead(int pin) {
  uint8_t read_val;
  
  read_val = digitalRead(pin);
  return read_val;
  }

void TS8000::ts_digitalWrite(int pin, uint8_t write_val) {
  digitalWrite(pin, write_val);
  }

uint16_t TS8000::calibrateDevice(uint16_t config_val) {
  uint16_t cal_return = 0x0000;
  uint16_t readback;

  ts_digitalWrite(CLK_pin, HIGH);
  ts_digitalWrite(DATA_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(DATA_pin, OUTPUT);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(DATA_pin, INPUT);
  mask = 0x0040;
  config_val = (config_val & 0xFF80) | mask;  //just operate on the FILTER_TRIM bits
  for (uint8_t i = 7; i > 0; i--) {
    writeConfig(config_val);  //start with FILTER_TRIM = 0x40
    readback = readConfig(CAL);
    while(TCC1_int_count < 2);  //wait until TCC0 ISR has executed 2 times, once for
                                //the rising edge of the calibration pulse and once
                                //on the falling edge of the calibration pulse

    //During the last 2 iterations of the loop, the difference values (offset)
    //and config_val values corresponding to those offsets are saved in order
    //to determine which has the smaller offset to be used as the FILTER_TRIM
    //calibratin value.
    if (calibration_count > timer_cal_value) {
      if (i == 2) {
        offset1 = calibration_count - timer_cal_value;  //store offset when FILTER_TRIM last bit is 0
        config_val1 = config_val;  //store config_val associated with offset
      }
      if (i == 1) {
        offset0 = calibration_count - timer_cal_value;  //store offset when FILTER_TRIM last bit is 1
        config_val0 = config_val;  //store config_val associated with offset
      }
      config_val = config_val ^ mask;  //flip bit in mask position on FILTER_TRIM
    }
    else {
      if (i == 2) {
        offset1 = timer_cal_value - calibration_count;  //store offset when FILTER_TRIM last bit is 0
        config_val1 = config_val;  //store config_val associated with offset
      }
      if (i == 1) {
        offset0 = timer_cal_value - calibration_count;  //store offset when FILTER_TRIM last bit is 1
        config_val0 = config_val;  //store config_val associated with offset
        }
      }
    mask = mask >> 1;  //shift right to operate on next bit in FILTER_TRIM
    config_val = config_val | mask;  //set next bit down
    }
  if (offset1 < offset0) config_val = config_val1;  //offset1 was smaller, so use that config_val
  else config_val = config_val0;  //offset0 was smaller, so use that config_val
  writeConfig(config_val);  //write final configuration word after calibration
  readback = readConfig(NO_CAL);
  if (readback == config_val) {  //verify configuration
    cal_return = config_val;
    }
  else cal_return = 0x0000;
  ts_delayUs(CONFIG_RECOVERY);  //delay to allow post-configuration chatter to cease
  return cal_return;
  }

void TS8000::writeConfig(uint16_t config_val) {
  ts_digitalWrite(CLK_pin, HIGH);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_pinMode(DATA_pin, OUTPUT);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  for (uint8_t i = 0; i < 16; i++) {
    if ((config_val & 0x8000) > 0) ts_digitalWrite(DATA_pin, HIGH);
    else ts_digitalWrite(DATA_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(CLK_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(CLK_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    config_val = config_val << 1;
  }
  ts_digitalWrite(DATA_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(DATA_pin, INPUT);
  }

uint16_t TS8000::readConfig(bool cal) {
  uint16_t readback;
  
  readback = 0x0000;
  ts_digitalWrite(CLK_pin, HIGH);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_pinMode(DATA_pin, OUTPUT);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(DATA_pin, INPUT);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, LOW);
  ts_delayUs(BUS_DRV_DLY);
  for (uint8_t i = 0; i < 16; i++) {
    ts_digitalWrite(CLK_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    readback = (readback << 1) | (ts_digitalRead(DATA_pin) & 0x0001);
    ts_digitalWrite(CLK_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    }
    if (cal == NO_CAL) {
      ts_digitalWrite(DATA_pin, LOW);
      ts_pinMode(DATA_pin, OUTPUT);
      ts_delayUs(BUS_DRV_DLY);
      ts_digitalWrite(CLK_pin, HIGH);
      ts_delayUs(BUS_DRV_DLY);
      ts_digitalWrite(DATA_pin, HIGH);
      ts_delayUs(BUS_DRV_DLY);
      ts_pinMode(DATA_pin, INPUT);
      ts_delayUs(BUS_DRV_DLY);
      ts_digitalWrite(CLK_pin, LOW);
      ts_delayUs(BUS_DRV_DLY);
      }
    else {
      TCC1_int_count = 0;  //clear ISR count
      ts_digitalWrite(CLK_pin, HIGH);
      ts_delayUs(BUS_DRV_DLY);
      ts_digitalWrite(CLK_pin, LOW);
      }
  return readback;
  }
