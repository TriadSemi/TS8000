/*******************************************************************
    Copyright (C) 2018 Triad Semiconductor

    ts8000.h - Library for configuring the Triad Semiconductor TS8000 Ultrasonic
               to Digital converter.

    Created by: John Seibel
*******************************************************************/

#include "ts8000.h"
#include <Arduino.h>


TS8000::TS8000(int device_CLK_pin, int device_DATA_pin) {
  configured = false;
  CLK_pin = device_CLK_pin;
  DATA_pin = device_DATA_pin;
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


uint8_t TS8000::configDevice(uint16_t config_val) {
  uint8_t config_success = 0x00;
  uint16_t readback;

  configured = false;
  ts_digitalWrite(DATA_pin, LOW);
  ts_digitalWrite(CLK_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(DATA_pin, OUTPUT);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(DATA_pin, INPUT);
  ts_pinMode(CLK_pin, INPUT);

  if (checkBus() == S3_STATE) {
    writeConfig(config_val);
    readback = readConfig();
    if (readback == config_val) {
      configured = true;
      if (goToListen()) config_success = CONFIG_PASS;
      else config_success = LISTEN_FAIL;
      }
    else config_success = VERIFY_FAIL;
    }
  else config_success = BUS_FAIL;
  
  return config_success;
  }

void TS8000::writeConfig(uint16_t config_val) {
  ts_digitalWrite(CLK_pin, HIGH);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_pinMode(CLK_pin, OUTPUT);
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
  ts_pinMode(CLK_pin, INPUT);
  ts_pinMode(DATA_pin, INPUT);
  }

uint16_t TS8000::readConfig(void) {
  uint16_t readback;
  
  readback = 0x0000;
  ts_digitalWrite(CLK_pin, HIGH);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_pinMode(CLK_pin, OUTPUT);
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
  ts_digitalWrite(DATA_pin, LOW);
  ts_pinMode(DATA_pin, OUTPUT);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(CLK_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_digitalWrite(DATA_pin, HIGH);
  ts_delayUs(BUS_DRV_DLY);
  ts_pinMode(CLK_pin, INPUT);
  ts_pinMode(DATA_pin, INPUT);
  return readback;
  }

//checkBus() performs a voting function where the bus is sampled 3 times
//to find 2 identical results.  This is necessary since ultrasonic detection is
//asynchronous and can indicate a false state.
uint8_t TS8000::checkBus(void) {
  uint8_t state;
  uint8_t CLK_state;
  uint8_t DATA_state;
  uint8_t S0_count = 0;
  uint8_t S1_count = 0;
  uint8_t S2_count = 0;
  uint8_t S3_count = 0;

  for (uint8_t i=0; i<3; i++) {
    CLK_state = ts_digitalRead(CLK_pin);
    DATA_state = ts_digitalRead(DATA_pin);
    if (DATA_state == HIGH) {
      if (CLK_state == HIGH) S3_count++;
      else S1_count++;
      }
    else {
      if (CLK_state == HIGH) S2_count++;
      else S0_count++;
      }
    ts_delayUs(BUS_CHECK_DLY);
    }
  if (S1_count >= 2) state = S1_STATE;
  else if (S2_count >= 2) state = S2_STATE;
  else if (S3_count >= 2) state = S3_STATE;
  else if (S0_count >= 2) state = LISTEN_STATE;
  else state = UNKNOWN_STATE;
  return state;
  }

bool TS8000::goToListen(void) {
  bool listen_success;
  
  if (configured == false)  listen_success = false;
  else {
    switch (checkBus()) {
      case LISTEN_STATE:
        listen_success = true;
        break;
      case S1_STATE:
        listen_success = false;
        break;
      case S2_STATE:
        listen_success = false;
        break;
      case S3_STATE:
        ts_digitalWrite(CLK_pin, LOW);
        ts_pinMode(CLK_pin, OUTPUT);
        ts_delayUs(BUS_DRV_DLY);
        ts_pinMode(CLK_pin, INPUT);
        ts_delayUs(BUS_DRV_DLY);
      if (checkBus() == LISTEN_STATE) listen_success = true;
        else listen_success = false;
        break;
      default:
        listen_success = false;
        break;
      }
    }
  return listen_success;
  }