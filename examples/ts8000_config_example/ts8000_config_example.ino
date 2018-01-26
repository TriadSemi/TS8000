/*******************************************************************
    Copyright (C) 2018 Triad Semiconductor

    ts8000_config_example.ino - Example application for configuring the Triad
              Semiconductor TS8000 Ultrasonic to Digital converter.
              
    Created by: John Seibel
*******************************************************************/
#include <ts8000.h>

//IMPORTANT NOTE: If porting the TS8000 library code to a non-Arduino architecture,
//be sure that the INPUT ports assigned to the DATA and CLK signals are configured as
//floating inputs with NO pull-up or pull-down function.  Using a pull-up or
//pull-down function on the input will cause the TS8000 to operate incorrectly.

#define device1_CLK_pin   your_CLK_pin   //User must replace your_CLK_pin with their pin number (compile error will occur if no number defined)
#define device1_DATA_pin   your_DATA_pin   //User must replace your_DATA_pin with their pin number (compile error will occur if no number defined)

uint8_t config_result;

TS8000  device1(device1_CLK_pin, device1_DATA_pin);  //instantiate the class as device1 and assign pins

void setup() {
  Serial.begin(9600);
  while (!Serial);  //wait for serial port to connect

  Serial.println("Serial Port Connected");
  Serial.println();
  Serial.println();

  config_result = device1.configDevice();

  //user can determine how to handle each return value for the configuration function
  switch (config_result) {
    case CONFIG_PASS:
      Serial.println("Configuration SUCCESS");
      break;
    case BUS_FAIL:  //unable to resolve state of TS8000 (3 samples of the bus signals resulted in 3 different states)
      Serial.println("Configuration Unsuccessful - BUS_FAIL");
      break;
    case VERIFY_FAIL:  //configuration read value did not match configuration write value, run configuration again
      Serial.println("Configuration Unsuccessful - VERIFY_FAIL");
      break;
    case LISTEN_FAIL:  //verify succeeded but entry into LISTEN mode failed, run configuration again
      Serial.println("Configuration Unsuccessful - LISTEN_FAIL");
      break;
    default:  //value returned was unknown
      Serial.println("Program Execution ERROR");
      break;
    }
  }

void loop() {
  //insert your main code here
  }
