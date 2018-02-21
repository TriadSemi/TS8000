/*******************************************************************
    Copyright (c) 2018 Triad Semiconductor
    ts8000_config_example.ino - Example application for calibrating
              and configuring the Triad Semiconductor TS8000 Ultrasonic
              to Digital converter.
    Created by: John Seibel
*******************************************************************/
#include <ts8000.h>

//IMPORTANT NOTES:
// 1) The TS8000 requires a timer counter to perform the calibration routine.  Timer counter
//    initialization is specific to a particular microcontroller, so this TS8000 Library
//    was developed with the SAMD21 (Arduino ZERO) as the target hardware.  Use of a
//    different microcontroller will require the user to modify the timer_init() and
//    TC3_Handler() functions found at the bottom of this example sketch for their
//    specific hardware.
// 2) The TS8000 configuration is volatile, so calibrateDevice() must be executed after
//    every power cycle
// 3) The INPUT port assigned to the DATA signal must be configured as a
//    floating input with NO pull-up or pull-down function.  Using a pull-up or
//    pull-down function on the input will cause the TS8000 to operate incorrectly.
// 4) The calibrateDevice() function uses a pin interrupt to capture a free-running
//    timer count value on each edge of a ~60us minimum wide pulse that the TS8000
//    generates during the calibration routine.  For the SAMD21 running at
//    48MHz, the code executed between interrupts executes fast enough that it is 
//    finished before the 2nd interrupt on the trailing edge of the pulse occurs.
//    If the SAMD21 microcontroller is not used, it is up to the user to verify
//    that their microcontroller is running fast enough to finish the code before
//    the 2nd interrupt occurs.
// 5) To simplify the TS8000 Library for use with an external pin interrupt, an additional
//    pin is used for the external interrupt input.  The DATA signal from the TS8000
//    should be connected to an Arduino external interrupt pin that will detect a rising
//    and falling edge on the DATA signal for the purposes of calibrating the TS8000 in
//    the calibrateDevice() function.  The diagram below illustrates the connections.
//
//    -------------           ------------
//    |  Arduino  |           |  TS8000  |
//    |           |           |          |
//    |       CLK |<--------->| CLK      |
//    |      DATA |<--------->| DATA     |
//    |           |     |     |          |
//    |       INT |<-----     |          |
//    |           |           |          |
//    -------------           ------------
//

#define IDEAL_CAL_COUNT_VAL  your_count_val    //User must replace your_count_val with their timer counter value that corresponds to the count value
                                               //at 75us - for a 48MHz counter clock, this value is 3600 decimal
#define device1_CLK_pin      your_CLK_pin   //User must replace your_CLK_pin with their pin number (compile error will occur if no number defined)
#define device1_DATA_pin     your_DATA_pin   //User must replace your_DATA_pin with their pin number (compile error will occur if no number defined)
#define device1_INT_pin      your_INT_pin   //User must replace your_INT_pin with their pin number (compile error will occur if no number defined)

uint16_t  user_config_word = 0x082C;  //default, uncalibrated configuration word is 0x082C, the user can modify this to include their
                                      //specific register settings in the upper 9 bits prior to executing calibrateDevice() - calibrateDevice()
                                      //does not modify the upper 9 bits, only the lower 7 bits for the FILTER_TRIM setting
uint16_t    calibrated_config_word;   //configuration word returned by calibrateDevice() after the FILTER_TRIM bits have been calibrated, the
                                      //upper 9 bits of this word are unmodified from the user_config_word that was passed to calibrateDevice()
char buf[5];

TS8000  device1(device1_CLK_pin, device1_DATA_pin, IDEAL_CAL_COUNT_VAL);  //instantiate the class as device1 and assign pins

void setup() {
  pinMode(device1_INT_pin, INPUT);
  Serial.begin(9600);
  while (!Serial);  //wait for serial port to connect

  Serial.println("Serial Port Connected");
  Serial.println();
  Serial.println();

  timer_init();  //configure TC3 as a free-running timer (specific to SAMD21 micro)

  Serial.println("Beginning Calibration and Configuration...");

  attachInterrupt(device1_INT_pin, TC3_Handler, CHANGE);  //enable interrupt on digital pin (formatted for Arduino ZERO, user may need to modify
                                                          //if using a different Arduino board)
  calibrated_config_word = device1.calibrateDevice(user_config_word);
  detachInterrupt(device1_INT_pin);  //disable interrupt on digital pin (formatted for Arduino ZERO, user may need to modify
                                     //if using a different Arduino board)

  if (calibrated_config_word == 0x0000) Serial.println("Calibration Unseccessful - Configuration Verify Failed");
  else {
    Serial.print("Calibration and Configuration Successful - Configuration Word = ");
    sprintf(buf, "0x%04X", calibrated_config_word);
    Serial.println(buf);
    }
  }
  
void loop() {
  //insert your main code here
  }

void timer_init(void) {
//This example sketch uses a free-running 48MHz timer counter to calculate the width of a calibration
//pulse that the TS8000 generates.  The timer_init() function initializes the timer hardware for
//the SAMD21 microcontorller (Arduino ZERO).  Other microcontrollers may require a different
//initialization routine.  Refer to the datasheet for your specific microcontroller to set up
//a free-running timer for use with the TS8000 Library.

  //enable the clock for TC3 
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID ( GCM_TCC2_TC3 ) ) ;
  while ( GCLK->STATUS.bit.SYNCBUSY);  //wait for sync to finish 

  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;  //disable TC
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync to finish 

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  //timer counter mode set to 16 bits
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync to finish 

  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;  //prescaler is 1 (48MHz)
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync to finish 
  
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;  //enable TC3
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync to finish 
}

void TC3_Handler()  //interrupt service routine for TC3
{
  calibration_count = TC3->COUNT16.COUNT.reg;  //calibration_count is a global variable used by the TS8000 Library, do not change
  calibration_count_done = true;  //set ISR "done" flag, calibration_count_done is a global variable used by the TS8000 Library, do not change
}

