/*******************************************************************
    Copyright (c) 2018 Triad Semiconductor
    ts8000_config_example.ino - Example application for calibrating
              and configuring the Triad Semiconductor TS8000 Ultrasonic
              to Digital converter.
    Created by: John Seibel
*******************************************************************/
#include <ts8000.h>
//#include <stdio.h>

//IMPORTANT NOTES:
// 1) The TS8000 requires timer/capture hardware to perform the calibration routine.  The
//    initialization is specific to a particular microcontroller, so this TS8000 Library
//    was developed with the SAMD21 (Arduino ZERO) as the target hardware.  Use of a
//    different microcontroller will require the user to modify the timer_init() and
//    TCC0_Handler() functions found at the bottom of this example sketch for their
//    specific hardware.
// 2) The TS8000 configuration is volatile, so calibrateDevice() must be executed after
//    every power cycle
// 3) The INPUT port assigned to the DATA signal must be configured as a
//    floating input with NO pull-up or pull-down function.  Using a pull-up or
//    pull-down function on the input will cause the TS8000 to operate incorrectly.
// 4) The calibrateDevice() function uses a pin interrupt signal as input to the
//    TCC0 timer/capture peripheral.  TCC0 is set up to measure the time between
//    interrupt events to measure an external ~60us minimum wide pulse that the TS8000
//    generates during the calibration routine.
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

#define device1_CLK_pin      your_CLK_pin   //User must replace your_CLK_pin with their pin number (compile error will occur if no number defined)
#define device1_DATA_pin     your_DATA_pin  //User must replace your_DATA_pin with their pin number (compile error will occur if no number defined)

//#define IDEAL_CAL_COUNT_VAL  your_count_val  //User must replace your_count_val with their count value that corresponds to the count value
                                               //at 75us - for a 48MHz counter clock, this value is 3600 decimal
#define IDEAL_CAL_COUNT_VAL  3600    //this value is assigned in the example sketch since the SAMD21 (Arduino ZERO) is the assumed target
                                     //and the timer_init() function configures the timer/capture hardware for 48MHz

//#define device1_INT_pin      your_INT_pin    //User must replace your_INT_pin with their pin number (compile error will occur if no number defined)
#define device1_INT_pin      12   //the timer_init() function assumes the use of pin 12 for the external pin interrupt (SAMD21 / Arduino ZERO)
                                  //so this value is assigned in the example sketch

uint16_t  user_config_word = 0x082C;  //default, uncalibrated configuration word is 0x082C, the user can modify this to include their
                                      //specific register settings in the upper 9 bits prior to executing calibrateDevice() - calibrateDevice()
                                      //does not modify the upper 9 bits, only the lower 7 bits for the FILTER_TRIM setting
uint16_t    calibrated_config_word;   //configuration word returned by calibrateDevice() after the FILTER_TRIM bits have been calibrated, the
                                      //upper 9 bits of this word are unmodified from the user_config_word that was passed to calibrateDevice()
char buf[7];  //character buffer which contains the HEX formatted configuration word string for display in the serial monitor window

TS8000  device1(device1_CLK_pin, device1_DATA_pin, IDEAL_CAL_COUNT_VAL);  //instantiate the class as device1 and assign pins

void setup() {
  pinMode(device1_INT_pin, INPUT);
  Serial.begin(9600);
  while (!Serial);  //wait for serial port to connect

  Serial.println("Serial Port Connected");
  Serial.println();
  Serial.println();

  timer_init();  //configure TCC0 to capture interrupt events from the pin interrupt (specific to SAMD21 micro)

  attachInterrupt(device1_INT_pin, NULL, CHANGE);  //enable interrupt on digital pin (formatted for Arduino ZERO, user may need to modify
                                                   //if using a different Arduino board)
  Serial.println("Beginning Calibration and Configuration...");
  calibrated_config_word = device1.calibrateDevice(user_config_word);  //performs configuration and calibration of the TS8000 device
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

//*****  The following functions require the use of timer/capture hardware and an interrupt, and are
//*****  specific to the SAMD21 (Arduino ZERO) microcontroller.  The code for these funcitons was put
//*****  in this example sketch for easy visibility by the user.  In order to make that work, 2 global
//*****  variables were needed to pass data between this example sketch and the TS8000 class.  They
//*****  are identified below in the code and must not be modified without appropriate modifications
//*****  to the TS8000 Library code.
//
//This example sketch uses the TCC0 timer/capture peripheral to calculate the width of a calibration
//pulse that the TS8000 generates.  The TCC0_Handler() and timer_init() functions are specific to
//the SAMD21 microcontroller (Arduino ZERO), so use of a different micro will require modification
//of the code in these functions.  Refer to the datasheet for your specific microcontroller to set up
//a similar capture function for use with the TS8000 Library.
void TCC0_Handler()
{
  calibration_count = REG_TCC0_CC1;  //read the time between pin interrupt events, which
                                     //correspond to the edges of the TS8000 calibration pulse,
                                     //calibration_count is global and used by the TS8000 class
                                     //so do not modify it
  TCC0_int_count++;  //count to indicate how many times the interrupt has occurred,
                     //TCC0_int_count is global and used by the TS8000 class
                     //so do not modify it
}

void timer_init(void) {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;//enable the event system peripheral
 
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN |
                                 GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID_TCC0_TCC1) ;  //enable clk for TCC0
  while (GCLK->STATUS.bit.SYNCBUSY); //wait for sync 
                          
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO3;  //enable event output on external interrupt 3 (Arduino ZERO pin 12, SAMD21)

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |  //attach the event user (receiver) to channel 0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1);  //set the event user (receiver) as timer TCC0, event 1

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |  //no event edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |  //event path is asynchronous (pass-thru)
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |  //set event generator (sender) as external interrupt 3 (Arduino ZERO pin 12, SAMD21)
                      EVSYS_CHANNEL_CHANNEL(0);  //attach the generator (sender) to channel 0

  REG_TCC0_EVCTRL |= TCC_EVCTRL_MCEI1 |           // Enable the match or capture channel 1 event input
                     TCC_EVCTRL_TCEI1 |           // Enable the TCC event 1 input
                     TCC_EVCTRL_EVACT1_PWP;       // Set up the timer for capture: CC1 period, CC0 pulsewidth

  NVIC_SetPriority(TCC0_IRQn, 0);  //set NVIC priority for TCC0 to 0 (highest)
  NVIC_EnableIRQ(TCC0_IRQn);  //enable TCC0 timer interrupt
 
  REG_TCC0_INTENSET = TCC_INTENSET_MC1;  //enable compare channel 1 (CC1) interrupts

  REG_TCC0_CTRLA |= TCC_CTRLA_CPTEN1 |              // Enable capture on CC1
                    TCC_CTRLA_PRESCALER_DIV1 |     // Set prescaler to 48MHz
                    TCC_CTRLA_ENABLE;               // Enable TCC0
                    
  while (TCC0->SYNCBUSY.bit.ENABLE);  //wait for sync
 }
