/*******************************************************************
    Copyright (c) 2018 Triad Semiconductor
    ts8000_config_example.ino - Example application for calibrating
              and configuring the Triad Semiconductor TS8000 Ultrasonic
              to Digital converter.
    Created by: John Seibel
*******************************************************************/
#include <ts8000.h>

//IMPORTANT NOTES:
//
//  THIS LIBRARY IS NOT FOR PRODUCTION USE AND SHOULD ONLY BE USED AS EXAMPLE CODE IN
//  THE DEVELOPMENT OF AN END APPLICATION.
//
//  This is a BETA version of the TS8000 library.  The BETA code adds the functions
//  to measure the time of flight of a 40kHz ultrasonic waveform.  It is considered BETA
//  because the functions added to this version still need some work and the overall
//  software architecture needs much improvment.  However, we wanted to release an early
//  functional version of the library that can perform a very basic 40kHz transmit and
//  receive operation to measure the time of flight of a reflected ultrasonic waveform.
//
// 1) The TS8000 requires timer/capture hardware to perform the library functions.  The
//    initialization is specific to a particular microcontroller, so this TS8000 Library
//    was developed with the SAMD21 (Arduino ZERO) as the target hardware.  Use of a
//    different microcontroller may require the user to modify the following functions
//    for their specific hardware:
//      - timer_init()  -- initializes the TCC1 timer for device calibration
//      - echo_init()  -- initializes the TC3 timer to measure the echo receive time of flight
//      - setup_40khz() -- initializes the TCC0 timer for 40kHz output
//      - enable_40khz() -- enables TCC0 for 40kHz output
//      - disable_40khz() -- disables TCC0 to stop 40kHz output
//      - TCC0_Handler() -- sets up the number of 40kHz pulses to generate
//      - TCC1_Handler() -- reads calibration pulse width
//      - echo_INT() -- reads TC3 count to determine the time of flight of the received echo, also
//                      sets how many pulses into the received pulse train to wait before reading TC3
// 2) The TS8000 configuration is volatile, so calibrateDevice() must be executed after
//    every power cycle
// 3) The INPUT port assigned to the DATA signal must be configured as a
//    floating input with NO pull-up or pull-down function.  Using a pull-up or
//    pull-down function on the input will cause the TS8000 to operate incorrectly.
// 4) The calibrateDevice() function uses a pin interrupt signal as input to the
//    TCC1 timer/capture peripheral.  TCC1 is set up to measure the time between
//    interrupt events to measure an external ~60us minimum wide pulse that the TS8000
//    generates during the calibration routine.
// 5) To simplify the TS8000 Library for use with an external pin interrupt, an additional
//    pin is used for the external interrupt input.  The DATA signal from the TS8000 CM Module
//    should be connected to an Arduino external interrupt pin that will detect a rising
//    and falling edge on the DATA signal for the purposes of calibrating the TS8000 in
//    the calibrateDevice() function.  
// 6) The 40kHz output drive signal is hard coded to Arduino pin D13 (see the setup_40khz()
//    function if you wish to re-assign this pin).  The 40kHz output is used to drive a
//    gate driver chip which drives a piezo transmitter to generate a 40kHz ultrasonic waveform.
// 7) The diagram below illustrates the connections.  The MCP1402T and MA40S4S shown below were
//    used in testing this library and are only examples of a gate driver and piezo transmitter
//    that can be implemented.  Also note that the circuit implements a TS8000 CM Module, which
//    integrates a MEMS microphone to receive the 40kHz ultrasonic echo.
//
//                            ------------
//                            |  TS8000  |
//    -------------           |    CM    |
//    |  Arduino  |           |  MODULE  |
//    |           |           |          |
//    |       CLK |<--------->| CLK      |            Gate
//    |      DATA |<--------->| DATA     |           Driver               Piezo
//    |           |     |     |          |        ------------         Transmitter
//    |       INT |<-----     ------------        | MCP1402T |         -----------
//    |           |                               |          |         | MA40S4S |
//    |       D13 |------------------------------>| IN   OUT |-------->|         |
//    -------------    40kHz Pulse Outout         ------------         -----------
//
//

//#define device1_CLK_pin      your_CLK_pin   //User must replace your_CLK_pin with their pin number (compile error will occur if no number defined)
//#define device1_DATA_pin     your_DATA_pin  //User must replace your_DATA_pin with their pin number (compile error will occur if no number defined)
#define device1_CLK_pin      5   //User must replace your_CLK_pin with their pin number (compile error will occur if no number defined)
#define device1_DATA_pin     6  //User must replace your_DATA_pin with their pin number (compile error will occur if no number defined)

//#define IDEAL_CAL_COUNT_VAL  your_count_val  //User must replace your_count_val with their count value that corresponds to the count value
                                               //at 75us - for a 48MHz counter clock, this value is 3600 decimal
#define IDEAL_CAL_COUNT_VAL  3600    //This value is assigned in the example sketch since the SAMD21 (Arduino ZERO) is the assumed target
                                     //and the timer_init() function configures the timer/capture hardware for 48MHz.  This #define should
                                     //only be changed if the target hardware is counting at a clock tick other than 48MHz.  The TS8000 calibration
                                     //FILTER_TRIM setting is adjusted until the calibration pulse has a width of 75us.  The #define value of 3600
                                     //decimal is the count value of the TCC1 timer running at 48MHz for a 75us pulse.  Equation is 1/48e6Hz * 3600 = 75us

//#define device1_INT_pin      your_INT_pin    //User must replace your_INT_pin with their pin number (compile error will occur if no number defined)
#define device1_INT_pin      12   //the timer_init() function assumes the use of pin 12 for the external pin interrupt (SAMD21 / Arduino ZERO)
                                  //so this value is assigned in the example sketch

#define ECHO_RX_DELAY 3   //implement a delay in ms after 40kHz pulse transmit before echo receive is enabled on Arduino (system dependent, may
                          //not be required for all hardware implementations)

#define ECHO_RX_PULSE_OFFSET 8  //sets the number of pulses from the beginning of the received 40kHz pulse echo to wait before the time of flight
                                //counter in TC3 is read (this value is system dependent and was arbitrarily chosen for the example sketch)

#define PULSE_COUNT_40KHZ 13  //sets the number of 40kHz pulses to generate with the TCC0 timer (this number is system dependent and was
                              //arbitrarily chosen for the example sketch)

uint16_t  user_config_word = 0x082C;  //default, uncalibrated configuration word is 0x082C, the user can modify this to include their
                                      //specific register settings in the upper 9 bits prior to executing calibrateDevice() - calibrateDevice()
                                      //does not modify the upper 9 bits, only the lower 7 bits for the FILTER_TRIM setting
uint16_t    calibrated_config_word;   //configuration word returned by calibrateDevice() after the FILTER_TRIM bits have been calibrated, the
                                      //upper 9 bits of this word are unmodified from the user_config_word that was passed to calibrateDevice()
char buf[20];  //character buffer which contains the HEX formatted configuration word string for display in the serial monitor window

//The following variables are used to share data with the interrupt service routines so they are defined as volatile
volatile uint8_t   edge_count;
volatile uint16_t  echo_count;
volatile uint8_t   echo_count_done;

TS8000  device1(device1_CLK_pin, device1_DATA_pin, IDEAL_CAL_COUNT_VAL);  //instantiate the class as device1 and assign pins

void setup() {
  pinMode(device1_INT_pin, INPUT);
  Serial.begin(9600);
  while (!Serial);  //wait for serial port to connect

  Serial.println("Serial Port Connected");
  Serial.println();
  Serial.println();

  echo_init();  //configure the TC3 timer to measure the echo time of flight
  setup_40khz();  //configure TCC0 to generate a 40kHz output waveform
  timer_init();  //configure TCC1 to capture interrupt events from the pin interrupt

  attachInterrupt(device1_INT_pin, NULL, CHANGE);  //enable interrupt on digital pin (formatted for Arduino ZERO, user may need to modify
                                                   //if using a different Arduino board)
  Serial.println("Beginning Calibration and Configuration...");
  calibrated_config_word = device1.calibrateDevice(user_config_word);  //performs configuration and calibration of the TS8000 device
  detachInterrupt(device1_INT_pin);  //disable interrupt on digital pin (formatted for Arduino ZERO, user may need to modify
                                     //if using a different Arduino board) - the pin will be re-attached to a different interrupt
                                     //function for the time of flight measurement

  if (calibrated_config_word == 0x0000) Serial.println("Calibration Unseccessful - Configuration Verify Failed");
  else {
    Serial.print("Calibration and Configuration Successful - Configuration Word = ");
    sprintf(buf, "0x%04X", calibrated_config_word);
    Serial.println(buf);
    }


  attachInterrupt(device1_INT_pin, echo_INT, RISING);  //enable interrupt on digital pin for use in measuring time of flight (formatted
                                                       //for Arduino ZERO, user may need to modify if using a different Arduino board)
  }
  
void loop() {
  enable_40khz();  //start 40kHz pulse output
  delay(ECHO_RX_DELAY);  //get past initial reflections if any, may not be required for all hardware implementations
  echo_count_done = false;  //reset flag that indicates when the TC3 timer is read during the received 40kHz echo
  while(!echo_count_done);  //wait for echo to be received
  Serial.print("Echo count value (decimal) = ");
  Serial.println(echo_count, DEC);
  Serial.print("Distance = ");
  Serial.print(((echo_count*1.0)/1000.0*1.087/2.0), 2);  //convert the TC3 timer count value to a distance in feet
  Serial.println(" ft.");
  Serial.println("----------------------------------");

  delay(1000);
  }

//*****  The following functions require the use of timer/capture hardware and interrupts, and are
//*****  specific to the SAMD21 (Arduino ZERO) microcontroller.  The code for these funcitons was put
//*****  in this example sketch for easy visibility by the user.  In order to make that work, global
//*****  variables were needed to pass data between this example sketch and the TS8000 class.  They
//*****  are identified below in the code and must not be modified without appropriate modifications
//*****  to the TS8000 Library code.
//
//The TCC1 timer/capture peripheral is used to calculate the width of a calibration
//pulse that the TS8000 generates.
void TCC1_Handler(void)
{
  calibration_count = REG_TCC1_CC1;  //read the time between pin interrupt events, which
                                     //correspond to the edges of the TS8000 calibration pulse,
                                     //calibration_count is global and used by the TS8000 class
                                     //so do not modify it
  TCC1_int_count++;  //count to indicate how many times the interrupt has occurred,
                     //TCC0_int_count is global and used by the TS8000 class
                     //so do not modify it
}

void timer_init(void) {
  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;//enable the event system peripheral
 
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN |
                                 GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID_TCC0_TCC1) ;  //enable clk for TCC1
  while (GCLK->STATUS.bit.SYNCBUSY); //wait for sync 
                          
  REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO3;  //enable event output on external interrupt 3 (Arduino ZERO pin 12, SAMD21)

  REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) |  //attach the event user (receiver) to channel 0 (n + 1)
                   EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1);  //set the event user (receiver) as timer TCC1, event 1

  REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |  //no event edge detection
                      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |  //event path is asynchronous (pass-thru)
                      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) |  //set event generator (sender) as external interrupt 3 (Arduino ZERO pin 12, SAMD21)
                      EVSYS_CHANNEL_CHANNEL(0);  //attach the generator (sender) to channel 0

  REG_TCC1_EVCTRL |= TCC_EVCTRL_MCEI1 |           // Enable the match or capture channel 1 event input
                     TCC_EVCTRL_TCEI1 |           // Enable the TCC event 1 input
                     TCC_EVCTRL_EVACT1_PWP;       // Set up the timer for capture: CC1 period, CC0 pulsewidth

  NVIC_SetPriority(TCC1_IRQn, 0);  //set NVIC priority for TCC0 to 0 (highest)
  NVIC_EnableIRQ(TCC1_IRQn);  //enable TCC0 timer interrupt
 
  REG_TCC1_INTENSET = TCC_INTENSET_MC1;  //enable compare channel 1 (CC1) interrupt

  REG_TCC1_CTRLA |= TCC_CTRLA_CPTEN1 |              // Enable capture on CC1
                    TCC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 48MHz
                    TCC_CTRLA_ENABLE;               // Enable TCC1
                    
  while (TCC1->SYNCBUSY.bit.ENABLE);  //wait for sync
 }

/*
speed of sound in air (first order effect dominates and is due to temp
Sair m/s = (331.3 + (0.606 * T degC)) m/s

*/
void TCC0_Handler(void){  //counts edges on the 40kHz TX pulses
  if (TCC0->INTFLAG.bit.OVF) {
    REG_TCC0_INTFLAG |= TCC_INTFLAG_OVF;  //clear overflow interrupt
    if (edge_count == ((PULSE_COUNT_40KHZ - 1)*2)) {  //convert PULSE_COUNT_40KHZ pulse value to an edge count
                                                      //the first edge is not an overflow, so it doesn't trigger the interrupt
      disable_40khz();  //disable the 40kHz output after specified number of pulses
    }
    else if (edge_count == 1){
      TC3->COUNT16.COUNT.reg = 0x0000;  //reset echo counter to start time of flight measurement
      while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync
      edge_count++;
    }
    else edge_count++;
  }
}

// Output 40kHz square wave on timer TCC0 digital pin D13
 void setup_40khz(void) {

  REG_PM_APBCMASK |= PM_APBCMASK_EVSYS;  //enable the event system peripheral

  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN |
                                 GCLK_CLKCTRL_GEN_GCLK0 |
                                 GCLK_CLKCTRL_ID_TCC0_TCC1) ;  //enable clk for TCC1
  while (GCLK->STATUS.bit.SYNCBUSY); //wait for sync 

  // Enable the port multiplexer for digital pin 13 (D13): timer TCC0 output
  PORT->Group[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port output - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[11].ulPort].PMUX[g_APinDescription[11].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  REG_TCC0_INTENSET |= TCC_INTENSET_OVF;  //enable the overflow interrupt

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NFRQ;    // Setup wave generation for normal frequency
  while (TCC0->SYNCBUSY.bit.WAVE);           //wait for sync

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC0_PER = 599;      // Set the frequency of the PWM on TCC0 to 40kHz
  while(TCC0->SYNCBUSY.bit.PER);

  NVIC_EnableIRQ(TCC0_IRQn);  //attach TCC0 interrupt output to NVIC (used for the OVF interrupt)

  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1;  //no prescaler divider (48MHz)
}

//start 40kHz output by enabling TCC0
void enable_40khz(void){
  edge_count = 0;
  TCC0 -> CTRLA.bit.ENABLE = 1;  // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);  //wait for sync
}

//disable 40kHz output by disabling TCC0
void disable_40khz(void){
  TCC0 -> CTRLA.bit.ENABLE = 0;  //Disable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);  //wait for sync
  edge_count = 0; //reset edge counter for next 40kHz pulse output
}

//Configure TC3 as a free-running timer to be used for measuring the 40kHz reflected time
//of flight.  TC3 count is reset when the 40kHz output begins, and then read when the
//reflected pulses are detected.  It is a 16-bit counter configured to count at 1MHz, so
//the max time of flight that can be measured is approximately 65ms or 22m round trip.
void echo_init(void) {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) |  // Divide the 48MHz clock source by divisor 48 = 1MHz
                    GCLK_GENDIV_ID(4);     // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);       //wait for sync

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              //wait for sync

  //enable the clock for TC3 
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID ( GCM_TCC2_TC3 ) ) ;
  while ( GCLK->STATUS.bit.SYNCBUSY);  //wait for sync 

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |  //timer counter mode set to 16 bits
                           TC_CTRLA_PRESCALER_DIV1 | //prescaler is 1
                           TC_CTRLA_ENABLE;  //enable TC3
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync
}

void echo_INT()  //interrupt service routine for TC3
{
  if (!echo_count_done){
    if (edge_count < ECHO_RX_PULSE_OFFSET) {
      edge_count++; //increment edge counter to count edges into the received pulse train
    }
    else {
      echo_count = TC3->COUNT16.COUNT.reg;  //read count
      while (TC3->COUNT16.STATUS.bit.SYNCBUSY);  //wait for sync
      echo_count_done = true;  //echo found, so indicate done
    }
  }
}

