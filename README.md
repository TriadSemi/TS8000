# TS8000 Library

#IMPORTANT NOTES:

  THIS LIBRARY IS NOT FOR PRODUCTION USE AND SHOULD ONLY BE USED AS EXAMPLE CODE IN
  THE DEVELOPMENT OF AN END APPLICATION.

  This is a BETA version of the TS8000 library.  The BETA code adds the functions
  to measure the time of flight of a 40kHz ultrasonic waveform.  It is considered BETA
  because the functions added to this version still need some work and the overall
  software architecture needs much improvment.  However, we wanted to release an early
  functional version of the library that can perform a very basic 40kHz transmit and
  receive operation to measure the time of flight of a reflected ultrasonic waveform.

 1) The TS8000 requires timer/capture hardware to perform the library functions.  The
    initialization is specific to a particular microcontroller, so this TS8000 Library
    was developed with the SAMD21 (Arduino ZERO) as the target hardware.  Use of a
    different microcontroller may require the user to modify the following functions
    for their specific hardware:
      - timer_init()  -- initializes the TCC1 timer for device calibration
      - echo_init()  -- initializes the TC3 timer to measure the echo receive time of flight
      - setup_40khz() -- initializes the TCC0 timer for 40kHz output
      - enable_40khz() -- enables TCC0 for 40kHz output
      - disable_40khz() -- disables TCC0 to stop 40kHz output
      - TCC0_Handler() -- sets up the number of 40kHz pulses to generate
      - TCC1_Handler() -- reads calibration pulse width
      - echo_INT() -- reads TC3 count to determine the time of flight of the received echo, also
                      sets how many pulses into the received pulse train to wait before reading TC3
 2) The TS8000 configuration is volatile, so calibrateDevice() must be executed after
    every power cycle
 3) The INPUT port assigned to the DATA signal must be configured as a
    floating input with NO pull-up or pull-down function.  Using a pull-up or
    pull-down function on the input will cause the TS8000 to operate incorrectly.
 4) The calibrateDevice() function uses a pin interrupt signal as input to the
    TCC1 timer/capture peripheral.  TCC1 is set up to measure the time between
    interrupt events to measure an external ~60us minimum wide pulse that the TS8000
    generates during the calibration routine.
 5) To simplify the TS8000 Library for use with an external pin interrupt, an additional
    pin is used for the external interrupt input.  The DATA signal from the TS8000 CM Module
    should be connected to an Arduino external interrupt pin that will detect a rising
    and falling edge on the DATA signal for the purposes of calibrating the TS8000 in
    the calibrateDevice() function.  
 6) The 40kHz output drive signal is hard coded to Arduino pin D13 (see the setup_40khz()
    function if you wish to re-assign this pin).  The 40kHz output is used to drive a
    gate driver chip which drives a piezo transmitter to generate a 40kHz ultrasonic waveform.
 7) The diagram below illustrates the connections.  The MCP1402T and MA40S4S shown below were
    used in testing this library and are only examples of a gate driver and piezo transmitter
    that can be implemented.  Also note that the circuit implements a TS8000 CM Module, which
    integrates a MEMS microphone to receive the 40kHz ultrasonic echo.

```
                            ------------
                            |  TS8000  |
    -------------           |    CM    |
    |  Arduino  |           |  MODULE  |
    |           |           |          |
    |       CLK |<--------->| CLK      |            Gate
    |      DATA |<--------->| DATA     |           Driver               Piezo
    |           |     |     |          |        ------------         Transmitter
    |       INT |<-----     ------------        | MCP1402T |         -----------
    |           |                               |          |         | MA40S4S |
    |       D13 |------------------------------>| IN   OUT |-------->|         |
    -------------    40kHz Pulse Outout         ------------         -----------
```

