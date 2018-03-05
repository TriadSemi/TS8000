# TS8000 Library

IMPORTANT NOTES:
 1) The TS8000 requires timer/capture hardware to perform the calibration routine.  The
    initialization is specific to a particular microcontroller, so this TS8000 Library
    was developed with the SAMD21 (Arduino ZERO) as the target hardware.  Use of a
    different microcontroller will require the user to modify the timer_init() and
    TCC0_Handler() functions found at the bottom of this example sketch for their
    specific hardware.
    
 2) The TS8000 configuration is volatile, so calibrateDevice() must be executed after
    every power cycle.
    
 3) The INPUT port assigned to the DATA signal must be configured as a
    floating input with NO pull-up or pull-down function.  Using a pull-up or
    pull-down function on the input will cause the TS8000 to operate incorrectly.
    
 4) The calibrateDevice() function uses a pin interrupt signal as input to the
    TCC0 timer/capture peripheral.  TCC0 is set up to measure the time between
    interrupt events to measure an external ~60us minimum wide pulse that the TS8000
    generates during the calibration routine.
    
 5) To simplify the TS8000 Library for use with an external pin interrupt, an additional
    pin is used for the external interrupt input.  The DATA signal from the TS8000
    should be connected to an Arduino external interrupt pin that will detect a rising
    and falling edge on the DATA signal for the purposes of calibrating the TS8000 in
    the calibrateDevice() function.  The diagram below illustrates the connections.

**    -------------           ------------
**    |  Arduino  |           |  TS8000  |
**    |           |           |          |
**    |       CLK |<--------->| CLK      |
**    |      DATA |<--------->| DATA     |
**    |           |     |     |          |
**    |       INT |<-----     |          |
**    |           |           |          |
**    -------------           ------------


