# TS8000 Library

**IMPORTANT NOTES:
 1) The TS8000 requires a timer counter to perform the calibration routine.  Timer counter
  initialization is specific to a particular microcontroller, so this TS8000 Library
  was developed with the SAMD21 (Arduino ZERO) as the target hardware.  Use of a
  different microcontroller will require the user to modify the timer_init() and
  TC3_Handler() functions found at the bottom of this example sketch for their
  specific hardware.
 
 2) The TS8000 configuration is volatile, so calibrateDevice() must be executed after
  every power cycle.
  
 3) The INPUT port assigned to the DATA signal must be configured as a
  floating input with NO pull-up or pull-down function.  Using a pull-up or
  pull-down function on the input will cause the TS8000 to operate incorrectly.
  
 4) The calibrateDevice() function uses a pin interrupt to capture a free-running
  timer count value on each edge of a ~60us minimum wide pulse that the TS8000
  generates during the calibration routine.  For the SAMD21 running at
  48MHz, the code executed between interrupts executes fast enough that it is 
  finished before the 2nd interrupt on the trailing edge of the pulse occurs.
  If the SAMD21 microcontroller is not used, it is up to the user to verify
  that their microcontroller is running fast enough to finish the code before
  the 2nd interrupt occurs.
  
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


