## OpenGlow Glowforge Kernel Module User API Documentation
(C) Copyright 2020 Scott Wiederhold    
This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.

This documentation is provided as is without warranty of any kind.  Use at your own risk.
### Overview
This module provides an interface to the Glowforge brand CNC laser hardware, supporting the current production Basic, Plus, and Pro models (as of March 2020).  
NOTE: This is an OpenGlow Fork of [https://github.com/Glowforge/kernel-module-glowforge](https://github.com/Glowforge/kernel-module-glowforge).  
There are differences between the OpenGlow implementation and the Glowforge original.  
This documentation only applies to the OpenGlow fork.  

### SYSFS / Device Structure
```pre
/sys/glowforge/cnc       <- Motion control, machine state
    |---disable:             (WO) Stop all motion and turn off stepper motors and laser
    |---enable:              (WO) Power on steppers and make ready for run
    |---faults:              (RO) Status of stepper axis faults
    |---ignored_faults:      (RW) Stepper axis faults to ignore      
    |---laser_latch:         (WO) Enable laser
    |---motor_lock:          (RW) Disable step output per motor
    |---position:            (RO) Current axis positions and loaded program size/progress
    |---resume:              (WO) Resume paused program
    |---run:                 (WO) Run loaded program
    |---sdma_context:        (RO) Value of SDMA registers
    |---state:               (RO) Current operating state
    |---step_freq:           (RW) Step frequency
    |---stop:                (WO) Stop/Pause currently running program
    |---x_decay:             (RW) Enable/Disable X-Axis decay
    |---x_mode:              (RW) X-Axis Micro-stepping mode
    |---y_decay:             (RW) Enable/Disable Y-Axis decay
    |---y_mode:              (RW) Y-Axis Micro-stepping mode
    |---z_step:              (WO) Single step Z-Axis

/sys/glowforge/head      <- Laser head hardware
    |---accel_irq:           (RO) Head accelerometer IRQ tripped flag
    |---air_assist_pwm:      (RW) Air Assist fan PWM output setting
    |---air_assist_tach:     (RO) Air Assist fan tachometer reading
    |---beam_detect_analog:  (RO) Current state of beam detector - analog
    |---beam_detect_digital: (RO) Current state of beam detector - digital
    |---hall_sensor:         (RO) Status of lens hall sensor
    |---measure_laser:       (RW) Output PWM of material height measuring laser
    |---purge_air:           (RW) Purge Air fan on/off
    |---purge_air_current:   (RO) Purge Air fan current
    |---uv_led:              (RW) Output PWM of UV LED
    |---white_led:           (RW) Output PWM of White LED
    |---z_current:           (RW) High/Low Z-Axis current
    |---z_enable:            (RW) Z-Axis driver enable
    |---z_mode:              (RW) Z-Axis Micro-stepping mode

/sys/glowforge/pic       <- PIC analog/digital I/O
    |---button_led_1:        (RW) Output PWM of Button Red LED
    |---button_led_2:        (RW) Output PWM of Button Green LED
    |---button_led_3:        (RW) Output PWM of Button Blue LED
    |---grp_all:             (RO) All register values in binary format
    |---grp_button_leds:     (RW) All button LEDs, in binary format
    |---grp_hv:              (RO) All HV sensors, in binary format
    |---grp_outputs:         (RW) All outputs, in binary format
    |---grp_sensors:         (RO) All inputs, in binary format
    |---hex:                 (RW) Read/Write registers in ASCII Hex
    |---hv_current:          (RO) HV current measurement
    |---hv_voltage:          (RO) HV voltage measurement
    |---id:                  (RO) PIC Firmware ID
    |---lid_ir_1:            (RO) Lid IR sensor 1 measurement
    |---lid_ir_2:            (RO) Lid IR sensor 2 measurement
    |---lid_ir_3:            (RO) Lid IR sensor 3 measurement
    |---lid_ir_4:            (RO) Lid IR sensor 4 measurement
    |---lid_led:             (RW) Output PWM of Lid LEDs
    |---pwr_temp:            (RO) Power supply temperature
    |---raw:                 (RW) Read/Write binary data to/from registers
    |---tec_temp:            (RO) Thermo-Electric Cooler temperature
    |---water_temp_1:        (RO) Coolant temperature, downstream
    |---water_temp_2:        (RO) Coolant temperature, upstream
    |---x_step_current:      (RW) X-Axis stepper driver current
    |---y_step_current:      (RW) Y-Axis stepper driver current

/sys/glowforge/thermal   <- Cooling hardware
    |---exhaust_pwm:         (RW) Exhaust fan PWM output setting
    |---heater_pwm:          (RW) Coolant heater PWM output setting
    |---intake_pwm:          (RW) Intake fan PWM output setting
    |---tach_exhaust:        (RO) Exhaust fan tachometer reading
    |---tach_intake_1:       (RO) Intake fan 1 tachometer reading
    |---tach_intake_2:       (RO) Intake fan 2 tachometer reading
    |---tec_on:              (RW) Enable/Disable Thermo-electric cooler
    |---water_pump_on:       (RW) Enable/Disable water pump

/dev/glowforge:              (RW) Load/Clear program, reset positions

/sys/class/leds/button_led_X  <- Big Button LED interfaces
    |---pulse_off:           (RW) Off time in milliseconds
    |---pulse_on:            (RW) On time in milliseconds
    |---speed:               (RW) Speed to target brightness
    |---target:              (RW) Brightness set-point
    |---(standard LED interfaces not used)

/sys/class/leds/camera_mux_oe  <- Enable output for camera mux
    |---brightness:          (RW) Output level
    |---(standard LED interfaces not used)

/sys/class/leds/interlock_reset  <- Reset interlock circuit
    |---brightness:          (RW) Output level
    |---(standard LED interfaces not used)

/sys/class/leds/lid_led_X  <- Big Button LED interfaces
    |---pulse_off:           (RW) Off time in milliseconds
    |---pulse_on:            (RW) On time in milliseconds
    |---speed:               (RW) Speed to target brightness
    |---target:              (RW) Brightness set-point
    |---(standard LED interfaces not used)
```

### SYSFS Interface Descriptions
---
#### /sys/glowforge/cnc
##### disable
Write, ASCII, 1  
Writing "1" to this will switch the device to the "disabled" state.

##### enable
Write, ASCII, 1  
Writing "1" to this will switch the device to the "enabled" state.

##### faults
Read, ASCII, 0-7  
Indicates any faults that have been set by the stepper drivers.
Bits: 0: X Axis, 1: Y1 Axis, 2: Y2 Axis  

##### ignored_faults
Read/Write, ASCII, 0-7  
Sets which stepper driver faults to ignore.
Bits: 0: X Axis, 1: Y1 Axis, 2: Y2 Axis  

##### motor_lock
Read/Write, ASCII, 0-15  
Sets axis lock. If the axis bit is set, that axis will not move when running a program.  
NOTE: This does not prevent the Z axis from moving when commanded by ```z_step```.  
Bits: 0: X Axis, 1: Y1 Axis, 2: Y2 Axis, 3: Z Axis  

##### position
Read, Binary, 32 bytes (little-endian)  
Current axis position and program size and position.  
Bytes: Value  
00-03: X position in steps  
04-07: Y position in steps  
08-11: Z position in steps  
12-15: Program bytes processed  
16-19: Program size in bytes
20-31: Reserved  

##### resume
Write, ASCII, -2147483647 - 2147483647  
Negative values: Laser disabled. Accelerate backwards, run number of specified steps, then decelerate and stop.  
Positive values: Accelerate forward, run number of requested steps, reenable laser, and continue program normally.  
Zero: Accelerate forward, continue program without reenabling laser.

##### run
Write, ASCII, 1  
Writing "1" to this will switch the device to the "run" state from "idle" and starts executing the loaded program.

##### sdma_context
Read, ASCII, (formatted)  
This dumps internal registers and status of the CNC SDMA context.  

##### state
Read, ASCII, CNC State  
The current CNC state:  
"idle": Steppers are on but no program is in progress  
"running": A program is in progress  
"disabled": Steppers are disabled, no program in progress  
"fault": Stepper driver fault  

##### step_freq
Read/Write, ASCII, 1000-200000  
Step frequency in Hz. Default is 10,000.  

##### stop
Write, ASCII, 1  
Writing "1" to this will disable laser, decelerate the device to a stop, and switch to the "idle" state from "run".

##### x_decay
Read/Write, ASCII, 0-1  
Sets the current decay mode for the X axis.  
0: Slow - fast stop, slow response  
1: Fast - fast response, slow stop  

##### x_mode
Read/Write, ASCII, [1, 2, 4, 8, 16, 32]  
Microstepping mode for X Axis.  1 = Full steps  

##### y_decay
Read/Write, ASCII, 0-1  
Sets the current decay mode for the Y axis.  
0: Slow - fast stop, slow response  
1: Fast - fast response, slow stop  

##### y_mode
Read/Write, ASCII, [1, 2, 4, 8, 16, 32]  
Microstepping mode for Y Axis.  1 = Full steps  

##### z_mode
Write, ASCII, 0-1  
Moves Z-Axis one step in requested direction.
0: Negative, towards bed  
1: Positive, away from bed  

---
#### /sys/glowforge/head
##### accel_irq
Read, ASCII, 0-1  
Indicates if the head accelerometer set its IRQ.  

##### air_assist_pwm
Read/Write, ASCII, 0-1023  
Air assist fan PWM period.
0: Off  
1023: Full speed  
For some reason, Glowforge never sets this to below 204, so the fan is never off.  

##### air_assist_tach
Read, ASCII, 0-65535  
Period between tach pulses.  
Best guess RPM formula: ```((1/(period/1000000))*60)/8```  

##### beam_detect_analog
Read, ASCII, 0-65535  
Analog output from beam detector.  
How this operates still needs to be investigated.  

##### beam_detect_digital
Read, ASCII, 0-1  
Digital output from beam detector.  
How this operates still needs to be investigated.  

##### hall_sensor
Read, ASCII, 0-1  
Output from lens home position sensor.  
0: Not at home position  
1: At home position  
This changes to 1 when the lens is at or above a specific positive position.  This position varies from unit to unit.  To adjust for this, Glowforge sends a "hunt" program to the device that tells it how many steps towards the bed to move to reach the 0 focus level.  

##### measure_laser
Read/Write, ASCII, 0-1023  
Measurement laser PWM. 0: Off, >0: On
Any value above 0 will turn the head measurement laser on.  

##### purge_air
Read/Write, ASCII, 0-1  
Turns the purge air fan on/off. 0: off, 1: on  
This fan purges smoke from the lens cavity.  

##### purge_air_current
Read, ASCII, 0-1023  
The current being drawn by the purge fan.  
Meaning of the values is unknown at this point. Observed values are: 1 when off, and 628 when on.  

##### uv_led
Read/Write, ASCII, 0-1023  
Head UV illumination LED PWM. 0=Off, 1023=100%

##### white_led
Read/Write, ASCII, 0-1023  
Head white illumination LED PWM. 0=Off, 1023=100%

##### z_current
Read/Write, ASCII, 0-1  
Z stepper drive current.  0: high, 1: low

##### z_enable
Read/Write, ASCII, 0-1  
Enable/disable Z driver.  0: enabled, 1: disabled

##### z_mode
Read/Write, ASCII, 0-1  
Z Axis microstepping.  0: Full, 1: 2 (half-step)

---
#### /sys/glowforge/pic
##### button_led_1, button_led_2, button_led_3
Read/Write, ASCII, 0-1023 (0=OFF, 1023=FULL)  
Red (1), Green (2), and Blue (3) LEDs in the big button.  
These are not intended to be set directly.  Instead, they should be set using the ```/sys/class/leds/button_led_X``` interface. See description below.

##### grp_X
Read/Write, Binary, Varies
Used to read/write data in groups.  
From ```pic.h```:
"Register groups. These allow reading or updating multiple registers at once. Input/output is a sequence of 16-bit binary little-endian values. When writing to a register group file, the number of bytes written must exactly match the number of registers in the group times 2. (because each register is 2 bytes in size.)"  

##### hex
Read/Write, ASCII, Varies
For reading and writing multiple registers.
From ```pic.h```:
"To write a set of registers:  
```echo 18=0123,19=4567,1a=89ab,1b=cdef > /sys/glowforge/pic/hex```  
The string must be a comma-separated list of register=value pairs.  
To read a set of registers:  
```echo 18,19,1a,1b > /sys/glowforge/pic/hex && cat /sys/glowforge/pic/hex```  
The string must be a comma-separated list of register numbers.  
Reading from this file returns the register values transmitted by the PIC during the previous write transaction. The string is a comma-separated list of register values, each exactly 4 hex characters long."  

##### hv_current
Read, ASCII, 0-1023  
HV current.  
The exact meaning of this value is yet to be determined.  

##### hv_voltage
Read, ASCII, 0-1023  
HV voltage.  
Note: To date, every power supply that has been examined ties the input to this A/D to ground.

##### id
Read, ASCII, 19795  
Firmware ID of PIC Analog/Digital IO.  

##### lid_ir_X
Read, ASCII, 0-1023  
Output of IR sensors on lid.  
Presumably, these can be used to detect fires within the unit.  

##### lid_led
Read/Write, ASCII, 0-1023 (0=OFF, 1023=FULL)  
Lid LEDs.  
This is not intended to be set directly.  Instead, it should be set using the ```/sys/class/leds/lid_led``` interface. See description below.

##### pwr_temp
Read, ASCII, 0-1023  
Power Supply temperature.  
Best guess formula for degrees C: ```(value * 0.08715) - 21```  

##### raw
Read/Write, Binary, Varies  
For reading/writing binary values to PIC.
From ```pic.h```:
"Write to this file to send a chunk of raw binary data to the PIC. The number of bytes written must be a multiple of 3.  
Read from this file to obtain the binary data transmitted by the PIC during the previous write transaction."  

##### tec_temp
Read, ASCII, 0-1023  
Thermal Electric Cooler temperature.  
Formula for conversion to temperature is yet to be determined.  

##### water_temp_1
Read, ASCII, 0-1023  
Water temperature, downstream of heater.  
Best guess formula for degrees C: ```(value * -0.09653) + 94```  

##### water_temp_2
Read, ASCII, 0-1023  
Water temperature, upstream of heater.  
Best guess formula for degrees C: ```(value * -0.09653) + 94```  

##### x_step_current
Read/Write, ASCII, 0-127  
X stepper drive current. 0=Min, 127=Max

##### y_step_current
Read/Write, ASCII, 0-31  
Y stepper drive current. 0=Min, 31=Max

---
#### /sys/glowforge/thermal
##### exhaust_pwm
Read/Write, ASCII, 0-65535  
Exhaust fan PWM period. 0: Off, 65535: Full speed  

##### heater_pwm
Read/Write, ASCII, 0-65535  
Water heater PWM period. 0: Off, 65535: Full power  
This heats the water between the water temp sensors to allow for flow detection.  

##### intake_pwm
Read/Write, ASCII, 0-65535  
Intake fans PWM period. 0: Off, 65535: Full power  
This controls the output for both intake fans.  

##### tach_exhaust
Read, ASCII, 0-65535  
Period between exhaust fan tach pulses.  
Best guess RPM formula: ```((1/(period/1000000000))*60)/2```  

##### tach_intake_1
Read, ASCII, 0-65535  
Period between intake fan 1 tach pulses.  
Best guess RPM formula: ```((1/(period/1000000000))*60)/2```  

##### tach_intake_2
Read, ASCII, 0-65535  
Period between intake fan 2 tach pulses.  
Best guess RPM formula: ```((1/(period/1000000000))*60)/2```  

##### tec_on
Read/Write, ASCII, 0-1  
Turns on/off the thermal electric cooler. 0: off, 1: on  

##### water_pump_on
Read/Write, ASCII, 0-1  
Turns on/off the water pump. 0: off, 1: on  

---
#### /dev/glowforge
Write/Seek/Lock, Binary, 128MB max  
Interface to the program buffer.  Programs are one byte per step period or laser power setting. Programs are written directly to buffer.  
Seeking to 0 will clear program data, byte counters, and position counters.  
Seeking to 1 will clear program data and byte counters.  
Seeking to 2 will clear position counters.  
Locking the file will set the "deadman switch". If the file is closed while the deadman switch is active, the device will perform and emergency stop.  

---
#### /sys/class/leds/button_led_X, /sys/class/leds/lid_led_X
Interface to control button LEDs.  
From ```ledtrig_smooth```:  
"target: (range: [0, 255]) The new brightness set-point. The LED fades from its current value to the target value. May be changed while the LED is already fading.  
speed: (range: [0, 160]) The speed at which the LED seeks its target brightness. The default is 64.  
pulse_on:  (milliseconds)  
pulse_off: (milliseconds)  
When both values are > 0, the LED's target will alternate between minimum and maximum brightness automatically.  
The delay between target=255 and target=0 is specified by pulse_on.  
The delay between target=0 and target=255 is specified by pulse_off.  
Both values are internally truncated to multiples of MSECS_PER_UPDATE.  

#### /sys/class/leds/camera_mux_oe
This controls the output on the camera multiplexer.  The ```brightness``` value should be set to 255.  Setting a value of 0 will shut the output off.  There is no reason to ever shut the output off.  

#### /sys/class/leds/interlock_reset
This resets the interlock safe circuit.  Setting ```brightness``` to 255 resets the circuit.  This should be set to 0 for normal operation.
