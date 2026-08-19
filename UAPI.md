## OpenGlow Glowforge Kernel Module User API Documentation
(C) Copyright 2020-2026 Scott Wiederhold    
This work is licensed under a Creative Commons Attribution-ShareAlike 4.0 International License.

This documentation is provided as is without warranty of any kind.  Use at your own risk.
### Overview
This module provides an interface to the Glowforge brand CNC laser hardware (the Basic, Plus, and Pro models on the factory i.MX6 control board).  
It is the OpenGlow fork of [https://github.com/Glowforge/kernel-module-glowforge](https://github.com/Glowforge/kernel-module-glowforge) and differs from the Glowforge original; this documentation applies to the OpenGlow fork only.  

#### Build assumptions
The target is the factory i.MX6 Solo control board, which is **uniprocessor**, and the
module's locking is written for it: the state spinlock (`spin_lock_bh()`) serializes the
main path against the SDMA and hrtimer callbacks, which is sufficient when only one CPU
can execute kernel code at a time. Building for a multiprocessor i.MX6 (Dual/Quad) needs
that locking re-reviewed first — in particular every place a callback reads or writes
driver state without taking the lock.

On a kernel panic the driver stops the EPIT and drives the output pins safe directly (the
laser FIRE line to high impedance, the charge pump low so the hardware watchdog stops
being fed, the latch reset asserted, and the steppers de-energized). SDMA and EPIT run
without the CPU, so without this the machine would play out the rest of the pulse ring
with no kernel alive.

On a dead man's switch trip the head is put in its safe state — measure laser and UV LED
off, lens motor de-energized — and the thermal loop de-energizes its heat sources (loop
heater and TEC). Everything airflow- and circulation-related is deliberately left as it
is: the head fans, the white LED, the coolant pump, and the exhaust/intake fans belong to
the cooling engine and the camera respectively, airflow and coolant circulation after an
aborted cut are wanted, and a pump stop/start cycle can airlock the loop.

### SYSFS / Device Structure
```pre
/sys/glowforge/cnc       <- Motion control, machine state
    |---button_latch:        (RO) Button-latch line state
    |---charge_pump_alive:   (RO) Charge-pump watchdog state
    |---disable:             (WO) Stop all motion and turn off stepper motors and laser
    |---enable:              (WO) Power on steppers and make ready for run
    |---faults:              (RO) Status of stepper axis faults
    |---free:                (RO) Free ring space in bytes (advisory; see the feeder contract)
    |---halt:                (WO) Immediately stop running program (no deceleration)
    |---ignored_faults:      (RW) Stepper axis faults to ignore      
    |---interlock_circuit:   (RO) Safety-chain GPIO snapshot (bitmask)
    |---interlock_latch_reset: (RO) Interlock latch-reset line state
    |---laser_enable:        (RO) Laser-enable (FIRE) line state
    |---laser_latch:         (WO) Enable laser
    |---laser_on:            (RO) Gated LASER_ON output state
    |---laser_on_sampled:    (RO) LASER_ON low-sample count (last ~1s)
    |---laser_pgood:         (RO) Laser power-good state
    |---laser_pgood_sampled: (RO) LASER_PGOOD low-sample count (last ~1s)
    |---motor_lock:          (RW) Disable step output per motor
    |---position:            (RO) Current axis positions and loaded program size/progress
    |---ramp_rate:           (RW) Accel/decel rate in Hz/s
    |---resume:              (WO) Resume paused program
    |---run:                 (WO) Run loaded program
    |---sdma_context:        (RO) Value of SDMA registers
    |---state:               (RO) Current operating state
    |---step_freq:           (RW) Step frequency
    |---stop:                (WO) Controlled stop of running program (decelerate to idle)
    |---streaming:           (RW) Live-feed mode: end-of-data mid-run is an underrun, not completion
    |---underruns:           (RO) Streaming underruns since module load
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
##### button_latch
Read, ASCII, 0-1  
State of the safety chain's button latch: 1 = set (emission blocked until the operator presses the button with the lid closed and `laser_latch` unlocked), 0 = cleared (armed).  

##### charge_pump_alive
Read, ASCII, 0-1  
Logical state of the charge-pump watchdog: 1 = the retriggerable one-shot fed by the driver's charge-pump pulses is still within its period (the HV_ENABLE path is alive), 0 = it has timed out. The line is active low on the board; this attribute reports the logical state. Monitoring only.  

##### disable
Write, ASCII, 1  
Writing "1" to this will switch the device to the "disabled" state.

##### enable
Write, ASCII, 1  
Writing "1" to this will switch the device to the "enabled" (idle) state. From "disabled" this powers the steppers on. From "fault" this is the explicit recovery lever: it clears the latched faults and returns to idle, but only once every non-ignored fault line has physically cleared — a still-asserted line refuses with EPERM.

##### faults
Read, ASCII, 0-7  
Indicates any faults that have been set by the stepper drivers.
Bits: 0: X Axis, 1: Y1 Axis, 2: Y2 Axis  

##### free
Read, ASCII, bytes  
Free space in the pulse ring (already net of the reserved 32 KiB backtrack gap): the largest write that will currently succeed. This is a diagnostic/advisory readback, not the backpressure primitive: a feeder paces by wall clock and takes the write's -ENOMEM return as the back-off signal (see the feeder contract under ```/dev/glowforge```). Each read performs SDMA channel-0 transactions - do not poll it for pacing.

##### halt
Write, ASCII, 1  
Writing "1" performs an immediate stop of a running program: pulse-data processing ends at once with no deceleration, and the device switches to the "idle" state.  
As with ```stop```, the laser-enable line is released (laser off) and the step lines are driven low, while the stepper motors remain powered.  
Unlike ```stop```, there is no controlled ramp-down, so if the machine is moving at speed the motors may lose steps (overshoot) and the reported position may no longer be accurate.  
Use ```halt``` for an immediate/emergency stop where stopping promptly matters more than preserving position; use ```stop``` for a clean, position-preserving stop.  
Has no effect unless the device is in the "running" state.

##### ignored_faults
Read/Write, ASCII, 0-7  
Sets which stepper driver faults to ignore.
Bits: 0: X Axis, 1: Y1 Axis, 2: Y2 Axis  

##### interlock_circuit
Read, ASCII, 0-63  
Raw snapshot of the laser-safety-chain GPIOs as a bitmask. Monitoring only; enforcement is in the hardware AND-gate.  
Bits: 0: LASER_ON, 1: LASER_ENABLE, 2: BUTTON_LATCH, 3: LASER_LATCH, 4: INTERLOCK_LATCH_RESET, 5: CHARGE_PUMP_ALIVE (raw pin, 0 = alive)  
Bits 1, 3 and 4 are driven outputs: mainline gpio-mxc reads output lines back from the data register, so those bits report the value the SoC last drove, not a sense of the pad. Bit 3 is therefore the authoritative view of what the driver commanded the latch to do (`laser_latch` is write-only) and not evidence that the latch hardware responded — the physical proof is `laser_on`/`laser_on_sampled`, which read the gated output of the safety AND-gate.  

##### interlock_latch_reset
Read, ASCII, 0-1  
State of the interlock latch-reset line, the SoC's SET input to the safety chain's interlock latch (read back from the data register, i.e. the value last driven). The driver owns this line: it is 1 (latch set, LASER_ON blocked in hardware) whenever the remote-interlock loop reads open, and also from probe until a switch device reporting the loop has attached - an unobservable loop is treated as open. It is 0 only while an attached switch device reports the loop closed. The loop state comes from the gpio-keys switch device (EV_SW code 5, `interlock`, active = open) through an in-kernel input handler, so no userspace round trip is involved and the GPIO stays owned by gpio-keys. Because the interlock latch is set-dominant, it stays set until this line is released *and* the loop is closed; the latch's own state is the `interlock_latch` switch (EV_SW code 6).  

##### laser_enable
Read, ASCII, 0-1  
State of the laser-enable (FIRE) drive line.  

##### laser_latch
Write, ASCII, 0-1  
Laser lockout latch. 1 = LOCK: the LASER_ON drive line is put in high impedance (the SDMA stream cannot enable the laser). 0 = UNLOCK: the line is restored as a driven output under SDMA control. Lock it whenever no print is in progress; the driver locks it automatically when /dev/glowforge is closed.

##### laser_on
Read, ASCII, 0-1  
Logical state of the gated LASER_ON output of the hardware safety AND-gate (line is active low; 1 = laser on).  

##### laser_on_sampled
Read, ASCII, 0-255  
Number of samples in the last ~1 second window (255 samples, one every ~3.9 ms) in which the LASER_ON line read low. Updated once per window.  

##### laser_pgood
Read, ASCII, 0-1  
Logical state of the laser power-good line (active low; 1 = power good).  

##### laser_pgood_sampled
Read, ASCII, 0-255  
Number of samples in the last ~1 second window (255 samples) in which the LASER_PGOOD line read low. Updated once per window.  

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
12-15: Program bytes processed (a 32-bit SDMA counter: wraps modulo 4 GiB under a long live stream)  
16-19: Program size in bytes (a 64-bit host tally, saturates at 4 GiB under a long live stream)  
20-31: Reserved  

Under a live feed the two byte counters diverge past 4 GiB (one wraps, one saturates); a controller that compares them for progress must track the wrap itself, or use its own count of bytes written.  

##### ramp_rate
Read/Write, ASCII, 10000-500000  
Controlled acceleration/deceleration rate in Hz/s — how fast the step frequency is ramped up or down during a controlled accel/decel (e.g. ```stop```, ```resume```). Default is 125,000.  
Independent of ```step_freq```; the rate is constant rather than scaling with the target speed.  
Cannot be changed while a program is running (returns -EBUSY).  

##### resume
Write, ASCII, -268435455 - 268435455  
Negative values: Laser disabled. Accelerate backwards, run number of specified steps, then decelerate and stop. Refused (EPERM) if the ring has been live-streamed since the last data clear — backtracking is a preload-model operation, and a streamed ring holds stale bytes beyond the retained gap.  
Positive values: Accelerate forward, run number of requested steps, reenable laser, and continue program normally.  
Zero: Accelerate forward, continue program without reenabling laser.  
The step count is a 28-bit waypoint counter; magnitudes at or above 2^28 are rejected with EINVAL rather than silently truncated.

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
"fault": Stepper driver fault. Recoverable via ```enable``` once every non-ignored fault line has physically cleared.  
"underrun": A streaming feeder (see ```streaming```) let the pulse buffer run dry mid-run. The stop is instantaneous (no deceleration), so steps may have been skipped at speed and the reported position can no longer be trusted. New runs are refused until the underrun is acknowledged by writing "1" to ```stop```; the feeder should treat this as an alarm and re-home before continuing.  

##### step_freq
Read/Write, ASCII, 1000-200000  
Step frequency in Hz. Default is 10,000.  

##### stop
Write, ASCII, 1  
Writing "1" performs a controlled stop of a running program: the step frequency ramps down to the minimum, motion comes to a smooth stop, and the device switches to the "idle" state.  
Because the deceleration is controlled, no steps are lost, so the reported position stays accurate.  
Once stopped, the laser-enable line is released (laser off) and the step lines are driven low; the stepper motors remain powered.  
For an immediate stop with no deceleration (at the cost of possibly losing steps), see ```halt```.  
In the "underrun" state, writing "1" acknowledges the underrun and returns the device to "idle" (position should be re-homed before it is trusted).  
Otherwise has no effect unless the device is in the "running" state.

##### streaming
Read/Write, ASCII, 0-1  
Declares how end-of-data is interpreted. A live feeder (one that streams pulse data while the program runs, rather than preloading it) writes "1" before starting a run: running out of data mid-run then transitions the device to the "underrun" state instead of "idle", making buffer starvation distinguishable from normal completion. Write "0" after enqueueing the final bytes of a job so the terminal end-of-data counts as completion.  
Default is 0 (factory/preload behavior: any end-of-data is a normal stop).  
At end-of-data the SDMA script itself forces the laser and step lines low before signaling the host, regardless of this setting.

##### underruns
Read, ASCII, count  
Number of streaming underruns (see ```streaming```) since the module was loaded.

##### x_decay
Read/Write, ASCII, 0-2  
Sets the current decay mode for the X axis.  
0: Slow - fast stop, slow response  
1: Mixed (decay pin Hi-Z)  
2: Fast - fast response, slow stop  

##### x_mode
Read/Write, ASCII, [1, 2, 4, 8, 16, 32]  
Microstepping mode for X Axis.  1 = Full steps  

##### y_decay
Read/Write, ASCII, 0-2  
Sets the current decay mode for the Y axis.  
0: Slow - fast stop, slow response  
1: Mixed (decay pin Hi-Z)  
2: Fast - fast response, slow stop  

##### y_mode
Read/Write, ASCII, [1, 2, 4, 8, 16, 32]  
Microstepping mode for Y Axis.  1 = Full steps  

##### z_step
Write, ASCII, 0-1  
Moves Z-Axis one step in requested direction (direct GPIO pulse, not via the pulse stream).  
0: Negative, toward bed  
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
This changes to 1 when the lens is at or above a specific positive position.  This position varies from unit to unit.  To adjust for this, Glowforge sends a "hunt" program to the device that tells it how many steps toward the bed to move to reach the 0 focus level.  

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
Best guess formula for degrees C: ```(value * 0.08715) - 21``` &mdash;
**still unverified.** The factory firmware instantiates the beta-equation
thermistor conversion (see the coolant section) only for the two water
sensors, so this sensor uses a different path; note the guess has a
*positive* slope, unlike the coolant NTCs.

##### raw
Read/Write, Binary, Varies  
For reading/writing binary values to PIC.
From ```pic.h```:
"Write to this file to send a chunk of raw binary data to the PIC. The number of bytes written must be a multiple of 3.  
Read from this file to obtain the binary data transmitted by the PIC during the previous write transaction."  

##### tec_temp
Read, ASCII, 0-1023  
Thermal Electric Cooler temperature.  
Formula for conversion to temperature is yet to be determined (it does
**not** use the coolant beta conversion - only the two water sensors do).
Pro machines only: on a Basic or Plus there is no TEC and no sensor, and
the reading sits at the 1023 rail.

##### water_temp_1
Read, ASCII, 0-1023  
Water temperature, downstream of heater. See the conversion below.  

##### water_temp_2
Read, ASCII, 0-1023  
Water temperature, upstream of heater. See the conversion below.  

##### Coolant temperature conversion (factory formula)
Both water sensors are 10 k&Omega; B3380 NTC thermistors in a 10 k&Omega;
divider behind a 1.3&times; gain stage, read by a 10-bit ADC. The factory
firmware converts with the single-parameter B (beta) equation:

```
F     = adc_steps * gain          = 1024 * 1.3 = 1331.2
Rinf  = R0 * exp(-beta / T0)      = 10000 * exp(-3380 / 298.15)
R     = Rd / (F / raw - 1)          # Rd = 10000
degC  = beta / ln(R / Rinf) - 273.15 # beta = 3380, T0 = 298.15 K (25 C)
```

Higher raw = colder (NTC), so a rising reading means a falling
temperature. The constants are the factory's own, not a curve fit, which
is why they are exact.

Reference points: raw 640 &rarr; 27.0 C, 680 &rarr; 23.9 C, 740 &rarr;
19.2 C; inversely 31.04 C &rarr; raw 591, 50.01 C &rarr; raw 391.
Bench-checked against a thermometer with the loop at room-temperature
equilibrium: within ~1 C of measured.

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
The heater sits in the coolant loop between the two water temperature
sensors, so heat it puts in shows up as a difference between them. What
to make of that is up to you.  

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
Pro machines only. Basic and Plus have no TEC fitted, and writing here
does nothing on those.  

##### water_pump_on
Read/Write, ASCII, 0-1  
Turns on/off the water pump. 0: off, 1: on  

---
#### /dev/glowforge
Write/Seek/Lock, Binary, ring (size = the ```ring_mb``` module parameter, default 16 MiB; power of two, must fit the cnc reserved-memory pool)  
Interface to the pulse-stream ring buffer. Exclusive-open: a second open fails with EBUSY, so one process holds one fd and routes every write and seek through it.  
Seeking to 0 will clear program data, byte counters, and position counters.  
Seeking to 1 will clear program data and byte counters.  
Seeking to 2 will clear position counters.  
Locking the file (flock LOCK_EX) arms the "dead man's switch": if the fd is closed while locked and a program is running, the device performs an emergency stop. The switch is per-holder state: every fresh open starts with it disarmed, shared locks (LOCK_SH) are rejected with EINVAL, and LOCK_UN disarms.  

##### Pulse-stream feeder contract
Everything a streaming feeder (e.g. a grblHAL step backend) must obey. All of
this is enforced by the SDMA script and driver, not negotiable at run time.

**Byte layout.** One byte per EPIT tick. If bit 7 is clear, the byte is a
step/laser command; if bit 7 is set, the low 7 bits are a laser power level:

    bit 0  X_STEP        bit 4  LASER (fire during this tick)
    bit 1  X_DIR         bit 5  Z_STEP
    bit 2  Y_STEP        bit 6  Z_DIR
    bit 3  Y_DIR         bit 7  0 = step byte, 1 = power byte (bits 0-6 = power)

X/Y convention: DIR bit set = negative direction for X, positive for Y
(Y1/Y2 are driven complementary). Z convention (hardware-verified):
**bit 6 SET moves the lens UP, away from the bed = positive Z**; the
position counter counts it as +Z and the ```z_step``` attr follows the
same sense (1 = away from bed).

**Fixed byte density - no per-byte timing.** The engine consumes exactly one
byte per timer tick at ```step_freq```. All velocity is expressed as step
DENSITY across bytes (software DDS/Bresenham resampling of variable-rate
segments into a fixed-tick stream). ```step_freq``` is immutable while
RUNNING (-EBUSY); mid-run speed change is density, not clock.

**Effective rate ceiling.** The script's per-byte execution time is ~6 us:
above ~165 kHz the EPIT outruns the script and the effective consumption
rate saturates (measured on hardware: 164.6 kHz sustained at
step_freq=200000). Plan machine ticks at or below 100 kHz; 20-50 kHz covers
realistic kinematics with big margin.

**Power bytes.** A power byte sets the laser PWM duty (7-bit, written raw
into PWMSAR; full range at the ~40 kHz carrier). Two rules:
- **Consecutive power bytes are DROPPED**: only the first of a run of
  power bytes applies; the rest are consumed without effect (one power
  change per non-power byte). Interleave a step byte between power changes.
- **Every run started without preserve_power (the plain ```run``` attr)
  resets duty to ~100%.** A stream MUST send its first power byte before its
  first laser-on byte, or the first pulses fire at full power.

**Termination.** End every stream with laser-off bytes (bit 4 clear). The
script forces laser and step lines low at end-of-data as a hardware backstop,
but the stream must not RELY on it: it is the underrun safety net.

**Streaming protocol.** Write ```streaming=1``` before a live-fed run; then
end-of-data mid-run lands in the ```underrun``` state (position no longer
trusted; re-home) instead of "idle", and new runs are refused until
acknowledged via ```stop```. Write ```streaming=0``` after enqueueing the
final bytes of a job so its terminal end-of-data counts as completion.

**Pacing and backpressure.** Writes either commit fully or fail -ENOMEM
(no partials; 32 KiB of ring is reserved as a backtrack gap). Do not poll
```free``` for pacing - every read costs two channel-0 SDMA transactions;
pace by wall clock (enqueued_target = elapsed * step_freq + queue_depth) and
treat -ENOMEM as "back off". Keep the queue depth BOUNDED (50-200 ms) so
feed/power overrides take effect promptly; the ring (default 16 MiB, the
```ring_mb``` module parameter) holds many minutes of stream, so depth is a
latency choice, not a capacity one. NOTE: a whole-file preloader (legacy
cloud mode) is capped by the ring size - ~1 MiB per 100 s of 10 kHz
stream. Measured reference (i.MX6 Solo, PREEMPT,
SCHED_FIFO feeder, full CPU+IO load): 150 ms depth at 100 kHz ran 2 minutes
with 0.2 ms worst write latency and zero underruns.

**Do not write while running backward** (backtrack): the write path would
clobber the backtrack dead-stop; such writes are rejected with -EBUSY.

**Recommended feeder shape.** Hold the fd open + flock'd for the whole job
(deadman armed); prefill one queue depth; ```run```; top up on a 10-20 ms
cadence by wall clock; on ```underrun``` raise a controller alarm, re-home,
ack via ```stop```, seek-clear, regenerate. For feed-hold / jog-cancel use
```stop``` (controlled decel) or ```halt``` + seek-clear + regenerate.

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

The interlock latch-reset line is owned by the cnc driver (see ```interlock_latch_reset```); it follows the remote-interlock switch and is not writable from userspace.
