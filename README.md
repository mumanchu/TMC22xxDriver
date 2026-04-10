# TMC22xxDriver Library
An advanced TCM22xx Stepper Motor Controller Library, another library from $${\color{green}mumanchu}$$.

This library is for configuring and monitoring the TRINAMIC TMC22xx range of intelligent Stepper Motor Controller chips via the single-wire UART interface. This includes the TMC2202/2208/2209 and TMC2224/2225/2226. It runs on all STM32, SAMD, AVR, ESP32 and ESP8266 boards, but 32-bit boards are recommended.

The library does not do the STEP/DIR control. That is done by a separate `MiniStepper` library (to be released soon). `MiniStepper` works together with this library or as a stand-alone library for controlling all standard motor controller chips via the STEP/DIR/EN pins. It is a non-blocking library which also provides S-curve acceleration and deceleration.


<!-- ========================================================================================== -->

## Contents
- [Introduction](#introduction)
- [Advantages of This Library](#advantages)
- [TMC2209 Modules](#tmc2209-modules)
- [Recommended 3D Printer or MCU Boards](#recommended-boards)
- [Library API](#api)
- [Example Sketch](#example-sketch)
- [Notes About the Code](#notes)
- [Single Wire UART Interface](#uart)
- [Prototype Shield](#prototype-shield)
- [Silent Running](#silent-running)
- [Velocity Control](#velocity-control)
- [STEP/DIR Control](#step-dir-control)
- [Microstepping](#microstepping)
- [Stall Detection](#stall-detection)
- [One Time Programming (OTP)](#otp)
- [TMC22xx Data Sheets](#data-sheets)


<!-- ========================================================================================== -->

<a name="introduction"></a>
## Introduction

Some of the best stepper motor controller chips were developed by 'TRINAMIC Motion Control Gmbh', which has now been absorbed by 'Analog Devices Inc' to become the 'ADI Trinamic™' range. These intelligent TMC chips have a fast serial interface (UART up to 500K bits-per-second) for configuration and monitoring. The chips also have the standard STEP, DIR and ENABLE signals for traditional stepper motor control.

The Trinamic range has several unique features: _StealthChop™_ and _MicroPlyer™_ for quiet motor operation; _SpreadCycle™_ for dynamic motor control using a "voltage chopper"; _StallGuard™_ for current load and stall detection; and _CoolStep™_ for current control with up to 75% energy savings. You can find out all about these features from the data sheet.

The TMC2202/8/9 and TMC2224/5/6 are all the same apart from the packaging, pinouts, voltage and current ratings. There is a TMC2130 which is similar but uses SPI communications instead for UART. (This library can easily use SPI by modifying `setRegister()` and `getRegister()` to use SPI instead of UART, maybe I'll add that later...)

I have several 3D printer controller boards that use Trinamic chips, and they _all_ use the TMC2209. I've not seen a board that uses the other versions. Some stand-alone controller modules which use the other chips are advertised, but does anybody buy them?

<!-- If you need to learn about stepper motors, check out this voluminous blog entry:
https://muman.ch/muman/index.htm?muman-stepper-motor-control.htm -->

<!-- I'm hoping the next generation of controllers will handle all the acceleration/deceleration internally, so you just select the curve and write a number of steps into a register and it will do the movement itself, signalling when the movement is complete. Then you won't need the `MiniStepper` library :-) -->

<!-- ========================================================================================== -->

<a name="advantages"></a>
## Advantages of This Library 

There are already many TMC2209 libraries for Arduinos. So just to confuse you, I've released another one.

This one is part of a Stepper Motor Control Library suite which is being released in stages. `MultiTimerSAMD` and `OptimizedGPIO` are already available. Next will be the `MiniStepper` library for STEP/DIR control (but until then you can use any of the many STEP/DIR libraries).

In addition:
- This library supports _all_ the TMC chip's features
- Has useful comments in the source code to help understand each method, although you'll still need to read the 80-page data sheet :-( 
- Detects and reports communications errors, with an error count. This is useful in "noisy" environments where high currents are being switched. 
- Uses the chip's received message counter `IFCOUNT` to validate communications (this has not been implemented in any other libraries) 
- Fast look-up table CRC calculation and byte reversal using `__asm__` instructions
- Provides commissioning routines for RMS current calculations based on algorithms in the data sheet
- Works together with the `MiniStepper` library, and should work with any other STEP/DIR library
- Supports [One Time Programming (OTP)](#otp), but it's patched out for safety because it can only be programmed once
- Additional `DEBUG` mode validation:
	- detects writes to read-only registers and reads of write-only registers
	- read-after-write validation of read/write registers
	- validates all TX data that's echoed back to RX, see [Single Wire UART Interface](#uart) below

<!-- ========================================================================================== -->

<a name="tmc2209-modules"></a>
## TMC2209 Modules

If you're not using a 3D printer board, then you'll need a TMC-based driver module.... 




<!-- ========================================================================================== -->


<a name="recommended-boards"></a>
## Recommended 3D Printer or MCU Boards

The library code runs on all architectures, SAMD, AVR, STM32, ESP32 and ESP8266, but 32-bit MCUs are recommended.

Old (or new) 3D printer boards with an ICSP connector can be easily programmed. Many 3D printer boards use TMC2209 chips, but not all have the UART connections, and some have only TX connected. The ICSP connector (In Circuit Serial Programming) is needed to download the program, and you'll need a suitable USB-to-ICSP adapter like the Atmel-ICE or ST-LINK box. 

However, debugging is rarely possible on 3D printer boards because few have the SWD (Serial Wire Debug) or JTAG connectors. So debugging and testing was first done with an Arduino Zero (built-in EDGB debugger) and a Nucleo-64 STM32 board (onboard ST-LINK debugger). A [Prototype Shield](#prototype-shield) was made for the tests. (The Arduino Zero can only support ONE stepper motor because it doesn't have enough outputs.)

If not using a 3D printer board, use a microcontroller board with enough outputs to control the number of stepper motors you will use. Each motor uses at least 3 pins (DIR, STEP and EN). For 4 motor's you will need (at least) 12 outputs, so something more than an Arduino-style board is required. Up to 4 TMC chips can share the same UART connections because the UART protocol contains the "node address" 0..3.

I recommend the 'Bigtreetech SKR Mini E3 V3.0' 3D printer board. This board has a nice STM32F401RCT6 ARM MCU, SWD debug connector, extra EEPROM memory, SD card, end stop inputs, high current MOSFET switched outputs, thermister inputs, etc. It is cheaper than an Arduino and has many more features! It costs around $30, but _rip-off_ prices can be more than double that!

**BIGTREETECH SKR MINI E3 V3.0** \
https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3
[Schematic](https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3/blob/master/hardware/BTT%20SKR%20MINI%20E3%20V3.0.1/Hardware/BTT%20E3%20SKR%20MINI%20V3.0.1_SCH..pdf)

TODO picture


The best MCU board is probably one of the Nucleo-64 STM32 "evaluation boards", like the STM32F446RE. These have a lot of GPIOs, connected to over 100 I/O pins! The STM32F446 board runs at 180MHz, has 512KB Flash, 128KB RAM, floating point FPU and built-in ST-LINK debugging. Available everywhere and it costs less than $20! But no onboard sensors, WiFi, Bluetooth, EEPROM, etc.

**Nucleo-64 STM32F446RE** \
[https://www.st.com/en/evaluation-tools/nucleo-f446re.html](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)



TODO picture

 
<!-- ========================================================================================== -->

<a name="api"></a>
## Library API

The library contains many methods. Only the first two are described here. Refer to the source code and data sheet for detailed descriptions of the others.

```cpp
class TMC22xxDriver
{
	bool begin(Stream* uart, uint address, uint enablePin, 
		uint fullstepsPerRevolution = 200);

	// Override this in a derived class to define your own configuration
	// or just edit the code in this file if the motors are all the same
	virtual bool setConfiguration();

	bool isConnected();
	uint getErrorCount(bool clearErrorCount);
	bool checkIFCNT();
	bool enableMotorDrivers(bool enable = true);
	bool disableMotorDrivers() { return enableMotorDrivers(false); }
	bool enableMotorDriversWithToff(bool enable = true);
	bool disableMotorDriversWithToff() { return enableMotorDriversWithToff(false); }
	bool readShadowRegisters();

	bool getGCONF(GCONF* gconf);
	bool getGSTAT(GSTAT* gstat);
	bool clearGSTAT();
	bool getIFCNT(uint* ifcnt);
	bool getOTP_READ(OTP_READ* otp_read);
	bool getIOIN(IOIN* ioin);
	bool getFACTORY_CONF(uint* fclkTrim, uint* otTrim);
	bool getTSTEP(uint* tstep);
	bool getSG_RESULT(uint* sg_result);
	bool getDRV_STATUS(DRV_STATUS* drv_status);
	bool getPWM_AUTO(uint* pwm_ofs_auto, uint* pwm_grad_auto);
	bool getPWM_SCALE(uint* pwn_scale_sum, int* pwm_scale_auto);

	bool setMicrosteps(uint microsteps);
	int microstepsToMres(uint microsteps);
	int mresToMicrosteps(uint mres);
	bool getMicrosteps(uint* microsteps, bool forceRead = false);
	bool getMicrostepRegisters(uint* mscnt, int* cur_a, int* cur_b);

	bool setDriverCurrent(uint ihold, uint irun, uint iholdDelay);
	bool setCHOPCONF(CHOPCONF chopConf);
	bool setPWMCONF(PWMCONF pwmConf);
	bool setCOOLCONF(COOLCONF coolConf);
	bool setTCOOLTHRS(uint tcoolthrs);
	bool setTPWMTHRS(uint tpwmthrs);
	bool setSGTHRS(uint sgthrs);
	bool setTPOWERDOWN(uint tpowerdown);

	bool velocityMoveStart(long vactual);
	bool velocityMoveStop();
	long rpsToVACTUAL(float revolutionsPerSecond);
	long rpmToVACTUAL(float revolutionsPerMinute);
	long fspsToVACTUAL(float fullstepsPerSecond);

	bool setRegister(uint reg, uint32_t data);
	bool getRegister(uint reg, uint32_t* data);

	// These are only needed for commissioning, they could be commented out
	bool getSettingsFromRmsCurrent(uint iRunCurrentRmsMilliamps, uint iHoldCurrentRmsMilliamps,
		float rsense, bool vsense, uint* irun, uint* ihold);
	bool getRmsCurrentFromSettings(uint irun, uint ihold, float rsense, bool vsense,
		uint* iRunCurrentRmsMilliamps, uint* iHoldCurrentRmsMilliamps);

	// One Time Programming (OTP), patched out with "#if 0 .. #endif" unless needed
	bool programOTP(OTP_READ otpData);
};
```

**`bool begin(Stream* uart, uint address, uint enablePin, uint fullstepsPerRevolution = 200);`** \
Once the serial port has been opened, call this from `setup()` to initialize the TMC chip. It returns `false` if something failed. Always check the return value. \
`uart` is the UART serial port, `address` is the stepper motor number 0..3, `enablePin` is the pin number of the chip's `EN` pin, `fullstepsPerRevolution` can be left as the default 200 (1.8deg/step), or use 400 (0.9deg/step).

**`bool setConfiguration();`** \
This function is called to configure the TMC chip's registers. You can either edit the library code for your desired configuration, or override it in a derived class as shown in the [Example Sketch](#example-sketch). You would normally use the same configuration for each stepper motor. If not, use the override method.

_See the comments in the source code for details of the all the other methods. There are data sheet page references to help too._


<!-- ========================================================================================== -->

<a name="example-sketch"></a>
## Example Sketch

Chip address 0..3 from MS1/MS2 wiring



<!-- ========================================================================================== -->

<a name="notes"></a>
## Notes About the Code

The library is in a single include file that contains both the `TMC22xxDriver` class definition and the code. This is fine if you want to use it in only one source file (.cpp or .ino) - which is recommended. But it won't work if you want to access the class from more than one source file. In that case, put the code into a separate '.cpp' file to be linked only once, keeping the class definition in the include file.

For a bit of variety, 'get' and 'set' are used instead of 'read' and 'write', e.g. `getRegister()` and `setRegister()`. This is like C#'s getters and setters.

The code assumes `byte` and `char` is a byte (8 bits), `int` is 16 ***or*** 32 bits (the library code works for both), `short` is always 16 bits, and `long` is always 32 bits. This is true for all MCUs. Because of this, `int8_t`, `int16_t` or `int32_t` (etc.) are not used unless it is to emphasize the number of bits.

`uint` and `ulong` are also used. You may have to `typedef` those if they are not defined for your platform:
```cpp
typedef unsigned char byte;
typedef unsigned int uint;
typedef unsigned long ulong;
```

The contents of the TMC22xx chip's registers are defined with `struct` and `union`, which gives names to each bit or group of bits. Each register can be handled either as a 32-bit value `data` or as individual items. The code uses the same names as the data sheet, except all lower case for data, and all upper case for register names (the data sheet is inconsistent). Refer to the data sheet for descriptions of the registers and data. A data sheet page number, e.g. p12, is given for each register. 
 
To save reading a register before modifying it, the library uses _shadow registers_ to hold the value last written to certain registers. This is managed automatically by code in `setRegister()`.

For the STM32, assembly language instructions `__asm__` are used to speed up CRC calculation and byte reversal in `crc8()` and `reverse4bytes()`. If not using an STM32, then normal C code is used. (But this will be extended to other MCUs in a future release.)

If `DEBUG` is defined, the code does additional checks to ensure read-only registers are not written to and write-only registers are not read. It can also verify writes to readable registers by doing a write-read-and-compare, but only if the chip's TX pin is connected. This detects a lot of mistakes if you are making changes to the library.

If both RX and TX are connected, it also validates the loopback message, since everything sent is also received (and discarded). This will detect bad connections or noise on the TX/RX lines. If RX is not connected, patch out the `TMC22xxDRIVER_HAS_ECHO` definition to disable this test.


TODO choose a better name for `TMC22xxDRIVER_HAS_ECHO`






<!-- ========================================================================================== -->

<a name="uart"></a>
## Single Wire UART Interface

Serial single-wire UART communications


Up to 4 motors can be connected to the UARTs and share the same TX and RX lines, because the serial protocol contains the "node address" (0..3). All chips listen to the messages sent by the MCU, but only the addressed chip will respond. The chip address is defined by the MS1_AD0 and MS2_AD1 pins (these are not needed for Microstep selection because that can be programmed via the UART interface). On 3D printer boards, each TMC2208 chip has the address hard-wired on the board.


The TMC22xx chips have a dual-purpose pin called PDN_UART. This can either work as a power-down input (for standstill current reduction) or as a fast serial TX/RX port for intelligent control. The mode selection is done via the `pdn_disable` bit in the `GCONF` configuration register. This is done in `setConfiguration()`.

Because there is only one pin for both TX and RX, it's necessary to use an external 1K ohm resistor to stop the MCU's TX driver affecting communications. Many of the stand-alone driver boards have this resistor already fitted. 


<!--image here-->

addressing multiple nodes p21

Some the 3D printer main boards (even the Qidi) do not have the RX connection, so the TMC2209 registers cannot be read - they are all 'write-only'.
Because the MCU's TX is effectively looped back to the RX pin via the 1K resistor, the MCU will receive all the data that it transmits (if the RX pin is connected). This has the advantage that the MCU can validate that the data received by the UART pin was correct and there was no noise on the line that disrupted communications. The Mattlabs library does this check (in DEBUG mode), see setRegister(). 

All sent and received messages have a 1-byte polynomial CRC which will detect most communications errors.
The port runs at up to 500000 baud, automatically adjusting to the baud rate of the received message. Running at a lower speed (say 256000) might be better for noise immunity, especially if you cannot read from the chip to check that everything is working.
Separate power supply pins for I/O (VCC) and the motor (VS)


<!-- ========================================================================================== -->

<a name="prototype-shield"></a>
## Prototype Shield

<!-- circuit diagram -->


Notes
(1) The INDEX and DIAG outputs could be connected, see data sheet
(2) Serial port pins may be different, or you can use SoftwareSerial
(3) Use 5V for a 5V MCU, or 3.3V for a 3.3V MCU, else "Bang!", see Disclaimer.



TWOTREES
   INDEX
     O
   O   O
 DIAG VREF

BIGTREETECH
   VREF
     O
   O   O
INDEX DIAG

Disc Encoder 20 slots




The I/O pins (i.e. STEP/DIR/EN and UART) can be driven by 3.3V or 5V MCUs, and the VIO (or VCC_IO) power pin is used for these. The motor can be supplied with 4.5 .. 28 volts, so there is a separate VM (or VS) pin to power the motor. The motor power also supplies the internal logic via its internal 5V regulator. If only VIO (VCC_IO) is present, UART communications will not work.
My first Arduino shield prototype, with a 24V motor supply, had a hidden short between a motor control pin output and an Arduino input. 24V on the microcontroller input immediately destroyed it! So take care!
The DIAG output



Most 3D printer boards have the TMC2209's RX and TX UART pins connected. But some don't, and some have only the chip's RX pin connected so you can write data but cannot read anything. If the UART pins are not connected then they may be us using the "One Time Programming" (OTP) feature to configure the chip's special features for their own motors - or maybe they are just using the default settings which are (probably) fine for most applications.


TODO
SoftwareSerial can be used but it will not work while the stepper motor interrupt is active because it affects the timing (SoftwareSerial uses delay loops). The solution is to disable the interrupt when sending messages to the TMC2209, which can be done only when the motor is stopped, . SoftwareSerial was used on the Arduino UNO...

"Diagnostic and StallGuard output. High level upon stall detection or driver error. Reset error condition by ENN=high."



<!-- ========================================================================================== -->

<a name="silent-running"></a>
## Silent Running

"Silent Running" was a brilliant 1972 science fiction movie starring Bruce Dern (https://en.wikipedia.org/wiki/Silent_Running). But that's not what this refers to, because they didn't use TMC chips in the movie (or maybe they did, that's why the robots were so quiet).

The TMC chips have a feature called _MicroPlyer™_, which breaks up a single STEP pulse into 256 microsteps, making the motor run very smoothly and quietly. It does this without microsteps being configured, so it works with the standard fullstep STEP control. So if you do one step, the chip actually does 256 microsteps to make the movement smooth. 

This is what they say in the data sheet:
_"The MicroPlyer™ step pulse interpolator brings the smooth motor operation of high-resolution microstepping to applications originally designed for coarser stepping."_

Even if you are using microstepping (via `CHOPCONF::MRES`), the chip will still extrapolate the movement to 256 microsteps. 

_MicroPlyer™_ can be disabled with the `CHOPCONF::intpol` bit. If you do this, the motor will get noticeably louder and will vibrate far more, especially at the motor's resonant frequency.

<!--
> [!TIP]
> You could do something similar with any controller chip by selecting the highest microstep setting and always stepping to the exact fullstep position in microsteps. For example, if 32 microsteps is selected, always do 32 steps for each full step, timing each step to take 1/32 of the full step time for the desired rotation speed. This requires  precise PWM control. See [Microstepping](#microstepping) below.
TODO MiniStepper PWM control
-->


<!-- ========================================================================================== -->

<a name="velocity-control"></a>
## Velocity Control

This library does not control the motor with the STEP/DIR pins, but it _can_ use the TMC's 'internal step pulse generator' to run the motor forwards or backwards at a particular velocity. This makes it run like a normal speed-controlled motor. 

TODO
For precise movements, you could use the `INDEX` pulse output to count the number of steps.

It does not do any acceleration or deceleration, so you must do that yourself. _"Motion at higher velocities will require ramping up and ramping down the velocity value by software."_ 

The velocity is a 24-bit signed value `microsteps / t`, with the direction controlled by the sign (+ve or -ve) of the value.

This library has two methods for velocity control:
```cpp
	// velocity : -8'388'608 .. 8'388'607 (0xff800000 .. 0x007fffff)
	// direction is controlled by the sign (+ve or -ve)
	bool velocityMoveStart(long velocity);
	bool velocityMoveStop();
	long rpmToVelocity(int rpm);
	long spsToVelocity(long sps);
```

<!-- ========================================================================================== -->

<a name="step-dir-control"></a>
## STEP/DIR Control

The TMC chips have the traditional STEP, DIR and EN stepper motor control signals. These are not handled by this library.

The code to do the STEP/DIR control has been put into a separate `MiniStepper` library. This is because the `MiniStepper` library can be used with _any_ motor controller chip, not just the intelligent TMC22xx chips. This is a sophisticated 'non-blocking' library that uses timers and PWM, has S-curve acceleration/deceleration, and controls up to 4 motors simultaneously.

The `MiniStepper` library will be released soon, but until then you can use any of the other stepper libraries. Note that some 'non blocking' libraries rely on a call from `loop()`, so the motor stops while the loop is doing something else. You won't have that problem with `MiniStepper` because the entire movement is controlled by interrupts.




TODO



<!-- ========================================================================================== -->

<a name="microstepping"></a>## Microstepping

This often causes some confusion...


Controller chips select the microstep setting either via two or three input pins, or by writing the microstep setting to a register.

MS0 MS1 MS2


Most steppers have 200 steps-per-turn, which is 1.8 degrees-per-step. Some have 0.9 degree steps, making 400 steps-per-turn. 1.8 degrees (or 0.9) may not provide the resolution you need. In this case you could use microsteps to divide the full step by 2, 4, 8, 16, 32, 64, 128 or max. 256. 

Each STEP pulse will move the motor by the microstep fraction of the full step rotation, by supplying varying voltages to the two coils. This can provide more accurate steps. 

BUT the torque decreases as the number of microsteps increases. At 16 microsteps, the torque per microstep is less than 10% of the fullstep torque. At 32 it's 5% and at 256 it is 0.61%! This means that the motor may not actually move until several microsteps have been made, enough to overcome the load's torque.

_"The consequence is that if the load torque plus the motor’s friction and detent torque is greater than the incremental torque of a microstep, successive microsteps will have to be realized until the accumulated torque exceeds the load torque plus the motor’s friction and detent torque. Simply stated, taking a microstep does not mean the motor will actually move!"_
https://www.faulhaber.com/fileadmin/Import/Media/AN015_EN.pdf
or
https://github.com/mumanchu/mumanchu/tree/main/assets/AN015_EN.pdf


This means that it's usually better to gear down the movement using a lead screw or pulley ratio and use fullsteps instead of microsteps.
However, microsteps are very useful for smooth running, especially if you always step until the full step position is reached. In fact, the Trinamic TMC series controllers do this automatically, dividing each full step into 256 microsteps. This is called _MicroPlyer™_ and it's what makes these controllers "silent". 

TODO microsteps and microPlyer, how does it work



Another disadvantage of microstepping is that the motor may move to the next (or nearest) fullstep position if power is cut to the drivers, so the actual microstep position is lost. The TRINAMIC controllers have a register which holds the microstep position, so you could write some clever software to restore the microstep position when power returns. This is described in the TMC2209 data sheet '3.6.1 Restart the Stepper Motor Without Position Loss', p15. 



INDEX PULSES
- one pulse every 4 fullsteps
- OR one edge for each internal pulse generator step, toggles on each step 


<!-- ========================================================================================== -->


<a name="stall-detection"></a>
## Stall Detection

The TMC chips measure the current through the motor's two coils. They are able to use this to detect if the motor has "stalled" because the current increases if the motor is unable to turn.

In theory, you could use this instead of having end-stop switches. But I spent a lot of time playing with this, and it does not seem to work well at low speeds. So switches are still needed. But stall detection could still be used as a fail-safe in case the low-tech mechanical switch fails.

TODO configuration

TODO diag pin interrupt
diag output was not cleared... ?

TODO DIAG bit can be polled no need for DAIG pin connection



<!-- ========================================================================================== -->

<a name="otp"></a>
## One Time Programming (OTP), p26

_One Time Programming_ is only needed if the chip cannot be configured via the UART. If the UART is not connected, then the default configuration can be programmed into the chip, to be restored on power-up. 

OTP memory can be programmed only once. Bits can be programmed from `0` to `1`, but not back to a `0`.

All the TMC Controller modules have the UART connection, so there is no reason to use the OTP feature. But many of the 3D Printer or CNC boards do _not_ connect the UART pin. In this case they _may_ use the OTP configuration to work with the product's motors, but it's hard to tell. I suspect thye just use the default "all zeros" configuration.

The OTP register data which is restored on power-up can be read from the `OTP_READ` register using `getOTP_READ()`. The default (unprogrammed) value is 0x000000 (24 bits).

If you do want to do One Time Programming, enable the `programOTP()` method in the library. This is patched out with `$if 0` for safety, in case it is called by mistake.

> [!CAUTION]
> **The factory-set clock frequency tuning `FCLKTRIM` should not be changed!**
> `programOTP()` does not allow this.



<!-- ========================================================================================== -->

<a name="data-sheets"></a>
## TMC22xx Data Sheets

TODO store PDF copies in mumanchu/assets

TMC2209_datasheet_rev1.09.pdf

https://github.com/mumanchu/mumanchu/tree/main/assets/TMC2209_datasheet_rev1.09.pdf


All page numbers mentioned in the code (e.g. p20), refer to _this_ TMC2209 data sheet \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf

or here if the link is broken
https://github.com/mumanchu/mumanchu/tree/main/assets/TMC2209_datasheet_rev1.09.pdf


This data sheet covers the TMC2202, TMC2208 and TMC2224 (all the same apart from pinouts and a few characteristics) \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf

TMC2225, this chip has not been tested, but I think it's the same \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2225_datasheet_rev1.14.pdf

**FYSETC-TMC2209** \
This is a great description of a typical TMC2209 board \
https://github.com/FYSETC/FYSETC-TMC2209

**AN-002: Parameterization of StallGuard2™ & CoolStep™** \
The Analog Devices Application Note to help you configure the TMC chips. It's complicated. \  
https://www.analog.com/en/resources/app-notes/an-002.html
https://www.analog.com/media/en/technical-documentation/app-notes/an-002.pdf

TODO pdf was downloaded, put in ASSETS


**Analog Devices TMC2209 Calculator** \
This downloads a spreadsheet with VACTUAL calculations for verification...
https://www.analog.com/media/en/engineering-tools/design-tools/tmc2209_calculations.xlsx


