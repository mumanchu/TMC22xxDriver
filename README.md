# TMC22xxDriver Library, from $${\color{green}mumanchu}$$

There are already _many_ TMC2209 libraries out there. So just to confuse you, here is another one.

This library is for configuring and monitoring the TRINAMIC TMC22xx range of intelligent Stepper Motor Controller chips via the single-wire UART interface. This includes the TMC2202/2208/2209 and TMC2224/2225/2226. It runs on all STM32, SAMD, AVR, ESP32 and ESP8266 microcontrollers, but 32-bit MCUs are recommended.

It can be used with stand-alone [TMC22xx modules](#tmc22xx-modules), or with old (or new) 3D printer main boards which can be re-programmed via an ICSP, SWD or JTAG connector.

The library does not do the STEP/DIR control. That is done by a separate STEP/DIR library called `MiniStepper`, to be released soon. (`MiniStepper` works together with this library or as a stand-alone library for controlling all standard motor controller chips via the STEP/DIR/EN pins. It is a non-blocking library which also provides S-Curve acceleration/deceleration.)

![TMC2209 Module](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/tmc2209-module.jpg)  ![Bigtreetech Board](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/bigtreetech-skr-mini-e3.jpg)


**All page numbers in the code and documentation (e.g. p20), refer to this TMC2209 data sheet** \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf


<!-- ========================================================================================== -->

## Contents
- [Introduction](#introduction)
- [Advantages of This Library](#advantages)
- [Library API](#api)
- [Example Sketch](#example-sketch)
- [Notes About The Code](#notes)
- [TMC22xx Modules](#tmc22xx-modules)
- [Recommended 3D Printer and MCU Boards](#recommended-boards)
- [Single Wire UART Interface](#uart)
- [Prototype Shield](#prototype-shield)
- [Silent Running](#silent-running)
- [Velocity Control](#velocity-control)
- [STEP/DIR Control](#step-dir-control)
- [Microstepping](#microstepping)
- [Driver Diagnostics](#driver-diagnostics)
- [Stall Detection: Does it work?](#stall-detection)
- [One Time Programming (OTP)](#otp)
- [Data Sheets and References](#data-sheets)
- [Revision History](#revision-history)

_\<i think you've written way too much, nobody's ever going to read it, not even me - ed\>_


<!-- ========================================================================================== -->

<a name="introduction"></a>
## Introduction

Some of the best stepper motor controller chips were developed by 'TRINAMIC Motion Control Gmbh', which has now been absorbed by 'Analog Devices Inc' to become the 'ADI Trinamic™' range. These intelligent TMC chips have a fast single-wire serial interface (UART up to 500KBaud) for configuration and monitoring. The chips also have the standard STEP, DIR and ENABLE signals for traditional stepper motor control.

The TRINAMIC range has several unique features: _StealthChop™_ and _MicroPlyer™_ for quiet motor operation; _SpreadCycle™_ for dynamic motor control using a "voltage chopper"; _StallGuard™_ for current load and stall detection; and _CoolStep™_ load-adaptive current regulation with up to 75% energy savings. You can find out all about these features from the data sheet.

The TMC2202/8/9 and TMC2224/5/6 are all the same apart from the packaging, pinouts, voltage and current ratings. There is also a TMC2130 which is similar but uses SPI communications instead for UART.

I have several 3D printer controller boards that use TRINAMIC chips, and they _all_ use the TMC2209. I've not seen a board that uses the other versions. Some stand-alone controller modules which the other chips are advertised, but does anybody buy them?

> **DREAM ON** \
> One day, a new generation of TMC chips might handle the acceleration/deceleration curves internally, so you select the curve (Trapezoidal or S-Curve) and the acceleration, and write the number of steps into a register. The chip then manages the movement itself, signalling when the movement is complete. Then you won't need the `MiniStepper` library.


<!-- ========================================================================================== -->

<a name="advantages"></a>
## Advantages of This Library 

This library is part of a "Stepper Motor Control Suit" which is being released in stages. `MultiTimerSAMD` and `OptimizedGPIO` are already available. Next will be the `MiniStepper` library for STEP/DIR control (but until then you can use any of the many Arduino STEP/DIR libraries).

In addition:
- This library supports _all_ the TRINAMIC chip's features.
- It has useful comments in the source code to help understand each method, although you'll still need to refer to the 80-page data sheet. The relevant data sheet page numbers are given to speed things up.
- Provides commissioning routines for RMS current calculations based on algorithms in the data sheet.
- Works together with the `MiniStepper` library, and should work with any other STEP/DIR library.
- Supports [One Time Programming (OTP)](#otp), but it's patched out for safety because it can only be programmed once.
- Uses the chip's received message counter `IFCOUNT` to validate communications.
- Detects and reports all UART communications errors, with an error count. This is useful during development and testing.
- Fast look-up table CRC calculation (the example code in the data sheet is quite slow).
- Optional `DEBUG` mode validation:
	- detects writes to read-only registers and reads of write-only registers
	- read-after-write validation of read/write registers
	- validates all TX data that's echoed back to RX, see [Single Wire UART Interface](#uart) below

 
<!-- ========================================================================================== -->

<a name="api"></a>
## Library API

The library contains many methods. Only the first two, `begin()` and `setConfiguration()`, are described here. Refer to the commented [Source Code](https://github.com/mumanchu/TMC22xxDriver/tree/main/src) and [Data Sheet](#data-sheets) for detailed descriptions of the other methods.

```cpp
class TMC22xxDriver
{
	bool begin(Stream* uart, uint address, uint enablePin, uint fullstepsPerRevolution = 200);
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

	bool setCHOPCONF(CHOPCONF chopConf);
	bool setPWMCONF(PWMCONF pwmConf);
	bool setCOOLCONF(COOLCONF coolConf);
	bool setTCOOLTHRS(uint tcoolthrs);
	bool setTPWMTHRS(uint tpwmthrs);
	bool setSGTHRS(uint sgthrs);
	bool setTPOWERDOWN(uint tpowerdown);
	bool setDriverCurrent(uint ihold, uint irun, uint iholdDelay);

	bool setMicrosteps(uint microsteps);
	int microstepsToMres(uint microsteps);
	int mresToMicrosteps(uint mres);
	bool getMicrosteps(uint* microsteps, bool forceRead = false);
	bool getMicrostepRegisters(uint* mscnt, int* cur_a, int* cur_b);

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

	// One Time Programming (OTP), commented out unless needed
	bool programOTP(OTP_READ otpData);
};
```

**`bool begin(Stream* uart, uint address, uint enablePin, uint fullstepsPerRevolution = 200);`** \
Once the serial port has been opened, call this from `setup()` to initialize the TMC chip. It returns `false` if something failed. Always check the return value. \
`uart` is the UART serial port, `address` is the stepper motor number 0..3, `enablePin` is the pin number of the chip's `EN` pin, `fullstepsPerRevolution` can be left as the default 200 (1.8deg/step), or use 400 (0.9deg/step).

**`virtual bool setConfiguration();`** \
This function is called to configure the TMC chip's registers. You can either edit the library code for your desired configuration, or override it in a derived class as shown in the [Example Sketch](#example-sketch). You would normally use the same configuration for each stepper motor. It's a good idea not to rely on the "power up reset" default values, because the chip may not always be reset - it has no RESET pin!

_See the comments in the source code for details of the other methods. There are also data sheet page references to help, e.g. "p12"._


<!-- ========================================================================================== -->

<a name="example-sketch"></a>
## Example Sketch

See [tmc2209-test.ino](https://github.com/mumanchu/TMC22xxDriver/blob/main/examples/tmc2209-test/tmc2209-test.ino).

To use the example sketch you will need to build a circuit similar to the [Prototype Shield](#prototype-shield), which connects the TMC22xx module to the MCU's I/Os. Or if you are running it on an existing 3D printer board, get the `pins_xxx.h` file from the relevant Marlin github repository to find the UART and STEP/DIR pins for the board.

The example sketch is quite big and may look confusing at first. It illustrates several things:

- Debug code using the `ASSERT()` and `LOGERROR()` macros in `MumanchuDebug.h`
- Creating a derived class to override the default `setConfiguration()` code in the library
- Using the `Tacho.h` class to count and timer input interrupts for the INDEX and TACHO inputs
- Accelerating and decelerating a stepper motor using the TMC chip's [Velocity Control](#velocity-control) feature

### `DEBUG` Code

The `ASSERT()` macro checks data at run time, and reports invalid values to Serial. `LOGERROR()` outputs and error message with the file and line number. If the DEBUG symbol is not defined then these macros do nothing.
 All $${\color{green}mumanchu}$$ libraries use these macros.

### Derived Class for `setConfiguration()`

To avoid editing the code in `TMC22xxDriver.h`, you can create a derived class and override the `setConfiguration()` to configure the TMC chip. Normally all the motors will have the same configuration. 

### `Tacho` Class

The INDEX and TACHO outputs are connected to inputs that generate interrupts. The interrupts are timed and counted by `Tacho` objects. See `Tacho.h`.


<!-- ========================================================================================== -->

<a name="notes"></a>
## Notes About The Code

The library is in a single include file `TMC22xxDriver.h` that contains both the `TMC22xxDriver` class definition and the code. This is fine if you want to use it from only _one_ source file (.cpp or .ino) - which is recommended. But it won't work if you want to access the class from more than one source file. In that case, cut-and-paste the _code_ part into a separate '.cpp' file to be linked only once, leaving the class definition in the include file.

For a bit of variety, 'get' and 'set' are used instead of 'read' and 'write', e.g. `getRegister()` and `setRegister()`. This is like C#'s getters and setters.

The contents of the TMC22xx chip's registers are defined with `struct` and `union`, which gives names to each bit or group of bits. Each register can be handled either as a 32-bit value `data` or as individual items. The code uses the same names as the data sheet, except lower case for data, and all upper case for register names (the data sheet is inconsistent). Refer to the data sheet for descriptions of the registers and data. A data sheet page number, e.g. p12, is given for each register. 
 
To save reading a register before modifying it, the library uses _shadow registers_ to hold the value last written to certain registers. This is managed automatically by code in `setRegister()`.

For the STM32, assembly language instructions `__asm__` are used to speed up CRC calculation and byte reversal in `crc8()` and `reverse4bytes()`. If not using an STM32, then normal C code is used. (But this will be extended to other MCUs in a future release.)

If `DEBUG` is defined, the code does additional checks to ensure read-only registers are not written to and write-only registers are not read. It can also verify writes to readable registers by doing a write-read-and-compare, but only if the chip's TX pin is connected. This detects a lot of mistakes if you are making changes to the library.

If both RX and TX are connected, it also validates the loopback message, since everything sent is also received (and normally discarded). This will detect bad connections or noise on the TX/RX lines. If RX is not connected, patch out the `TMC22xxDRIVER_LOOPBACK` definition to disable this test.

The native `int` (16 or 32 bits) is used for efficiency (there's no need to force it to use `int16_t` if it's a 32-bit processor). The code assumes `byte` and `char` is a byte (8 bits), `int` is 16 ***or*** 32 bits (the library code works for both), `short` is always 16 bits, and `long` is always 32 bits. This is true for all MCUs. Because of that, `int8_t`, `int16_t` or `int32_t` (etc.) are not used unless it is to emphasize the number of bits.

The types `ushort`, `uint` and `ulong` may also be used (System V compatibility). You might have to `typedef` these if they are not defined for your platform (see`types.h`).
```cpp
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;
```

<!-- ========================================================================================== -->

<a name="tmc22xx-modules"></a>
## TMC22xx Modules

If you're not using a 3D printer board, then you'll need a TMC-based driver module for each motor. There are many modules that use the TMC2209 chip, and all of them will work with this library. Here's just a few of them... 

![TMC2209 Modules](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/tmc2209-modules.jpg)

All these modules have similar pinouts, EXCEPT for three pins on the left hand side, marked with the red box in the picture below. These can be TX/RX/CLK, UART/PDN/CLK, SP/TX/RX, R8/UART/NC or SPRD/UART/PDN etc. Check these pins carefully. Some already have the 1K resistor for TX, see [Single Wire UART Interface](#uart) below. 

The three pins or holes next to the trimmer (in a triangle format) are usually DIAG, VREF and INDEX. The lower two pins, DIAG and INDEX are aligned on 2.54mm, so a standard pin connector can be used for these two. The TWOTREES and BIGTREETECH boards have the three pins wired as DIAG, INDEX and VREF, so INDEX is not aligned for 2.54mm pin connector! (But I cut the VREF track and connected the pin to INDEX so it fitted my prototype shield.) There are no standards for these three pins.

In the pictures below, pins 4, 5 and 6 on the left are marked with the red boxes, these may be different. The three aux pins on the top left are also different. The remaining pins are always the same.

![TMC2209 Pinouts](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/tmc-board-pinouts.png) \
_(On the middle board, is pin 6 CLK or TYPE? And what is TYPE? The documentation is often wrong.)_

<!-- ========================================================================================== -->

<a name="recommended-boards"></a>
## Recommended 3D Printer and MCU Boards

The `TCM22xxDriver` library runs on all architectures, SAMD, AVR, STM32, ESP32 and ESP8266, but 32-bit MCUs are recommended.

Old (or new) 3D printer boards with an ICSP connector for programming can be used. Many recent 3D printer boards use TMC2209 chips, but not all have the UART connections, and some have only RX connected so the TMC chips can only be written to. The ICSP connector (In Circuit Serial Programming) is needed to download the program (or upload, if you are an Arduino), and you'll need a suitable USB-to-ICSP adapter like the Atmel-ICE or ST-LINK box. 

However, debugging is rarely possible on 3D printer boards because few have the SWD (Serial Wire Debug) or JTAG connectors. So debugging and testing was first done with a Nucleo-64 STM32 board (onboard ST-LINK debugger), and an Arduino Zero (built-in EDGB debugger). A [Prototype Shield](#prototype-shield) was made for the tests. Note that the Arduino Zero can only support one stepper motor because it doesn't have enough outputs.

If not using a 3D printer board, use a microcontroller board with enough outputs to control the desired number of stepper motors. Each motor uses at least 3 pins (DIR, STEP and EN). For 4 motor's you will need (at least) 12 outputs, so something more than an Arduino-style board is required. Up to 4 TMC chips can share the same UART connections because the UART protocol contains the "node address" 0..3.

### BIGTREETECH SKR MINI E3 V3.0

I recommend the 'Bigtreetech SKR Mini E3 V3.0' 3D printer board. This board has a nice STM32F401RCT6 ARM MCU _with an SWD debug connector_, four TMC2209 chips, extra EEPROM memory, SD card, filtered end-stop inputs, high current MOSFET switched outputs, thermister analog inputs, etc. It is cheaper than an Arduino, costing less than $30 (but _rip-off_ prices can be more than double that - do not buy those!)

https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3 \
[Schematic V3.0.1](https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3/blob/master/hardware/BTT%20SKR%20MINI%20E3%20V3.0.1/Hardware/BTT%20E3%20SKR%20MINI%20V3.0.1_SCH..pdf) \
![Bigtreetech SKR MINI E3](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/bigtreetech-skr-mini-e3-small.png)

### Nucleo-64 STM32F446RE

The best MCU boards are the Nucleo-64 STM32 evaluation boards, like the STM32F446RE. These have the standard set of Arduino connectors, plus over 100 additional I/O pins! The STM32F446RE board runs at 180MHz, has 512KB Flash, 128KB RAM, floating point uint (FPU) and _fantastic_ built-in ST-LINK debugging. It costs less than $20! These are _great_ for debugging. But they have no onboard sensors, EEPROM, SD card, high-power outputs, etc. 

https://www.st.com/en/evaluation-tools/nucleo-f446re.html \
![Nucleo-64 STM32F446RE](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/nucleo-64-stm32f446re.jpg)


<!-- ========================================================================================== -->

<a name="uart"></a>
## Single Wire UART Interface

The TMC22xx chips have a dual-purpose pin called PDN_UART. This can either work as a power-down input (for standstill current reduction) or as a fast serial TX/RX port for intelligent control. The mode selection is done via the `pdn_disable` bit in the `GCONF` configuration register. See the library's `setConfiguration()` method.

The UART pin is normally an RX input, listening for messages. When it receives a message with the chip's "node address" (0..3, as defined by the AD0/AD1 pins), it becomes a TX output and sends a response, then switches back to RX mode. The chip automatically adjusts to the baud rate of the incomming message, 9600..500'000 baud. Recommended is around 256'000 baud - it does not need to be super fast.

All messages have a 1-byte polynomial CRC which is used to validate each message.

Up to 4 motors can be connected, sharing the same TX and RX lines. Each message sent by the MCU contains a "node address" 0..3. All chips listen to the messages sent by the MCU, but only the addressed chip will respond. On 3D printer boards, each TMC2208 chip has the address hard-wired on the board. The AD0/AD1 pins can also be used as MS0/MS1 to select the microsteps, but because UART programming is used, the microsteps are progammed via the `CHOPCONF` register's `MRES` bits.

There is only one pin for both TX and RX, so it's necessary to use an external 1K ohm resistor to prevent conflicts between the MCU's TX driver and the TMC chip's UART outputs. Some of the stand-alone driver boards have this resistor already fitted. If not, you must add it to your circuit.

![TMC22xx TX Resistor](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/tmc2209-tx-rx.jpg)


A few of the 3D printer main boards do not have the RX connection, so the TMC22xx registers cannot be read. Only the writeable registers can be written. Not even the status registers can be read. This is not good.

The MCU's TX is effectively looped back to the RX pin via the 1K resistor, so the MCU will receive all the data that it transmits (if the RX pin is connected). This has the advantage that the MCU can validate that the data received by the UART pin was correct and there was no noise on the line that disrupted communications. The TMC22xxDrive library library does this check (in `DEBUG` mode), see `setRegister()`. But this is probably overkill.

It is easy to address more than four TMC chips. The data sheet (p21) recommends an "analog switch" 74HC4066 to select individual chips with individual "chip select" outputs. That allows MC0/MC1 to be used for microstep selection, but it uses one additional output per chip which is not very efficient.

A better way is to use a single output to control a 1-pole 2-way (single pole double throw SPDT) analog switch to select a bank of four TMC chips. Bank0 = 0..3 and Bank1 = 4..7, so node addresses 0..3 can be used for both banks of four chips. This can be done with a cheap DG41xx analog multiplexer chip (Vishay or Maxim), or even an old CD4016, CD4051, CD4053 etc.

![TMC Multiplexer](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/tmc-multiplexer.png)


<!-- ========================================================================================== -->

<a name="prototype-shield"></a>
## Prototype Shield

For library development, a prototype shield was designed that works with the Arduino Zero and the Nucleo-64 STM32 boards. The prototype has jumpers for selecting the RX/TX pins, because the Nucleo boards use D0 and D1 for USB so they won't work with the TMC's UART, but the Arduino Zero can use the standard D0/D1 pins. 

![TMC22xx Prototype Shield](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/prototype-shield-2.png)

**NOTES**
1. On some modules, the INDEX and DIAG pins may be reversed, or one may be VREF.
2. These jumpers select the RX/TX pins which wil be used. For the Arduino Zero, jumper 1-2 and 4-5. For the Nucleo-64 jumper 2-3 and 5-6.
3. Use 5V for a 5V MCU, or 3.3V for a 3.3V MCU, else "BANG!". See Disclaimer. 3.3V usually works for 5V MCUs, but not the other way round!
4. The ZERO does not support interrupts on pin D4. So use D10 instead for INDEX or DIAG (see note 1).

For validating the motor movement, a disc encoder with 20 slots was fitted to the motor's spindle. This is read using an "opto interrupter" (like the ITR9606 or ITR9608) to provide 20 pulses-per-revolution into the TACHO (tachometer) input.

![Tachometer](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/tmc-tachometer.png)

The INDEX and TACHO inputs generate iterrupts which are handled by the `Tacho` class in the [Example Sketch](*example-sketch), see `Tacho.h`.

The I/O pins (i.e. STEP/DIR/EN and UART) can be driven by 3.3V or 5V MCUs, the VIO (or VCC_IO) power pin is used for these. 

The motor power VM is supplied by an external 12V or 24V power supply (1A per motor). VM also supplies the internal logic via the TMC chip's internal 5V regulator. UART communications will not work without it.

![Prototype Shield Photo](https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/prototype-shield.jpg)


> [!CAUTION]
> My first shield prototype, with a 24V motor supply, had a hidden short between a motor control output pin and an MCU input. 24V on the MCU input immediately destroyed it. 12V or 24V and 3.3V MCU I/Os do not mix! Also note that the VIO pin (the MCU's VCC) is right next to a stepper motor control output - that's VERY dangerous.


<!-- ========================================================================================== -->

<a name="silent-running"></a>
## Silent Running

"Silent Running" was a brilliant 1972 science fiction movie starring Bruce Dern (https://en.wikipedia.org/wiki/Silent_Running). But that's not what this refers to, because they didn't use TRINAMIC chips in those days. Or maybe they did, that's why the robots were so quiet.

The TRINAMIC chips have a feature called _MicroPlyer™_, which breaks up a single STEP pulse into 256 microsteps, making the motor run very smoothly and quietly. It does this without microsteps being configured, so it works with the standard fullstep STEP control. So if you do one step, the chip actually does 256 microsteps to make the movement smooth. 

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
## Velocity Control, p67

This library does not control the motor with the STEP/DIR pins, but it _can_ use the TMC chip's 'internal step pulse generator' to run the motor forwards or backwards at a particular velocity. This makes it run like a speed-controlled motor. 

The velocity is set by the VACTUAL register. The VACTUAL value is in _microsteps per time interval 't'_, where 't' is (fclk / 2^24). The `xxxToVACTUAL()` methods can be used to get the `vactual` value of a speed in revolutions-per-minute, revolutions-per-second or fullsteps-per-second. There is also an Analog Devices spreadsheet for this, see the link at the end of the page.

To control thw direction, a +ve `vactual` value rotates one way and a -ve `vactual` rotates the other way. 

The chip does not do any acceleration or deceleration, so you must do that in software. The data sheet says, _"Motion at higher velocities will require ramping up and ramping down the velocity value by software."_ It's true. If you try to start the motor at a speed above its starting speed, it will not move, and sits there making very strange noises. But it works pefectly if you ramp up the speed slowly.

Maximum speeds are about 300rpm, 5rps or 1000sps, but it depends on the motor and its load.

The library has these methods for velocity control:
```cpp
	bool velocityMoveStart(long vactual);
	bool velocityMoveStop();
	long rpmToVACTUAL(float revolutionsPerMinute);
	long rpsToVACTUAL(float revolutionsPerSecond);
	long fspsToVACTUAL(float fullstepsPerSecond);
```

The INDEX output (p66) can be used to count the steps from the internal step pulse generator. It is _toggled_ on each step, so the interrupt must be on both rising and falling edges. 

The [Example Sketch](#example-sketch) illustrates acceleration and deceleration using velocity control, and the counting of INDEX pulses.

<!-- ========================================================================================== -->

<a name="step-dir-control"></a>
## STEP/DIR Control, p63

The TMC chips use the traditional STEP, DIR and EN stepper motor control signals. 

The code to do the STEP/DIR control has been put into a separate `MiniStepper` library. This is because the `MiniStepper` library can be used with _any_ motor controller chip, not just the intelligent TMC22xx chips. `MiniStepper` is a 'non-blocking' library that uses a single hardware timer, has S-curve acceleration/deceleration, and controls up to 4 motors simultaneously.

The `MiniStepper` library will be released soon, but until then you can use any of the other stepper libraries. Note that some 'non blocking' stepper motor libraries rely on a call from `loop()`, so the motor stops or slws down while the loop is doing something else. You won't have this problem with `MiniStepper` because all movements are controlled by the timer interrupt.


<!-- ========================================================================================== -->

<a name="microstepping"></a>
## Microstepping

Microstepping is often misunderstood.

Most steppers have 200 fullsteps-per-turn, which is 1.8 degrees-per-step. Some have 0.9 degree steps, making 400 steps-per-turn.

When microstepping is enabled, it divides the full step by 2, 4, 8, 16, 32, 64, 128 or max. 256 'microsteps'. Each STEP pulse will move the motor by the microstep fraction of the full step, by supplying varying voltages to the two coils.

BUT the _torque decreases_ as the number of microsteps increases. At 16 microsteps, the torque per microstep is less than 10% of the fullstep torque. At 32 it's 5%, and at 256 it is 0.61%! This means that the motor may not actually move until several microsteps have been made, enough to overcome the torque.

_"The consequence is that if the load torque plus the motor’s friction and detent torque is greater than the incremental torque of a microstep, successive microsteps will have to be realized until the accumulated torque exceeds the load torque plus the motor’s friction and detent torque. Simply stated, taking a microstep does not mean the motor will actually move!"_ - Dr. Fritz Faulhaber

This means that it's sometimes better to gear down the movement using a lead screw or pulley ratio and use fullsteps instead of microsteps. 

However, microsteps are very useful for smooth running, especially if you always step until the full step position is reached. In fact, the Trinamic TMC series controllers do this internally, dividing each full step into 256 microsteps. This is called _MicroPlyer™_ and it's what makes these controllers "silent".

Another disadvantage of microstepping is that if power is cut to the drivers, the motor will move to the nearest fullstep position - unpowered motors always lock at the fullstep position. This makes re-starting an operation at the _exact_ place where is stopped is very difficult. The TRINAMIC controllers have an `MSCNT` register which holds the microstep position, so clever software is needed to restore the exact microstep position when power is returned to the drivers. This is described in the TMC2209 data sheet p15, 'Restart the Stepper Motor Without Position Loss'.

<!-- TODO add a library feature to do this? -->

Dr. Fritz has a great introduction to microstepping... \
https://www.faulhaber.com/fileadmin/Import/Media/AN015_EN.pdf


### Configuring microsteps, p33

When UART communications is used, the TMC controller chips configure the microstep setting via the `CHOPCONF` register's `MRES` bits (Microstep Resolution, p33). The library has a `bool setMicrosteps(uint microsteps)` method to do this. But first you must enable `MRES` with the `mstep_reg_select` bit in the `GCONF` register, see `setConfiguration()`. If this bit not set, it will attempt to use the MS0 and MS1 pins for the microstep setting.

<!-- TODO if 4 boards are connected, how can they be programmed if AD0/AD1 are MS0.MS1 by default? -->

<!-- ========================================================================================== -->

<a name="driver-diagnostics"></a>

## Driver Diagnostics, p68

Another nice feature of the TMC chips is the runtime diagnostics. It can detect:

- Over temperature
- Coil short circuit 
- Coil open circuit
- Charge pump under voltage
- [Stall Detection](#stall-detection)

In each case the DIAG output and `IOIN.DIAG` flag is set. See the diagram on data sheet p69. Individual status bits are accessed via the `DRV_STATUS` register, p37. 

It's a good idea to check for errors regularly at runtime, so the movement can be stopped if something nasty happens. Using the DIAG output to generate an interrupt is the fastest way to react to an error. And/or poll the status bits.

<!-- ========================================================================================== -->


<a name="stall-detection"></a>
## Stall Detection: Does it work? p58

The TMC chips measure the current through the motor's coils. This can be used to detect if the motor has "stalled". For example, if it has hit the end-stop or your tie has caught in the pulley wheel and you are being strangled ;-)

In theory, you can use 'stall detection' instead of having end-stop switches. But at low speeds it does not work. On p28, the data sheet states, _"DIAG is pulsed by StallGuard when SG_RESULT falls below SGTHRS. It is only enabled in StealthChop mode, and when `TCOOLTHRS >= TSTEP > TPWMTHRS`"_. This means that stall detection is disabled at slow speeds when `TSTEP > TCOOLTHRS`, which happens when the motor is turning slowly.

The TCOOLTHRS value is used to disable _CoolStep_ at slow speeds, where it cannot work reliably. Stall detection is enabled only when the TCOOLTHRS velocity is exceeded. It is disabled when the velocity falls below TCOOLTHRS.

If the motor is close to the end stop when the movement begins, it will not be going fast enough to enable stall detection. So actual end-stop switches are still needed! But stall detection can still be used for safety during a fast movement. Especially if you are wearing a tie ;-)

For speed, use an interrupt from the DIAG output. Or poll the `IOIN.DIAG` bit regularly.


<!-- ========================================================================================== -->

<a name="otp"></a>
## One Time Programming (OTP), p26

_One Time Programming_ is only needed if the chip cannot be configured via the UART. The default configuration is programmed into the chip and is restored on power-up. 

OTP memory can be programmed only once. Bits can be programmed from `0` to `1`, but not from `1` back to a `0`.

All the TMC controller modules have the UART connection, so there is no reason to use the OTP feature. But many of the 3D Printer or CNC boards do not connect the UART pin. In this case they might use the OTP configuration to work with the product's motors, but it's hard to tell. I suspect they just use the default "all zeros" configuration.

The OTP register data which is restored on power-up can be read from the `OTP_READ` register using `getOTP_READ()`. The default (unprogrammed) value is 0x000000 (24 bits).

If you do want to do One Time Programming, enable the `programOTP()` method in the library. It is commented out for safety, in case it gets called by mistake.

> [!NOTE]
> The factory-set clock frequency tuning `FCLKTRIM` cannot (and should not) be changed, `programOTP()` ignores these bits.


<!-- ========================================================================================== -->

<a name="data-sheets"></a>
## Data Sheets and References

**All page numbers in the code and documentation (e.g. p20), refer to this TMC2209 data sheet** \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf \
https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/TMC2209_datasheet_rev1.09.pdf


This data sheet covers the TMC2202, TMC2208 and TMC2224 (all the same apart from pinouts and a few characteristics) \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf

TMC2225, this chip has not been tested, but I think it's the same \
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2225_datasheet_rev1.14.pdf

**FYSETC-TMC2209** \
This is a great description of a typical TMC2209 board \
https://github.com/FYSETC/FYSETC-TMC2209

**AN-002: Parameterization of StallGuard2™ & CoolStep™** \
The Analog Devices Application Note to help you configure the TMC chips. It's complicated. \  
https://github.com/mumanchu/mumanchu/blob/main/assets/tmc22xxdriver/an-002.html \
https://www.analog.com/en/resources/app-notes/an-002.html
<!-- https://www.analog.com/media/en/technical-documentation/app-notes/an-002.pdf -->


**Analog Devices TMC2209 Calculator** \
This is an Analog devices spreadsheet for checking the VACTUAL calculations \
https://www.analog.com/media/en/engineering-tools/design-tools/tmc2209_calculations.xlsx

**Dr. Fritz's great introduction to microstepping** \
https://www.faulhaber.com/fileadmin/Import/Media/AN015_EN.pdf \
and much more good stuff too... \
https://www.faulhaber.com/en/know-how/application-notes


<!-- ========================================================================================== -->

<a name="revision-history"></a>
# Revision History

| Date       | Version  | Description |
|:---------- |:---------|:----------- |
| 2026.04.14 | 1.0.0	| The first version! |

<br/>

**Joke of the Week** \
_What's the difference between hardware and software?_ \
_Hardware is all the bits you can hit with a hammer._

