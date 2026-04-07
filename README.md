# TMC22xxDriver Library
Advanced TCM22xx Stepper Motor Controller Library, from $${\color{green}mumanchu}$$

This library is for configuring and monitoring the TRINAMIC TMC22xx range of intelligent Stepper Motor Controller chips via the UART RX/TX interface. This includes the TMC2202/2208/2209 and TMC2224/2225/2226. It runs on all STM32, SAMD, AVR, ESP32 and ESP8266 boards.

The library does not do the STEP/DIR control. That is done by a separate `MiniStepper` library (to be released soon). `MiniStepper` works together with this library or as a stand-alone library for controlling all standard motor controller chips via the STEP/DIR/EN pins. It is a non-blocking library which also provides S-curve acceleration and deceleration.

<!-- ========================================================================================== -->

## Contents
- [Introduction](#introduction)
- [Advantages of this Library](#advantages)
- [UART RX/TX Interface](#uart)
- [Test Circuit](#test-circuit)
- [Silent Running](#silent-running)
- [Velocity Control](#velocity-control)
- [STEP/DIR Control](#step-dir-control)
- [Microstepping](#microstepping)
- [Stall Detection and the DIAG Pin](#stall-detection)
- [One Time Programming (OTP)](#otp)
- [Library API](#api)
- [Example Sketch](#example-sketch)
- [Notes About the Code](#notes)
- [Commissioning and RMS Current Calculations](#commissioning)
- [TMC22xx Data Sheets](#data-sheets)


<!-- ========================================================================================== -->

<a name="introuction"></a>
## Introduction

Some of the best stepper motor controller chips were developed by 'TRINAMIC Motion Control Gmbh', which has now been absorbed by 'Analog Devices Inc' to become the 'ADI Trinamic™' range. These intelligent TMC chips have a fast serial interface (UART up to 500K bits-per-second) for configuration and monitoring via multiple internal registers. They also have the standard STEP, DIR and ENABLE signals for classic stepper control.

The Trinamic range has several unique features: _StealthChop™_ and _MicroPlyer™_ for silent motor operation; _SpreadCycle™_ for dynamic motor control via a voltage chopper; _StallGuard™_ for current load and stall detection; and _CoolStep™_ for current control with up to 75% energy savings. You can find out all about these features from the data sheet.

I think the TMC2202/8/9 and TMC2224/5/6 are all the same apart from the packaging, pinouts, voltage and current ratings. There is a TMC2130 which is also the same but uses SPI communications instead for UART. (SPI could easily be handled by changing `setRegister()` and `getRegister()` to use SPI instead of UART, maybe I'll add that later...)

I have several 3D printer controller boards that use Trinamic chips, and they _all_ use the TMC2209. I've not seen a board that uses the other versions. Some are advertised, but does anybody buy them?

<!-- If you need to learn about stepper motors, check out this voluminous blog entry:
https://muman.ch/muman/index.htm?muman-stepper-motor-control.htm -->

<!-- I'm hoping the next generation of controllers will handle all the acceleration/deceleration internally, so you just select the curve and write a number of steps into a register and it will do the movement itself, signalling when the movement is complete. Then you won't need the `MiniStepper` library :-) -->

<!-- ========================================================================================== -->

<a name="advantages"></a>
## Advantages of this Library 

There are already several TMC2209 libraries for Arduinos. So just to confuse you, I've released another one.

This one is part of a Stepper Motor Control Library suite which is being released in stages. `MultiTimerSAMD` and `OptimizedGPIO` are already available. Next will be the `MiniStepper` library for STEP/DIR control, followed by a multiplatform Timer library for STM32, AVR, ESP32, AVR and ESP8266.

In addition:
- This library supports _all_ the TMC chip's features
- Has useful comments in the source code to help understand each method, although you'll still have to read the data sheet :-( 
- Detects and reports communications errors, with an error count. This is useful in a "noisy" environment when high currents are being switched. 
- Also uses the chip's received message counter `IFCOUNT` to validate communications 
- Fast look-up table CRC calculation and byte reversal using __asm__ instructions
- Provides commissioning routines for RMS current calculations based on algorithms in the data sheet
- Works together with the `MiniStepper` library, and should work with any other STEP/DIR library
- Supports [One Time Programming (OTP)](#otp), but it's patched out for safety because it can be programmed only once
- In `DEBUG` mode
	- detects writes to read-only registers and reads of write-only registers
	- read-after-write validation of read/write registers
	- validates all TX data that's echoed back to RX, see [UART RX/TX Interface](#uart) below


<!-- ========================================================================================== -->

<a name="uart"></a>
## UART RX/TX Interface

Serial one-wire UART communications

The 22xx chips have a dual-purpose pin called PDN_UART. This can either work as a power-down input (for standstill current reduction) or as a fast serial TX/RX port for intelligent control. The mode selection is done via the pdn_disable bit in the GCONF configuration register. At start-up when pulled high it works as a UART pin, so you can program this bit to 0 to disable the PDN feature (use setDefaultConfiguration() in the Mattlabs' TMC2209 Driver Library below).

Because there is only one pin for both TX and RX, it's necessary to use an external 1K ohm resistor to stop the MCU's TX driver affecting communications. Many of the stand-alone driver boards have this resistor already fitted. 


<!--image here-->



Some the 3D printer main boards (even the Qidi) do not have the RX connection, so the TMC2209 registers cannot be read - they are all 'write-only'.
Because the MCU's TX is effectively looped back to the RX pin via the 1K resistor, the MCU will receive all the data that it transmits (if the RX pin is connected). This has the advantage that the MCU can validate that the data received by the UART pin was correct and there was no noise on the line that disrupted communications. The Mattlabs library does this check (in DEBUG mode), see setRegister(). 

All sent and received messages have a 1-byte polynomial CRC which will detect most communications errors.
The port runs at up to 500000 baud, automatically adjusting to the baud rate of the received message. Running at a lower speed (say 256000) might be better for noise immunity, especially if you cannot read from the chip to check that everything is working.
Separate power supply pins for I/O (VCC) and the motor (VS)


<!-- ========================================================================================== -->

<a name="test-circuit"></a>
## Test Circuit

<!-- circuit diagram -->


Notes
(1) The INDEX and DIAG outputs could be connected, see data sheet
(2) Serial port pins may be different, or you can use SoftwareSerial
(3) Use 5V for a 5V MCU, or 3.3V for a 3.3V MCU, else "Bang!", see Disclaimer.




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

"Silent Running" was a brilliant 1972 science fiction movie starring Bruce Dern (https://en.wikipedia.org/wiki/Silent_Running). But that's not what this refers to, because they didn't use TMC chips in the movie (or maybe they did, the robots were very quiet).

The TMC chips have a feature called _MicroPlyer™_, which breaks up a single STEP pulse into 256 microsteps, making the motor run very smoothly and quietly. It does this without microsteps being configured, so it works with the standard fullstep STEP control. So if you do one step, the chip actually does 256 microsteps to make the movement smooth. 

This is what they say in the data sheet:
_"The MicroPlyer™ step pulse interpolator brings the smooth motor operation of high-resolution microstepping to applications originally designed for coarser stepping."_

Even if you are using microstepping (via `CHOPCONF::MRES`), the chip will still extrapolate the movement to 256 microsteps. 

_MicroPlyer™_ can be disabled with the `CHOPCONF::intpol` bit. If you do this, the motor will get noticeably louder and will vibrate far more, especially at the motor's resonant frequency.

You could probably do something similar with any controller chip by selecting a high microstep setting and always stepping to the exact fullstep position in microsteps. For example, if 32 microsteps is selected, always do 32 steps for each full step, timing each step to take 1/32 of the full step rate for the desired rotation speed (this probably requires PWM control). See [Microstepping](#microstepping) below.


<!-- ========================================================================================== -->

<a name="velocity-control"></a>
## Velocity Control

This library does not control the motor with the STEP/DIR pins, but it _can_ use the TMC's 'internal step pulse generator' to run the motor forwards or backwards at a particular velocity, by setting the `VACTUAL` register. This makes it run like a normal speed-controlled motor. You could use the `INDEX` pulse output to count the number of steps.

It does not do any acceleration or deceleration, so you must do that yourself. _"Motion at higher velocities will require ramping up and ramping down the velocity value by software."_ 

The velocity is a 24-bit signed value `microsteps / t`, with the direction controlled by the sign (+ve or -ve) of the value.

This library has two methods for velocity control:
```cpp
	// velocity := -8'388'608 .. 8'388'607 (0xff800000 .. 0x007fffff)
	//             signed 24-bit velocity in 'microSteps / t'
	// Direction is controlled by the sign (+ve or -ve).
	bool velocityMoveStart(long velocity);
	bool velocityMoveStop();
```

<!-- ========================================================================================== -->

<a name="step-dir-control"></a>
## STEP/DIR Control

The TMC chips have the traditional STEP, DIR and EN stepper motor control signals. These are not handled by this library.

The code to do the STEP/DIR control has been put into a separate `MiniStepper` library. This is because the `MiniStepper` library can be used with _any_ motor controller chip, not just the intelligent TMC22xx chips. This is a sophisticated 'non-blocking' library that uses timers and PWM, has S-curve acceleration/deceleration, and controls up to 4 motors simultaneously.

The `MiniStepper` library will be released soon, but until then you can use any of the other stepper libraries. Note that some 'non blocking' libraries rely on a call from `loop()`, so the motor stops while the loop is doing something else. You won't have that problem with `MiniStepper` because the entire movement is controlled by interrupts.




TODO



<!-- ========================================================================================== -->

<a name="microstepping"></a>
## Microstepping

This often causes some confusion...


Controller chips select the microstep setting either via two or three input pins, or by writing the microstep setting to a register.

MS0 MS1 MS2


Most steppers have 200 steps-per-turn, which is 1.8 degrees-per-step. Some have 0.9 degree steps, making 400 steps-per-turn. 1.8 degrees (or 0.9) may not provide the resolution you need. In this case you could use microsteps to divide the full step by 2, 4, 8, 16, 32, 64, 128 or max. 256. 

Each STEP pulse will move the motor by the microstep fraction of the full step rotation, by supplying varying voltages to the two coils. This can provide more accurate steps. 

BUT the torque decreases as the number of microsteps increases. At 16 microsteps, the torque per microstep is less than 10% of the fullstep torque. At 32 it's 5% and at 256 it is 0.61%! This means that the motor may not actually move until several microsteps have been made, enough to overcome the load's torque.

_"The consequence is that if the load torque plus the motor’s friction and detent torque is greater than the incremental torque of a microstep, successive microsteps will have to be realized until the accumulated torque exceeds the load torque plus the motor’s friction and detent torque. Simply stated, taking a microstep does not mean the motor will actually move!"_
https://www.faulhaber.com/fileadmin/Import/Media/AN015_EN.pdf

This means that it's usually better to gear down the movement using a lead screw or pulley ratio and use fullsteps instead of microsteps.
However, microsteps are very useful for smooth running, especially if you always step until the full step position is reached. In fact, the Trinamic TMC series controllers do this automatically, dividing each full step into 256 microsteps. This is called _MicroPlyer™_ and it's what makes these controllers "silent". 

TODO microsteps and microPlyer, how does it work



Another disadvantage of microstepping is that the motor may move to the next (or nearest) fullstep position if power is cut to the drivers, so the actual microstep position is lost. The TRINAMIC controllers have a register which holds the microstep position, so you could write some clever software to restore the microstep position when power returns. This is described in the TMC2209 data sheet '3.6.1 Restart the Stepper Motor Without Position Loss', p15. 


<!-- ========================================================================================== -->


<a name="stall-detection"></a>
## Stall Detection and the DIAG Pin

The TMC chips measure the current through the motor's two coils. They are able to use this to detect if the motor has "stalled" because the current increases if the motor is unable to turn.

In theory, you could use this instead of having end-stop switches. But I spent a lot of time playing with this, and it does not seem to work well at very low speeds. So switches are still needed. But stall detection could still be used in case the switch fails.

TODO configuration

TODO diag pin interrupt
diag output was not cleared... ?

TODO DIAG bit can be polled 



<!-- ========================================================================================== -->

<a name="otp"></a>
## One Time Programming (OTP), p26

One Time Programming is only needed if the chip cannot be configured via the UART. If the UART is not connected, then the default configuration can be programmed into the chip, to be restored on power-up. 

OTP memory can be programmed only once. Bits can be programmed to a `1`, but not back to a `0`.

All the TMC Controller boards have the UART connection, so there is no reason to use the OTP feature. Some 3D Printer or CNC boards have not connected the UART pin, in this case they may use the OTP configuration to work with the product's motors, but it's hard to tell.

The OTP register data which is restored on power-up can be read from the `OTP_READ` register using `getOTP_READ()`. The default (unprogrammed) value is 0x000000 (24 bits).

If you do want to do One Time Programming, enable the `programOTP()` method in the library. This is patched out with `$if 0` for safety, in case it is called by mistake.

***The factory-set clock frequency tuning `FCLKTRIM` cannot (and should not) be changed!***


<!-- ========================================================================================== -->

<a name="api"></a>
## Library API

The library contains many methods. Only the main methods (the first three) are described here. Refer to the source code and data sheet for detailed descriptions of the others.

```cpp
class TMC22xxDriver
{
	TMC22xxDriver(uint address, uint enablePin);
	bool begin(Stream* uart);
	// Override this in a derived class to define your own configuration
	// or just edit the code in this file
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
	bool getMicrosteps(uint* microSteps, bool forceRead = false);
	bool getMicrostepRegisters(uint* mscnt, int* cur_a, int* cur_b);

	bool setDriverCurrent(uint ihold, uint irun, uint iholdDelay);
	bool setCHOPCONF(CHOPCONF chopConf);
	bool setPWMCONF(PWMCONF pwmConf);
	bool setCOOLCONF(COOLCONF coolConf);
	bool setTCOOLTHRS(uint tcoolthrs);
	bool setTPWMTHRS(uint tpwmthrs);
	bool setSGTHRS(uint sgthrs);
	bool setTPOWERDOWN(uint tpowerdown);

	bool velocityMoveStart(long velocity);
	bool velocityMoveStop();

	bool setRegister(uint reg, uint32_t data);
	bool getRegister(uint reg, uint32_t* data);

	// These are only needed for commissioning, they could be commented out
	bool getSettingsFromRmsCurrent(uint iRunCurrentRmsMilliamps, uint iHoldCurrentRmsMilliamps,
		float rsense, bool vsense, uint* irun, uint* ihold);
	bool getRmsCurrentFromSettings(uint irun, uint ihold, float rsense, bool vsense,
		uint* iRunCurrentRmsMilliamps, uint* iHoldCurrentRmsMilliamps);

	// One Time Programming (OTP), patched out unless needed
	//bool programOTP(OTP_READ otpData);
}		
```

**`TMC22xxDriver(uint address, uint enablePin);`** \
This is the constructor. `address` is the stepper motor number 0..3. `enablePin1` is the pin number of the chip's `EN` pin. This is the same `EN` pin that you would use with the `MiniStepper` library. 

**`bool begin(Stream* uart);`** \
Once the serial port has been opened, call this from `setup()` to connect it to the TMC chip. It returns `false` if something failed. Always check the return value. \
```cpp
	Serial1.begin(384000);
	if (!tmcDriver1.begin(&Serial1)) {
		LOGERROR("tmcDriver1.begin() failed");
		while (1)
			yield();
	}
```

**`bool setConfiguration();`** \
This function is called to configure the TMC chip's registers. You can either edit the library code for your desired configuration, or override it in a derived class as shown in the [Example Sketch](#example-sketch). You would normally use the same configuration for each stepper motor. If not, use the override method.


<!-- ========================================================================================== -->

<a name="example-sketch"></a>
## Example Sketch

Chip address 0..3 from MS1/MS2 wiring



<!-- ========================================================================================== -->

<a name="notes"></a>
## Notes About the Code

The library is in a single include file that contains both the class definition and the code. This is fine if you want to use it in only one source file (.cpp or .ino) - which is recommended. But it won't work if you want to access the class from more than one source file. In that case, put the code in a separate '.cpp' file to be linked only once, keeping the class definition in the include file.

For a bit of variety, 'get' and 'set' are used instead of 'read' and 'write', e.g. `getRegister()` and `setRegister()`. This is like C#'s getters and setters.

The code assumes `byte` and `char` is a byte (8 bits), `int` is 16 **OR** 32 bits (the library code works for both), `short` is always 16 bits, and `long` is always 32 bits. This is true for all MCUs. Because of this, `int8_t`, `int16_t` or `int32_t` (etc.) are not used unless it is to emphasize the number of bits.

`uint` and `ulong` are also used. You may have to `typedef` those if they are not defined for your compiler: \
```cpp
typedef unsigned char byte;
typedef unsigned int uint;
typedef unsigned long ulong;
```

The contents of the TMC22xx chip's registers are defined with `struct` and `union`, which gives names to each bit or group of bits. Each register can be handled either as a 32-bit value `data` or as individual items. The code uses the same names as the data sheet, except all lower case for content, and all upper case for the register name (the data sheet is inconsistent). Refer to the data sheet for descriptions of the registers and data. A data sheet page number, e.g. 'p12' is given for each register. 
 
To save reading a register before modifying it, the library uses _shadow registers_ to hold the value last written to the register. This is managed automatically by the `setRegister()` code.

For the STM32, assembly language instructions `__asm__` are used to speed up CRC calculation and byte reversal in `crc8()` and `reverse4bytes()`. If not using an STM32, then normal C code is used. (But this will be updated in a future release.)

If `DEBUG` is defined, the code does additional checks to ensure read-only registers are not written to and write-only registers are not read. It can also verify writes to readable registers by doing a write-read-and-compare, but only if the chip's TX pin is connected.

If both RX and TX are connected, it will also validate the loopback message, since everything sent is also received (and usually discarded). This detects bad connections or noise on the TX/RX lines. If RX is not connected, patch out the `TMC22xxDRIVER_HAS_ECHO` definition to disable this test.


TODO choose a better name for `TMC22xxDRIVER_HAS_ECHO`




<!-- ========================================================================================== -->

<a name="commissioning"></a>
## Commissioning and RMS Current Calculations







<!-- ========================================================================================== -->

<a name="data-sheets"></a>
## TMC22xx Data Sheets

All page numbers mentioned in the code (e.g. p20), refer to _this_ TMC2209 data sheet \
[https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf)

TODO store copies in ASSETS


This data sheet covers the TMC2202, TMC2208 and TMC2224 (all the same apart from the pinouts) \
[https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2202_TMC2208_TMC2224_datasheet_rev1.14.pdf)

TMC2225, this chip has not been tested, but I think it's the same \
[https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2225_datasheet_rev1.14.pdf](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2225_datasheet_rev1.14.pdf)

**FYSETC-TMC2209** \
This is a great description of a typical TMC2209 board \
[https://github.com/FYSETC/FYSETC-TMC2209](https://github.com/FYSETC/FYSETC-TMC2209)

