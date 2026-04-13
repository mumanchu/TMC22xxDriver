///////////////////////////////////////////////////////////////////////////
// Example sketch for the TXM22xxDriver Library
// Copyright (C) muman.ch, 2026.04.13
// 
// For details see the library's github entry:
// https://github.com/mumanchu/TMC22xxDriver


// This enables the DEBUG code, LOGERROR(), ASSERT() etc.
// Comment this out for the Release build to reduce the code size
#define DEBUG

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// In DEBUG mode, detect and log errors
#include "MumanchuDebug.h"

#ifdef DEBUG
// if using a hardware debugger, disable all GCC compiler optimisations
// (breakpoints sometimes don't work due to optimization) 
#pragma GCC optimize ("-O0")

// Shared error logging function
void LogError(const char* msg, const char* filePath, uint line)
{
	char buf[256];
	const char* fname = strrchr(filePath, '\\');
	fname = fname ? fname + 1 : filePath;
	sprintf(buf, "ERROR: %s : %s(%u)", msg, fname, line);
	Serial.println(buf);
	Serial.flush();
}
#endif
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// The library
#include "TMC22xxDriver.h"

// TMC2209 Module Pins
#define RX2				2	// UART RX pin on Nucleo-64 (see JPR)
#define INDEX_PIN		10	// INDEX and DIAG may be swapped
#define DIAG_PIN		4	// on ZERO use D10 (no interrupts on D4)
#define DIR_PIN			5
#define STEP_PIN		6
#define ENABLE_PIN		7
#define TX2				8	// UART TX pin on Nuclep-64 (see JPR)
#define TACHO_PIN		9	// optional opto interrupter tachometer

// Arduino Zero : Serial is SerialUSB, so use Serial1 for RX/TX pins 0 and 1
// Nucleo-64 : D0/D1 (Serial) used for USB, so use Serial1 on D2 and D8
// The prototype shield has jumpers for selecting the TX/RX pins
#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(RX2, TX2);
#endif
#define tmcUart Serial1


/////////////////////////////////////////////////////////////////////
// Optional derived class so we can use a custom setConfiguration()
// (or you can modify setConfiguration() in TMC22xxDriver.h)

class MyTMC22xxDriver : public TMC22xxDriver
{
public:
	// this overrides the method in the base class
	bool setConfiguration();
};

// THIS CODE COPY/PASTED FROM TMC22xxDriver.h
// >>> MODIFY THIS ACCORDING TO YOUR DESIRED CONFIGURATION <<<

// Set the configuration by setting all writeable registers
// overrides the OTP settings
// These settings are application dependent, they are not necessarily 
// the power-up defaults.
// This also initialises the shadow register values.
bool MyTMC22xxDriver::setConfiguration()
{
	// clear the latched reset and drv_err status flags
	if (!clearGSTAT())
		return false;

	// power-up default values are 0 unless (default = n) is shown
	// (default = OTP) means it's initialized from One-Time Programmable memory

	// Global configuration flags
	GCONF gconf;
	gconf.data = 0;						// set all values to 0 (false)
	//(comment out the zero values, they're already 0)
	//gconf.i_scale_analog = false;		// 0 = use internal reference voltage, 1 = use VREF pin (default = 1)
	//gconf.internal_Rsense = false;	// 0 = use external sense resistors, 1 = internal sense resistors (default = OTP)
	//gconf.en_spreadCycle = false;		// 0 = StealthChop PWM mode, 1 = SpreadCycle mode (default = OTP)
	//gconf.shaft = false;				// 0 = normal motor direction, 1 = inverse (use this if the motor turns the wrong way)
	//gconf.index_otpw = false;			// 0 = INDEX pin shows microsteps, 1 = INDEX shows overtemperature prewarning (otpw)
	gconf.index_step = true;			// 0 = INDEX output as selected by index_otpw, 1 = INDEX shows step pulses from internal pulse generator

	gconf.pdn_disable = true;			// 1 = use UART interface, power down is disabled, 0 = PDN function
	gconf.mstep_reg_select = true;		// 1 = microstep resolution selected by MRES register, 0 = selected by MS1/MS2 pins
	gconf.multistep_filt = true;		// 1 = software pulse generator optimization enabled, 0 = no filtering of step pulses (default = 1)
	if (!setRegister(GCONF_REG, gconf.data))
		return false;

	// SENDDELAY for read access, number of bit times until reply is sent
	// bit time depends on the baud rate
	// sendDelay = 0..15, where: (sendDelay | 1) * 8 bit times
	// e.g. 0, 1   = 1 x 8 bit times
	//      2, 3   = 3 x 8 bit times
	//      4, 5   = 5 x 8 bit times
	//      ...
	//      14, 15 = 15 x 8 bit times
	NODECONF nodeConf;
	nodeConf.data = 0;				// all 0s
	nodeConf.sendDelay = 2;			// 3*8 bit times (default = 2)
	if (!setRegister(NODECONF_REG, nodeConf.data))
		return false;

	// Driver current control
	IHOLD_IRUN ihold_irun;
	ihold_irun.data = 0;			// all 0s
	ihold_irun.ihold = 3;			// standstill current (0=1/32..31=32/32) (default = OTP) 0 for freewheeling or coil short circuit (passive breaking)
	ihold_irun.irun = 5; //31;		// motor run current (default = 31 = max current according to rSense)
	ihold_irun.iholdDelay = 2;		// number of clock cycles for motor powerdown after standstill detected (stst = 1) (default = OTP)
	if (!setRegister(IHOLD_IRUN_REG, ihold_irun.data))
		return false;

	// CoolStep configuration, 'smart energy control' (default = 0x00000000)
	COOLCONF coolConf;
	coolConf.data = 0;				// all 0s
	//coolConf.semin = 0;			// minimum StallGuard value
	//coolConf.seup = 0;			// current increment steps per measured StallGuard value 
	//coolConf.semax = 0;			// StallGuard hystereis value
	//coolConf.sedn = 0;			// current down step speed
	//coolConf.seimin = 0;			// minimum current for smart current control
	if (!setRegister(COOLCONF_REG, coolConf.data))
		return false;

	// Chopper and driver configuration (reset default = 0x10000053)
	CHOPCONF chopConf;
	chopConf.data = 0;				// all 0s
	chopConf.toff = 3;				// off time and driver enable (default = 3) (0 = drivers off)
	chopConf.hstrt = 5;				// hysteresis start value added to HEND (default = 5)
	//chopConf.hend = 0;			// hysteresis low value
	//chopConf.tbl = 0;				// comparator blank time 0..3 (default = 0)
	chopConf.vsense = true;			// sense resistor voltage, 0=high voltage, 1=low voltage (default = 0)
	chopConf.mres = 8;				// microstep setting 0=256..8=1 (default = 0, 256 microsteps)
	chopConf.intpol = true;			// interpolation to 256 microsteps (default = 1) <<< true for "silent" operation
	//chopConf.dedge = false;		// STEP, 0 = rising edge, 1 = both edges
	//chopConf.diss2g = false;		// 0 = short to GND protection on, 1 = off
	//chopConf.diss2vs = false;		// 0 = Short protection low side on, 1 = off
	if (!setRegister(CHOPCONF_REG, chopConf.data))
		return false;

	// StealthChop PWM chopper configuration (reset default = 0xC10D0024)
	PWMCONF pwmConf;
	pwmConf.data = 0;				// all 0s
	pwmConf.pwm_ofs = 36;			// amplitude 0..255 (default = 36)
	pwmConf.pwm_grad = 14; 			// amplitude gradient 0..255 (default = 14)
	pwmConf.pwm_freq = 1;			// pwm frequency selection 0..3 (default = 1)
	pwmConf.pwm_autoscale = true;	// pwm automatic amplitude scaling (default = 1)
	pwmConf.pwm_autograd = true;	// pwm automatic gradient adaptation (default = 1)
	pwmConf.freewheel = 0;			// standstill mode 0..3, if i_hold==0 (default = 0)
	pwmConf.pwm_reg = 8;			// regulation loop gradient 1..15, when pwm_autoscale==1 (default = 8)
	pwmConf.pwm_lim = 12;			// PWM automatic scale amplitude limit when switching on (default = 12)
	if (!setRegister(PWMCONF_REG, pwmConf.data))
		return false;

	// Misc
	setRegister(TPOWERDOWN_REG, 20);// the delay time from stand still (stst) detection to motor current power down (default = 20)
	setRegister(TPWMTHRS_REG, 0);	// upper velocity for StealthChop voltage PWM mode, 0 = disabled (default = 0)
	setRegister(VACTUAL_REG, 0);	// 0 = normal operation, use STEP input (default = 0), >0 = move velocity
	setRegister(TCOOLTHRS_REG, 0);	// the lower threshold velocity for switching on smart energy CoolStep and StallGuard to DIAG output
	setRegister(SGTHRS_REG, 0);		// detection threshold for stall, when SG_RESULT <= (2 * SGTHRS), see p59 'Tuning StallGuard4'
									// TCOOLTHRS >= TSTEP > TPWMTHRS

	// check the chip's "interface counter" matches ours
	if (!checkIFCNT())
		return false;

	// blimey! it worked!
	return true;
}

MyTMC22xxDriver tmcDriver;


/////////////////////////////////////////////////////////////////////
// Interrupt driven counters/timers for tachometer and INDEX pulses

#include "Tacho.h"

// the rotation tachometer
Tacho tachometer;
void tachometerInterruptHandler() {
	tachometer.interruptHandler();
}

// tacho1 is the INDEX pulse counter
Tacho indexPulse;
void indexPulseInterruptHandler() {
	indexPulse.interruptHandler();
}


/////////////////////////////////////////////////////////////////////
// Startup Code

void setup()
{
	Serial.begin(115200);
	delay(5000);		// delay to give time to open the Serial Monitor

	Serial.println("\n\rStarted...\n\r");
	Serial.flush();

	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(DIAG_PIN, INPUT);
	tachometer.begin(TACHO_PIN, tachometerInterruptHandler);
	indexPulse.begin(INDEX_PIN, indexPulseInterruptHandler, CHANGE);

	tmcUart.begin(384000);
	bool ok = tmcDriver.begin(&tmcUart, 0, ENABLE_PIN);
	if (!ok) {
		LOGERROR("tmcDriver.begin() failed");
		while (1)
			yield();
	}
	tmcDriver.enableMotorDrivers();
}


/////////////////////////////////////////////////////////////////////
// Continuous Loop

void loop()
{
	// 100ms scheduler
	static ulong t1 = 0;
	ulong t = millis();
	if ((t - t1) >= 100) {
		t1 = t;

		// toggle the led
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

		// accelerate/decelerate the motor using velocity control
		// via the TMC chip's internal step pulse generator 
		const float maxRotationsPerSecond = 4.5f;
		const float acceleration = 0.1f;

		static float rotationsPerSecond = 0.0f;
		static bool direction = true;
		static bool accelerating = true;

		if (accelerating)
			rotationsPerSecond += acceleration;
		else
			rotationsPerSecond -= acceleration;

		// if stopped, reverse direction and start accelerating
		if (rotationsPerSecond <= 0.0f) {
			rotationsPerSecond = 0.0f;
			direction = !direction;
			accelerating = true;

			// show tachometer and index pulses
			char buf[100];
			sprintf(buf, "tacho=%lu", tachometer.getCountAndReset());
			Serial.println(buf);
			sprintf(buf, "index=%lu", indexPulse.getCountAndReset());
			Serial.println(buf);
		}

		// if at maximum speed, start decelerating
		if (rotationsPerSecond >= maxRotationsPerSecond) {
			rotationsPerSecond = maxRotationsPerSecond;
			accelerating = false;
		}

		// velocity control
		long vactual = tmcDriver.rpsToVACTUAL(rotationsPerSecond);
		if (direction)
			vactual = -vactual;
		tmcDriver.velocityMoveStart(vactual);
	}
}

