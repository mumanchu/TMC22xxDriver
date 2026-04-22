#pragma once

///////////////////////////////////////////////////////////////////////////////
// TMC22xx Stepper Motor Controller Library
// Copyright (C) muman.ch, 2026.04.22
// info@muman.ch
/*
For full details see github
https://github.com/mumanchu/TMC22xxDriver
or here
https://muman.ch/muman/index.htm?muman-matts-blog.htm

All page numbers (e.g. p20), refer to this data sheet
https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
*/

#include <Arduino.h>

#ifndef PNUM_NOT_DEFINED
#define PNUM_NOT_DEFINED (-1)
#endif

#ifndef digitalPinIsValid
#define digitalPinIsValid(pin) ((uint)pin < NUM_DIGITAL_PINS)
#endif

#ifdef DEBUG
// Comment this out if the TMC's TX pin is not connected to the MCU's RX
#define TMC22xxDRIVER_LOOPBACK
#endif


class TMC22xxDriver 
{
protected:
	// for TMC2209 UART communications
	Stream* uart = NULL;
	// number of communications errors, see 'uint getErrorCount(bool clearErrorCount)'
	uint errorCount = 0;
	// number of successful write messages, compare this with ifcnt
	uint expectedifcnt = 0;
	// EN pin, active low to enable the drivers
	uint enablePin = PNUM_NOT_DEFINED;

	uint fullstepsPerRevolution = 200;
	uint currentMicrosteps = 1;			// 1 2 4 8 16 32 64 128 256

	// 2^24 / fclk, for VACTUAL calculations
	const float t1 = 16777216.0f / 12000000.0f;

public:
	// device address 0..3, set by AD0/AD1 pins (MS1/MS2)
	uint address = 0;
	// 'this' pointers for each driver, accessed by [address], NULL if chip is not present
	static TMC22xxDriver* driverPtrs[4];


	///////////////////////////////////////////////////////////////////////////////
	// Register Numbers, p23

	// General registers
	#define GCONF_REG       0x00
	#define GSTAT_REG       0x01
	#define IFCNT_REG       0x02
	#define NODECONF_REG    0x03
	#define OTP_PROG_REG    0x04
	#define OTP_READ_REG    0x05
	#define IOIN_REG        0x06
	#define FACTORY_CONF_REG 0x07
	// Velocity dependent control
	#define IHOLD_IRUN_REG  0x10
	#define TPOWERDOWN_REG  0x11
	#define TSTEP_REG       0x12
	#define TPWMTHRS_REG    0x13
	#define VACTUAL_REG     0x22
	// StallGuard control
	#define TCOOLTHRS_REG   0x14
	#define SGTHRS_REG      0x40
	#define SG_RESULT_REG   0x41
	#define COOLCONF_REG    0x42
	// Sequencer registers
	#define MSCNT_REG       0x6a
	#define MSCURACT_REG    0x6b
	// Chopper control registers
	#define CHOPCONF_REG    0x6c
	#define DRV_STATUS_REG  0x6f
	#define PWMCONF_REG     0x70
	#define PWM_SCALE_REG   0x71
	#define PWM_AUTO_REG    0x72


	///////////////////////////////////////////////////////////////////////////////
	// Register Bit Definitions
	// this uses the same names as the data sheet, 
	// but all lower case (or camel case) to be consistent

	// General registers, p23

	union GCONF {
		struct {		// (anonymous structure)
			bool i_scale_analog : 1;
			bool internal_Rsense : 1;
			bool en_spreadCycle : 1;
			bool shaft : 1;
			bool index_otpw : 1;
			bool index_step : 1;
			bool pdn_disable : 1;
			bool mstep_reg_select : 1;
			bool multistep_filt : 1;
			//bool test_mode : 1;	// not used, always 0
		};
		uint32_t data;
	};

	union GSTAT {
		struct {
			bool reset : 1;
			bool drv_err : 1;
			bool uv_cp : 1;
		};
		uint32_t data;
	};

	union IFCNT {
		struct {
			uint ifcnt : 8;
		};
		uint32_t data;
	};

	union NODECONF {
		struct {
			uint _0 : 8;
			uint sendDelay : 4;
		};
		uint32_t data;
	};

	union OTP_PROG {
		struct {
			uint otpBit : 3;
			bool _0 : 1;
			uint otpByte : 2;
			uint _1 : 2;
			uint otpMagic : 8;
		};
		uint32_t data;
	};

	// The OTP_READ register has two unions
	//   OTP_READ    for otp_en_spreadcycle = 0
	//   OTP_READ_1  for otp_en_spreadcycle = 1
	// cast the value according to the 'otp_en_spreadcycle' bit
	// (nested unions/structs do not seem to work)

	// Use this when otp_en_spreadcycle = 0 (StealthChop), p26
	union OTP_READ {
		struct {
			uint otp_fclkTrim : 5;
			bool otp_otTrim: 1;
			bool otp_internal_Rsense : 1;
			bool otp_tbl : 1;
			//>>> if otp_en_spreadcycle = 0
			uint otp_chopconf : 9;
			//<<<
			bool otp_pwm_reg : 1;
			bool otp_pwm_freq : 1;
			uint otp_iholdDelay : 2;
			uint otp_ihold : 2;
			// 0=StealthChop, 1=SpreadCycle -> GCONF.en_spreadCycle
			uint otp_en_spreadcycle : 1;
		};
		uint32_t data;
	};

	// Use this when otp_en_spreadcycle = 1, p26
	union OTP_READ_1 {
		struct {
			uint otp_fclkTrim : 5;
			bool otp_otTrim : 1;
			bool otp_internal_Rsense : 1;
			bool otp_tbl : 1;
			//>>> if otp_en_spreadcycle = 1
			uint otp_pwm_grad : 4;
			bool otp_pwm_autograd : 1;
			uint otp_tpwmthrs : 3;
			bool otp_pwm_ofs : 1;
			//<<<
			bool otp_pwm_reg : 1;
			bool otp_pwm_freq : 1;
			uint otp_iholdDelay : 2;
			uint otp_ihold : 2;
			// 0=StealthChop, 1=SpreadCycle -> GCONF.en_spreadCycle
			uint otp_en_spreadcycle : 1;
		};
		uint32_t data;
	};

	// p24
	union IOIN {
		struct {
			bool enn : 1;
			bool _0 : 1;
			bool ms1 : 1;
			bool ms2 : 1;
			bool diag : 1;
			bool _1 : 1;
			bool pdn_serial : 1;
			bool step : 1;
			bool spread_en : 1;
			bool dir : 1;
			uint _2 : 14;
			uint version : 8;
		};
		uint32_t data;
	};

	union FACTORY_CONF {
		struct {
			uint fclkTrim : 5;
			uint otTrim : 2;
		};
		uint32_t data;
	};

	// Velocity dependent control, p28

	union IHOLD_IRUN {
		struct {
			uint ihold : 5;
			uint irun : 5;
			uint iholdDelay : 4;
		};
		uint32_t data;
	};

	union TPOWERDOWN {
		struct {
			uint tpowerDown : 8;
		};
		uint32_t data;
	};

	union TSTEP {
		struct {
			uint32_t tstep : 20;
		};
		uint32_t data;
	};

	union TPWMTHRS {
		struct {
			uint32_t tpwmthrs : 20;
		};
		uint32_t data;
	};

	union VACTUAL {
		struct {
			int32_t vactual : 24;
		};
		uint32_t data;
	};

	// StallGuard control, p29

	union TCOOLTHRS {
		struct {
			uint32_t tcoolthrs : 20;
		};
		uint32_t data;
	};

	union SGTHRS {
		struct {
			uint sgthrs : 8;
		};
		uint32_t data;
	};

	union SG_RESULT {
		struct {
			uint sg_result : 10;
		};
		uint32_t data;
	};

	// CoolStep configuration, p30
	union COOLCONF {
		struct {
			uint semin : 4;
			bool _0 : 1;
			uint seup : 2;
			bool _1 : 1;
			uint semax : 4;
			bool _2 : 1;
			uint sedn : 2;
			bool seimin : 1;
		};
		uint32_t data;
	};

	// Sequencer registers, p31

	union MSCNT {
		struct {
			uint mscnt : 10;
		};
		uint32_t data;
	};

	union MSCURACT {
		struct {
			int cur_b : 9;
			int cur_a : 9;
		};
		uint32_t data;
	};

	// Chopper control registers, p32

	union PWM_SCALE {
		struct {
			uint pwm_scale_sum : 8;
			uint _0 : 8;
			int pwm_scale_auto : 9;
		};
		uint32_t data;
	};

	union PWM_AUTO {
		struct {
			uint pwm_ofs_auto : 8;
			uint _0 : 8;
			uint pwm_grad_auto : 8;
		};
		uint32_t data;
	};

	// p33
	union CHOPCONF {
		struct {
			uint toff : 4;
			uint hstrt : 3;
			uint hend : 4;
			uint _0 : 4;
			uint tbl : 2;
			bool vsense : 1;
			uint _1 : 6;
			uint mres : 4;
			bool intpol : 1;
			bool dedge : 1;
			bool diss2g : 1;
			bool diss2vs : 1;
		};
		uint32_t data;
	};

	// p35
	union PWMCONF {
		struct {
			uint pwm_ofs : 8;
			uint pwm_grad : 8;
			uint pwm_freq : 2;
			bool pwm_autoscale : 1;
			bool pwm_autograd : 1;
			uint freewheel : 2;
			uint _0 : 2;
			uint pwm_reg : 4;
			uint pwm_lim : 4;
		};
		uint32_t data;
	};

	// Driver status, p37

	union DRV_STATUS {
		struct {
			bool otpw : 1;
			bool ot : 1;
			bool s2ga : 1;
			bool s2gb : 1;
			bool s2vsa : 1;
			bool s2vsb : 1;
			bool ola : 1;
			bool olb : 1;
			bool t120 : 1;
			bool t143 : 1;
			bool t150 : 1;
			bool t157 : 1;
			uint _0 : 4;
			uint cs_actual : 5;
			uint _1 : 9;
			bool stealth : 1;
			bool stst : 1;
		};
		uint32_t data;
	};

protected:
	// Shadow Registers, maintained by setRegister()
	// these are only needed if you want to change individual settings
	// instead of writing the entire register 
	// (IHOLD_IRUN and COOLCONF cannot be read)
	// see also readShadowRegisters()
	GCONF gconf_buf;
	IHOLD_IRUN ihold_irun_buf;	// write only! cannot be read
	COOLCONF coolConf_buf;		// write only! cannot be read
	CHOPCONF chopConf_buf;
	PWMCONF pwmConf_buf;
	uint toff_buf;				// original toff value, saved if set to 0

public:
	// Detailed descriptions of each method are with the code below
	
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
	bool getTSTEP(ulong* tstep);
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
	long vactualToTSTEP(long vactual);

	bool setRegister(uint reg, uint32_t data);
	bool getRegister(uint reg, uint32_t* data);

	// These are only needed for commissioning, they could be commented out
	bool getSettingsFromRmsCurrent(uint iRunCurrentRmsMilliamps, uint iHoldCurrentRmsMilliamps,
		float rsense, bool vsense, uint* irun, uint* ihold);
	bool getRmsCurrentFromSettings(uint irun, uint ihold, float rsense, bool vsense,
		uint* iRunCurrentRmsMilliamps, uint* iHoldCurrentRmsMilliamps);

	// One Time Programming (OTP), commented out unless needed
	bool programOTP(OTP_READ otpData);

protected:
	uint calculateCS(float irms, float Vfs, float rsense);
	float calculateIrms(uint CS, float Vfs, float rsense);
	byte crc8(const byte* msg, uint length);
	static const byte crcTable[256];
	uint32_t reverse4Bytes(const byte* data);
};


// The following code could go in TMC22xxDriver.cpp >>>>


// Static array of 'this' pointers, one for each driver, accessed by [address]
TMC22xxDriver* TMC22xxDriver::driverPtrs[4] = { NULL, NULL, NULL, NULL };


// Call this after the UART has been initialised
// uart      : pointer to HardwareSerial or SoftwareSerial object, 
//             max. baud rate is 500000, 384000 recommended for HardwareSerial
// address   : 0..3, the UART communications address, set by AD0 and AD1
// enablePin : the pin which connects to the EN signal of the TMC chip
bool TMC22xxDriver::begin(Stream* uart, uint address, uint enablePin,
	uint fullStepsPerRevolution)
{
	ASSERT(address < 4 && digitalPinIsValid(enablePin) && fullstepsPerRevolution >= 200);
	ASSERT(driverPtrs[address] == NULL);

	this->uart = uart;
	this->address = address;
	this->enablePin = enablePin;
	this->fullstepsPerRevolution = fullstepsPerRevolution;

	// 'this' pointers, accessed by [address] 0..3
	driverPtrs[address] = this;

	// EN pin, output driver enable
	pinMode(enablePin, OUTPUT);		// (pin has 20K pullup on my board)
	enableMotorDrivers(false);

	// counts number of write messages received, 0..255 then rolls over
	uint ifcnt;
	if (!getIFCNT(&ifcnt))
		return false;
	expectedifcnt = ifcnt;

	// configure the settings (custom or factory default)
	return setConfiguration();
}

// Set the configuration by setting all writeable registers
// Overrides the OTP settings
// These settings are application dependent, they are not necessarily the 
// power-up defaults.
// This also initialises the shadow register values.
// Modify this according to your configurations, or create a derived class 
// and override setConfguration() in the derived class, see README for 
// details.
bool TMC22xxDriver::setConfiguration()
{
	// clear the latched reset and drv_err status flags
	if (!clearGSTAT())
		return false;

	// power-up default values are 0 unless (default = n) is shown
	// (default = OTP) means it's initialized from One-Time Programmable memory

	// Global configuration flags
	GCONF gconf;
	gconf.data = 0;						// set all values to 0 (false)
	//(comment out the zero values, they're already 0) (but the optimiser will do that anyway)
	//gconf.i_scale_analog = false;		// 0 = use internal reference voltage, 1 = use VREF pin (default = 1)
	//gconf.internal_Rsense = false;	// 0 = use external sense resistors, 1 = internal sense resistors (default = OTP)
	//gconf.en_spreadCycle = false;		// 0 = StealthChop PWM mode, 1 = SpreadCycle mode (default = OTP)
	//gconf.shaft = false;				// 0 = normal motor direction, 1 = inverse (use this if the motor turns the wrong way)
	//gconf.index_otpw = false;			// 0 = INDEX pin shows microsteps, 1 = INDEX shows overtemperature prewarning (otpw)
	//gconf.index_step = false;			// 0 = INDEX output as selected by index_otpw, 1 = INDEX shows step pulses from internal pulse generator
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
	ihold_irun.ihold = 5;			// standstill current (0=1/32..31=32/32) (default = OTP) 0 for freewheeling or coil short circuit (passive breaking)
	ihold_irun.irun = 16; //31;		// motor run current (default = 31 = max current according to rSense)
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
	chopConf.intpol = true;			// interpolation to 256 microsteps (default = 1) <<< ALWAYS true FOR SILENT OPERATION!
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

// Check the TMC2209 chip version 0x21 is responding
inline bool TMC22xxDriver::isConnected()
{
	IOIN ioin;
	return getIOIN(&ioin) && ioin.version == 0x21;
}

// Returns the number of communication errors detected by the library
// this should normally be 0, but errors can occur in noisy environments 
// or if baud rate is too high/too low, etc
// see also checkIFCNT() to validate the count maintained by the chip
inline uint TMC22xxDriver::getErrorCount(bool clearErrorCount)
{
	uint cnt = errorCount;
	if (clearErrorCount)
		errorCount = 0;
	return cnt;
}

// Check the chip's 'write message' counter is as expected,
// the error counter (expectedifcnt) is reset when it is read
bool TMC22xxDriver::checkIFCNT()
{
	uint ifcnt;
	if (!getIFCNT(&ifcnt))
		return false;
	if (ifcnt != expectedifcnt) {
		LOGERROR("invalid ifcnt");
		// auto reset
		expectedifcnt = ifcnt;
		return false;
	}
	return true;
}

// Enable/disable motor drivers with EN pin
bool TMC22xxDriver::enableMotorDrivers(bool enable) 
{ 
	ASSERT(digitalPinIsValid(enablePin));

	// EN = high to disable, low to enable
	digitalWrite(enablePin, !enable);
	return true;
}

// Enable motor drivers using the 'toff' value in the CHOPCONF register
// toff time/driver enable := 0 for all bridges off
bool TMC22xxDriver::enableMotorDriversWithToff(bool enable)
{
	if (enable) {
		// restore the saved toff value
		// if unknown, use the default value 3
		chopConf_buf.toff = (toff_buf == 0) ? 3 : toff_buf;
	}
	else {
		// save existing toff value and set it to 0
		if (chopConf_buf.toff != 0)
			toff_buf = chopConf_buf.toff;
		chopConf_buf.toff = 0;
	}
	// this also saves the last non-zero toff value to toff_buf
	return setRegister(CHOPCONF_REG, chopConf_buf.data);
}

// This could be useful if the chip has been reset
// but the IHOLD_IRUN and COOLCONF registers cannot be read
bool TMC22xxDriver::readShadowRegisters()
{
	if (!getRegister(GCONF_REG, &gconf_buf.data))
		return false;
	if (!getRegister(CHOPCONF_REG, &chopConf_buf.data))
		return false;
	if (!getRegister(PWMCONF_REG, &pwmConf_buf.data))
		return false;
	// note: IHOLD_IRUN_REG and COOLCONF_REG cannot be read
	return true;
}

inline bool TMC22xxDriver::getGCONF(GCONF* gconf)
{
	return getRegister(GCONF_REG, (uint32_t*)gconf);
}

// Clear the latched reset and drv_err status flags
// drv_err is only cleared when all error conditions are cleared
bool TMC22xxDriver::clearGSTAT()
{
	GSTAT gstat;
	gstat.data = 0;
	gstat.reset = 1;
	gstat.drv_err = 1;
	return setRegister(GSTAT_REG, gstat.data);
}

inline bool TMC22xxDriver::getGSTAT(GSTAT* gstat)
{
	return getRegister(GSTAT_REG, (uint32_t*)gstat);
}

// Read the Interface transmission counter (0..255)
// This is incremented with each successful UART interface write access.
// Use it to check for lost messages. Read accesses do not change it.
// It wraps from 255 to 0.
bool TMC22xxDriver::getIFCNT(uint* ifcnt)
{
	IFCNT reg;
	bool ok = getRegister(IFCNT_REG, &reg.data);
	*ifcnt = reg.ifcnt;
	return ok;
}

// Read the One-Time Programmable data
// the chip uses the OTP settings to initialize if no UART interface is present
// use setDefaultConfiguration() to override the OTP settings
inline bool TMC22xxDriver::getOTP_READ(OTP_READ* otp_read)
{
	return getRegister(OTP_READ_REG, (uint32_t*)otp_read);
}

// Reads the state of all input pins
// can be used to check DIAG state
inline bool TMC22xxDriver::getIOIN(IOIN* ioin)
{
	return getRegister(IOIN_REG, (uint32_t*)ioin);
}

// Trim values for internal clock frequency and over temperature
// these are initialized from the OTP memory on startup from factory set values
// fclkTrim : 0..31, lowest to highest clock frequency, measure at charge pump output
// otTrim   : 0..3, see data sheet
bool TMC22xxDriver::getFACTORY_CONF(uint* fclkTrim, uint* otTrim)
{
	FACTORY_CONF factory_conf;
	bool ok = getRegister(FACTORY_CONF_REG, &factory_conf.data);
	*fclkTrim = factory_conf.fclkTrim;
	*otTrim = factory_conf.otTrim;
	return ok;
}

// Get the actual measured time between two 1/256 microsteps derived
// from the step input frequency in units of 1/fCLK. 
// Measured value is (2^20)-1 in case of overflow or stand still. 
// TSTEP always relates to 1/256 step, independent of the actual MRES.
// The TSTEP related threshold uses a hysteresis of 1/16 of the compare 
// value to compensate for jitter in the clock or the step frequency: 
// (Txxx*15/16)-1 is the lower compare value for each TSTEP based 
// comparison.
// This means that the lower switching velocity equals the calculated 
// setting, but the upper switching velocity is higher than defined by 
// the hysteresis setting.
bool TMC22xxDriver::getTSTEP(ulong* tStep)
{
	TSTEP tstep;
	bool ok = getRegister(TSTEP_REG, &tstep.data);
	*tStep = tstep.tstep;
	return ok;
}

// Returns the StallGuard result. For use with StealthChop mode only.
// SG_RESULT is updated with each full step, independent of TCOOLTHRS and SGTHRS. 
// A higher value signals a lower motor load and more torque headroom.
// Bits 9 and 0 will always be 0. 
// Scaling to 10 bit is for compatibility with StallGuard2.
bool TMC22xxDriver::getSG_RESULT(uint* sg_result)
{
	SG_RESULT sg_result_reg;
	bool ok = getRegister(SG_RESULT_REG, &sg_result_reg.data);
	*sg_result = sg_result_reg.sg_result;
	return ok;
}

// Driver status register
inline bool TMC22xxDriver::getDRV_STATUS(DRV_STATUS* drv_status)
{
	return getRegister(DRV_STATUS_REG, (uint32_t*)drv_status);
}

// These automatically generated values can be used to determine the 
// default power up setting for PWM_OFS and PWM_GRAD.
bool TMC22xxDriver::getPWM_AUTO(uint* pwm_ofs_auto, uint* pwm_grad_auto)
{
	PWM_AUTO pwm_auto;
	bool ok = getRegister(PWM_AUTO_REG, &pwm_auto.data);
	// note: if it fails then reg == 0
	*pwm_ofs_auto = pwm_auto.pwm_ofs_auto;
	*pwm_grad_auto = pwm_auto.pwm_grad_auto;
	return true;
}

// Get results of StealthChop amplitude regulator.
// Values can be used to monitor automatic PWM amplitude scaling (255=max voltage).
// pwm_scale_sum  : Actual PWM duty cycle. This value is used for scaling the 
//    CUR_A and CUR_B values read from the sine wave table.
// pwm_scale_auto : Signed offset added to the calculated PWM duty cycle. 
//    This is the result of the automatic amplitude regulation based on current 
//    measurement.
bool TMC22xxDriver::getPWM_SCALE(uint* pwm_scale_sum, int* pwm_scale_auto)
{
	PWM_SCALE pwm_scale;
	bool ok = getRegister(PWM_SCALE_REG, &pwm_scale.data);
	// note: if it fails then reg == 0
	*pwm_scale_sum = pwm_scale.pwm_scale_sum;
	*pwm_scale_auto = pwm_scale.pwm_scale_auto;
	return ok;
}


// Chopper, PWM and CoolStep configuration
// note: setRegister() updates the shadow registers

inline bool TMC22xxDriver::setCHOPCONF(CHOPCONF chopConf)
{
	return setRegister(CHOPCONF_REG, chopConf.data);
}

inline bool TMC22xxDriver::setPWMCONF(PWMCONF pwmConf)
{
	return setRegister(PWMCONF_REG, pwmConf.data);
}

inline bool TMC22xxDriver::setCOOLCONF(COOLCONF coolConf)
{
	return setRegister(COOLCONF_REG, coolConf.data);
}

inline bool TMC22xxDriver::setTCOOLTHRS(uint tcoolthrs)
{
	ASSERT(tcoolthrs < 0x100000);	// 20-bit
	return setRegister(TCOOLTHRS_REG, tcoolthrs);
}

inline bool TMC22xxDriver::setTPWMTHRS(uint tpwmthrs)
{
	ASSERT(tpwmthrs < 0x100000);	// 20-bit
	return setRegister(TPWMTHRS_REG, tpwmthrs);
}

// Set the detection threshold for stall. 
// The StallGuard value SG_RESULT is compared to double this threshold value.
// A stall is signalled when SG_RESULT <= SGTHRS * 2
inline bool TMC22xxDriver::setSGTHRS(uint sgthrs)
{
	ASSERT(sgthrs < 256);
	return setRegister(SGTHRS_REG, sgthrs);
}

// Set the delay time from stand still (stst) detection to motor
// current power down. Time range is about 0 to 5.6 seconds.
// tPowerDown = 0 .. 255 * 2^18 tCLK (tClk = 83.333nS at 12MHz)
// reset default = 20 (437 milliseconds)
// mimimum of 2 for automatic tuning of StealthChop PWM_OFFS_AUTO
bool TMC22xxDriver::setTPOWERDOWN(uint tpowerDown)
{
	ASSERT(tpowerDown < 256);
	TPOWERDOWN reg;
	reg.data = 0;
	reg.tpowerDown = tpowerDown;
	return setRegister(TPOWERDOWN_REG, reg.data);
}

// Driver Current Control
// ihold      : Standstill current (0=1/32 .. 31=32/32)
// irun       : Motor run current (0=1/32 .. 31=32/32) 
// iholdDelay : Controls the number of clock cycles for motor power down
//    after standstill is detected (stst = 1) and TPOWERDOWN has expired.
//    The smooth transition avoids a motor jerk upon power down.
//    0 = instant power down
//    1..15 = delay per current reduction step in multiples of 2^18 clocks
// See also getRMSCurrent() and getRMSCurrentSettings()
bool TMC22xxDriver::setDriverCurrent(uint ihold, uint irun, uint iholdDelay)
{
	ASSERT(ihold < 32 && irun < 32 && iholdDelay < 16);
	IHOLD_IRUN reg;
	reg.data = 0;
	reg.ihold = ihold;
	reg.irun = irun;
	reg.iholdDelay = iholdDelay;
	return setRegister(IHOLD_IRUN_REG, reg.data);
}


///////////////////////////////////////////////////////////////////////////////
// Microsteps

// Set the microstep resolution
// NOTE: 'GCONF.mstep_reg_select' must be true, see setConfiguration().
// NOTE: If using MiniStepper, call MiniStepper.setMicrosteps() too
// microsteps : 0..8, corresponding to: 256 128 64 32 16 8 4 2 1(FULLSTEP)
// The resolution gives the number of microstep entries per sine quarter wave.
// When choosing a lower microstep resolution, the driver automatically uses 
// microstep positions which result in a symmetrical wave.
// Number of microsteps per step pulse = 2^MRES 
bool TMC22xxDriver::setMicrosteps(uint microsteps)
{
	ASSERT(microsteps >= 1 && microsteps <= 256);
	ASSERT(microsteps == 1 ? true : (microsteps % 2) == 0);
	this->currentMicrosteps = microsteps;
	chopConf_buf.mres = microstepsToMres(microsteps);
	return setRegister(CHOPCONF_REG, chopConf_buf.data);
}

bool TMC22xxDriver::getMicrosteps(uint* microsteps, bool forceRead/*= false*/)
{
	if (forceRead) {
		if (!getRegister(CHOPCONF_REG, &chopConf_buf.data))
			return false;
	}
	*microsteps = mresToMicrosteps(chopConf_buf.mres);
	this->currentMicrosteps = *microsteps;
	return true;
}

// mres is 0..8, corresponding to microsteps of 256 128 64 32 16 8 4 2 1
int TMC22xxDriver::microstepsToMres(uint microsteps)
{
	if (microsteps > 256)
		return 8;
	int mres = 8;
	while (microsteps >>= 1)
		--mres;
	return mres;
}
inline int TMC22xxDriver::mresToMicrosteps(uint mres)
{
	if (mres > 8)
		return 1;
	return 1 << (8 - mres);
}


// Read the microstep sequencer registers
// mscnt : Microstep counter. Indicates actual position in the microstep 
//    table for CUR_A. CUR_B uses an offset of 256 into the table. 
//    Reading MSCNT allows determination of the motor position within the 
//    electrical wave.
// cur_a, cur_b : Actual microstep current for motor phase A or B as read 
//    from the internal sine wave table (not scaled by current setting)
bool TMC22xxDriver::getMicrostepRegisters(uint* mscnt, int* cur_a, int* cur_b)
{
	MSCNT reg;
	bool ok = getRegister(MSCNT_REG, &reg.data);
	MSCURACT mscuract;
	ok &= getRegister(MSCURACT_REG, (uint32_t*)&mscuract);
	// note: if getRegister() fails then reg == 0
	*mscnt = reg.mscnt;
	*cur_a = mscuract.cur_a;
	*cur_b = mscuract.cur_b;
	return ok;
}


///////////////////////////////////////////////////////////////////////////////
// Velocity Movement, p67
// Normally the STEP/DIR signals control the movement, but the TMC chips 
// have an "internal step pulse generator". Movement is controlled by the
// value in the VACTUAL register.
// NOTE! Motion at high velocities will require ramping up and ramping 
// down the 'vactual' velocity value in software.


// Run the motor at the given velocity until velocityMoveStop() is called
// vactual : 0 = normal control using STEP and DIR
//    -8'388'608..8'388'607 (0xff800000 .. 0x007fffff) =  signed 24-bit 
//    velocity in 'microsteps-per-t', where 't' is 'fclk / 2^24'
// Direction is controlled by vactual's sign (+ve or -ve).
// Step pulses can be counted via the INDEX output, if connected.
// NOTE: Use the MiniStepper class for STEP/DIR control.
inline bool TMC22xxDriver::velocityMoveStart(long vactual)
{
	ASSERT(vactual >= -8388608 && vactual <= 8388607);
	return setRegister(VACTUAL_REG, vactual);
}

// Stop the current velocity movement
inline bool TMC22xxDriver::velocityMoveStop()
{
	return setRegister(VACTUAL_REG, 0);
}

// Convert revolutions-per-minute to VACTUAL for velocityMoveStart(vactual)
// the practical maximum values for revolutionsPerMinute is +-300rpm
inline long TMC22xxDriver::rpmToVACTUAL(float revolutionsPerMinute)
{
	return rpsToVACTUAL(revolutionsPerMinute / 60.0f);
}

// Convert revolutions-per-second to VACTUAL for velocityMoveStart(vactual)
// the practical maximum value for revolutionsPerSecond is +-5rps
long TMC22xxDriver::rpsToVACTUAL(float revolutionsPerSecond)
{
	return (revolutionsPerSecond * currentMicrosteps * fullstepsPerRevolution * t1) + 0.5f;
}

// Convert fullsteps-per-second to VACTUAL for velocityMoveStart(vactual)
// the practical maximum value for fullstepsPerSecond is +-1000
long TMC22xxDriver::fspsToVACTUAL(float fullstepsPerSecond)
{
	return (fullstepsPerSecond * currentMicrosteps * t1) + 0.5f;
}

// TSTEP is the time between two 1/256 microsteps derived from the step input 
// frequency in units of 1/fCLK
long TMC22xxDriver::vactualToTSTEP(long vactual)
{
	return (((vactual >> 1) - 1) + ((16777216 / 256) * currentMicrosteps)) / vactual;
}


///////////////////////////////////////////////////////////////////////////////
// Calculate the IHOLD_IRUN register and CHOPCONF.vsense values for the given 
// run and hold currents in milliamps RMS, or vice-versa.
// IRUN and IHOLD (0..31) allow for scaling of the actual current scale (CS) 
// from 1/32 to 32/32 when using UART interface.
// 
// Use this to determine the values for IHOLD_IRUN.irun, IHOLD_IRUN.ihold 
// and CHOPCONF.vsense, according to the motor currents you need. Use the 
// minimum currents that provide satisfactory performance.
// 
// rsense : The current sense resistor value in ohms, usually 0.11 or 0.22 ohms, 
//          found from the board's schematic diagram. 0.11 for max. 2A motors
//          or 0.22 for 1A motors.
// vsense : The CHOFCONF.vsense boolean value, selects Vfs, 0=0.180V, 1=0.325V.
// 
bool TMC22xxDriver::getSettingsFromRmsCurrent(uint iRunCurrentRmsMilliamps, 
	uint iHoldCurrentRmsMilliamps, float rsense, bool vsense, 
	uint* irun, uint* ihold)
{
	*irun = 0;
	*ihold = 0;

	ASSERT(iRunCurrentRmsMilliamps <= 2000 && iHoldCurrentRmsMilliamps <= 2000 && 
		iHoldCurrentRmsMilliamps <= iRunCurrentRmsMilliamps);
	ASSERT(rsense >= 0.075f && rsense <= 1.0f);

	float Vfs = vsense ? 0.180f : 0.325f;
	*irun  = calculateCS((float)iRunCurrentRmsMilliamps / 1000.0f, Vfs, rsense);
	*ihold = calculateCS((float)iHoldCurrentRmsMilliamps / 1000.0f, Vfs, rsense);
	return true;
}

bool TMC22xxDriver::getRmsCurrentFromSettings(uint irun, uint ihold, 
	float rsense, bool vsense,
	uint* iRunCurrentRmsMilliamps, uint* iHoldCurrentRmsMilliamps)
{
	*iRunCurrentRmsMilliamps = 0;
	*iHoldCurrentRmsMilliamps = 0;

	ASSERT(irun < 32 && ihold < 32);
	ASSERT(rsense >= 0.075f && rsense <= 1.0f);

	float Vfs = vsense ? 0.180f : 0.325f;
	*iRunCurrentRmsMilliamps  = (uint)(calculateIrms(irun, Vfs, rsense) * 1000.0f);
	*iHoldCurrentRmsMilliamps = (uint)(calculateIrms(ihold, Vfs, rsense) * 1000.0f);
	return true;
}

// Calculations from data sheet, p53
uint TMC22xxDriver::calculateCS(float irms, float Vfs, float rsense)
{
	float cs = ((irms * 32.0f * (rsense + 0.02f)) / (Vfs * 0.7071f)) - 1.0f;
	uint ics = (uint)(cs + 0.5f);		// round up
	return ics > 31 ? 31 : ics;			// limit to max
}
float TMC22xxDriver::calculateIrms(uint CS, float Vfs, float rsense)
{
	return ((CS + 1.0f) / 32.0f) * (Vfs / (rsense + 0.02f)) * 0.7071f;
}


///////////////////////////////////////////////////////////////////////////////
// One Time Programming (OTP), p26, see README.md
// You can only do this once! So it's patched out for safety.
// OTP IS ONLY NEEDED IF UART CONFIGURATION IS NOT USED
// >>> OTP_FCLKTRIM is NOT changed, this is factory-programmed <<<
/* commented out unless needed
bool TMC22xxDriver::programOTP(OTP_READ otpData)
{
	OTP_PROG otpProg;
	otpProg.data = 0;
	otpProg.otpMagic = 0xbd;

	// program in bits 5..23, one bit at a time
	// (bits 0..4 are the factory-set clock trim, we do not change those!)
	uint nbit = 5;
	uint nbyte = 0;
	uint32_t mask = 0x000020;
	while (1) {

		// program only the 1s
		if ((otpData.data & mask) != 0) {

			// set the bit
			otpProg.otpByte = nbyte;
			otpProg.otpBit = nbit;
			if (!setRegister(OTP_PROG_REG, otpProg.data))
				return false;
			delay(50);		// min. 10ms to write, allow 50ms

			// was the bit written correctly?
			uint32_t readData;
			if (!getRegister(OTP_READ_REG, &readData))
				return false;
			if ((readData & mask) == 0) {
				LOGERROR("OTP validation failed");
				return false;
			}
		}

		// next bit 0..7, next byte 0..3
		if (++nbit > 7) {
			nbit = 0;
			// 3 bytes done, we have finished
			if (++nbyte > 2)
				break;
		}
		mask <<= 1;
	}
	return true;
}
*/


///////////////////////////////////////////////////////////////////////////////
// Communications
// to use SPI, just replace these two methods

// Writes to a register
// also automatically updates the shadow registers
bool TMC22xxDriver::setRegister(uint reg, uint32_t data)
{
	// automatically save the shadow register values
	// do this here in one place so we ALWAYS have the actual 
	// values, especially for the registers which cannot be read
	switch (reg) {
	case GCONF_REG:
		gconf_buf.data = data;
		break;
	case IHOLD_IRUN_REG:
		ihold_irun_buf.data = data;
		break;
	case COOLCONF_REG:
		coolConf_buf.data = data;
		break;
	case CHOPCONF_REG:
		chopConf_buf.data = data;
		// save the last non-zero toff value for enableMotorDriversWithToff()
		if (chopConf_buf.toff != 0)
			toff_buf = chopConf_buf.toff;
		break;
	case PWMCONF_REG:
		pwmConf_buf.data = data;
		break;
	}

	#ifdef DEBUG
	// can this register be written?
	switch (reg) {
	case GCONF_REG:
	case GSTAT_REG:
	case NODECONF_REG:
	case OTP_PROG_REG:
	case FACTORY_CONF_REG:
	case IHOLD_IRUN_REG:
	case TPOWERDOWN_REG:
	case TPWMTHRS_REG:
	case VACTUAL_REG:
	case TCOOLTHRS_REG:
	case SGTHRS_REG:
	case COOLCONF_REG:
	case CHOPCONF_REG:
	case PWMCONF_REG:
		break;
	default:
		LOGERROR("write read-only reg");
		return false;
	}
	#endif

	//                  0     1     2     3     4     5     6     7
	//                  sync  adds  reg  |------32-bit data-----| crc
	//byte wrmsg[8] = { 0x05, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xcc };
	byte msg[8];
	msg[0] = 0x05;				// sync byte
	msg[1] = address;			// chip address 0..3, from MS1/MS2
	msg[2] = 0x80 | reg;		// register address + R/W bit
	byte* p = (byte*)&data;		// 32-bit data, ms byte first
	msg[6] = *p++;
	msg[5] = *p++;
	msg[4] = *p++;
	msg[3] = *p;
	msg[7] = crc8(msg, 7);

	// empty the receive buffer and send the message
	if (uart == NULL)
		return false;			// begin() not called?
	while (uart->available())
		uart->read();
	uart->write(msg, 8);

	#ifdef TMC22xxDRIVER_LOOPBACK
	// everything sent is also received (TX and RX are connected via a 1K resistor)
	// receive and check the echoed 8 bytes, this will detect noise on the lines
	long t = millis();
	uint count = 0;
	while (1) {
		while (uart->available()) {
			byte b = uart->read();
			if (b != msg[count]) {
				LOGERROR("write echo error");
				++errorCount;
				return false;
			}
			if (count++ == 8)
				break;
		}
		if (count == 8)
			break;
		if ((millis() - t) > 1000) {
			LOGERROR("write echo timeout");
			++errorCount;
			return false;
		}
	}
	#endif

	//TODO you may want to keep this in the release version
	//TODO patch this out if the TMC22xx's TX output is not connected
	#if defined(DEBUG) && defined(TMC22xxDRIVER_LOOPBACK)
	// read-after-write validation for RW resisters
	switch (reg) {
	case GCONF_REG:
	case FACTORY_CONF_REG:
	case CHOPCONF_REG:
	case PWMCONF_REG:
		uint32_t data1;
		if (!getRegister(reg, &data1))
			return false;
		if (data != data1) {
			LOGERROR("compare error");
			++errorCount;
			return false;
		}
	}
	#endif

	// count number of sent messages, 0..255 then rolls over to 0
	// compare this with the getIFCNT() value to check for comms errors
	if (++expectedifcnt > 255)
		expectedifcnt = 0;

	return true;
}

// Reads a register value, always returns 32-bits
// only use this if the TX output is connected
bool TMC22xxDriver::getRegister(uint reg, uint32_t* data)
{
	*data = 0;			// returned value is 0 if it fails

	#ifdef DEBUG
	// can this register be read?
	switch (reg) {
	case GCONF_REG:
	case GSTAT_REG:
	case IFCNT_REG:
	case OTP_PROG_REG:
	case OTP_READ_REG:
	case IOIN_REG:
	case FACTORY_CONF_REG:
	case TSTEP_REG:
	case SG_RESULT_REG:
	case MSCNT_REG:
	case MSCURACT_REG:
	case CHOPCONF_REG:
	case DRV_STATUS_REG:
	case PWMCONF_REG:
	case PWM_SCALE_REG:
	case PWM_AUTO_REG:
		// can be read
		break;
	default:
		LOGERROR("register cannot be read");
		return false;
	}
	#endif

	byte msg[4];
	msg[0] = 0x05;			// sync byte
	msg[1] = address;		// chip address, from MS1/MS2
	msg[2] = reg;			// register address + R/W bit (0)
	msg[3] = crc8(msg, 3);

	// empty the receive buffer and send the message
	if (uart == NULL)
		return false;		// begin() not called?
	while (uart->available())
		uart->read();
	uart->write(msg, 4);

	// everything sent is also received (TX and RX are connected via a 1K resistor)
	// receive echoed 4 bytes + 8 byte response
	// 012345678901
	// eeeesarddddc
	byte response[12];
	uint rxLength = 8;
	#ifdef TMC22xxDRIVER_LOOPBACK
	rxLength = 12;
	#endif

	long t = millis();
	uint count = 0;
	while (1) {
		while (uart->available()) {
			response[count++] = uart->read();
			if (count == rxLength)
				break;
		}
		if (count == rxLength)
			break;
		if ((millis() - t) > 1000) {
			LOGERROR("read timeout");	// check #define TMC22xxDRIVER_LOOPBACK
			++errorCount;
			return false;
		}
	}
	byte* rxmsg;

	#ifdef TMC22xxDRIVER_LOOPBACK
	// check echoed sent message is ok, compare 4 bytes
	// this will detect noise on the lines
	if (*(uint32_t*)msg != *(uint32_t*)response) {
		LOGERROR("echo error");
		++errorCount;
		return false;
	}
	rxmsg = response + 4;
	#else
	rxmsg = response;
	#endif

	// validate the response
	if (rxmsg[0] != 0x05 || rxmsg[1] != 0xff || rxmsg[2] != reg) {
		LOGERROR("invalid response");
		++errorCount;
		return false;
	}
	if (rxmsg[7] != crc8(rxmsg, 7)) {
		LOGERROR("crc error");
		++errorCount;
		return false;
	}

	// return the 32-bit value
	*data = reverse4Bytes(rxmsg + 3);

	return true;
}

// Fast CRC8 using look-up table
byte TMC22xxDriver::crc8(const byte* msg, uint length)
{
	uint result = 0;
	while (length--)
		result = crcTable[result ^ *msg++];

	#if defined(__ARM_ARCH) && (__ARM_ARCH >= 7)
	// reverse the bits with RBIT instruction
	uint reversed;
	__asm__("rbit %0, %1" : "=r"(reversed) : "r"(result));
	return ((byte*)&reversed)[3];

	#else
	// reverse the bits with C code
	uint reversed = 0;
	uint mask1 = 0x01;
	uint mask2 = 0x80;
	while (1) {
		if (result & mask1)
			reversed |= mask2;
		if ((mask1 <<= 1) == 0)
			break;
		mask2 >>= 1;
	}
	return (byte)reversed;
	#endif
}

const byte TMC22xxDriver::crcTable[256] =
{
	0x00, 0x91, 0xe3, 0x72, 0x07, 0x96, 0xe4, 0x75, 0x0e, 0x9f, 0xed, 0x7c, 0x09, 0x98, 0xea, 0x7b,
	0x1c, 0x8d, 0xff, 0x6e, 0x1b, 0x8a, 0xf8, 0x69, 0x12, 0x83, 0xf1, 0x60, 0x15, 0x84, 0xf6, 0x67,
	0x38, 0xa9, 0xdb, 0x4a, 0x3f, 0xae, 0xdc, 0x4d, 0x36, 0xa7, 0xd5, 0x44, 0x31, 0xa0, 0xd2, 0x43,
	0x24, 0xb5, 0xc7, 0x56, 0x23, 0xb2, 0xc0, 0x51, 0x2a, 0xbb, 0xc9, 0x58, 0x2d, 0xbc, 0xce, 0x5f,
	0x70, 0xe1, 0x93, 0x02, 0x77, 0xe6, 0x94, 0x05, 0x7e, 0xef, 0x9d, 0x0c, 0x79, 0xe8, 0x9a, 0x0b,
	0x6c, 0xfd, 0x8f, 0x1e, 0x6b, 0xfa, 0x88, 0x19, 0x62, 0xf3, 0x81, 0x10, 0x65, 0xf4, 0x86, 0x17,
	0x48, 0xd9, 0xab, 0x3a, 0x4f, 0xde, 0xac, 0x3d, 0x46, 0xd7, 0xa5, 0x34, 0x41, 0xd0, 0xa2, 0x33,
	0x54, 0xc5, 0xb7, 0x26, 0x53, 0xc2, 0xb0, 0x21, 0x5a, 0xcb, 0xb9, 0x28, 0x5d, 0xcc, 0xbe, 0x2f,
	0xe0, 0x71, 0x03, 0x92, 0xe7, 0x76, 0x04, 0x95, 0xee, 0x7f, 0x0d, 0x9c, 0xe9, 0x78, 0x0a, 0x9b,
	0xfc, 0x6d, 0x1f, 0x8e, 0xfb, 0x6a, 0x18, 0x89, 0xf2, 0x63, 0x11, 0x80, 0xf5, 0x64, 0x16, 0x87,
	0xd8, 0x49, 0x3b, 0xaa, 0xdf, 0x4e, 0x3c, 0xad, 0xd6, 0x47, 0x35, 0xa4, 0xd1, 0x40, 0x32, 0xa3,
	0xc4, 0x55, 0x27, 0xb6, 0xc3, 0x52, 0x20, 0xb1, 0xca, 0x5b, 0x29, 0xb8, 0xcd, 0x5c, 0x2e, 0xbf,
	0x90, 0x01, 0x73, 0xe2, 0x97, 0x06, 0x74, 0xe5, 0x9e, 0x0f, 0x7d, 0xec, 0x99, 0x08, 0x7a, 0xeb,
	0x8c, 0x1d, 0x6f, 0xfe, 0x8b, 0x1a, 0x68, 0xf9, 0x82, 0x13, 0x61, 0xf0, 0x85, 0x14, 0x66, 0xf7,
	0xa8, 0x39, 0x4b, 0xda, 0xaf, 0x3e, 0x4c, 0xdd, 0xa6, 0x37, 0x45, 0xd4, 0xa1, 0x30, 0x42, 0xd3,
	0xb4, 0x25, 0x57, 0xc6, 0xb3, 0x22, 0x50, 0xc1, 0xba, 0x2b, 0x59, 0xc8, 0xbd, 0x2c, 0x5e, 0xcf
};

#if 0
// The slow version, from example in datasheet section '4.2 CRC Calculation', p20
byte TMC22xxDriver::crc8(const byte* msg, uint length)
{
	byte crc = 0;
	for (uint i = 0; i < length; ++i) {
		byte b = msg[i];
		for (int j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (b & 1))
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
			b >>= 1;
		}
	}
	return crc;
}
#endif

// Convert 32-bit little startian to little endian, or back again
uint32_t TMC22xxDriver::reverse4Bytes(const byte* data)
{
	#if defined(__ARM_ARCH) && (__ARM_ARCH >= 6)
	uint32_t v = *(uint32_t*)data;
	__asm__("rev %0, %1" : "=r"(v) : "r"(v));
	return v;

	#else 
	uint32_t v;
	byte* pv = ((byte*)&v) + 3;
	for (int i = 0; i < 4; ++i)
		*pv-- = *data++;
	return v;
	#endif
}

