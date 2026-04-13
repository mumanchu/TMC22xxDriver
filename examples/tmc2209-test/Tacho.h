#pragma once

/////////////////////////////////////////////////////////////////////
// A simple class for counting and timing input interrupts
// muman.ch, 2026.04.12
/*
USAGE EXAMPLE
=============

#include "Tacho.h"

#define TACHO0_PIN 2
Tacho tacho0;

// The interrupt handler must be static
// it calls the Tacho instance's interrupt handler
void tacho0InterruptHandler() {
	tacho0.interruptHandler();
}

void setup() {
	...
	tacho0.begin(TACHO0_PIN, tacho0InterruptHandler);
	...
}
*/

class Tacho
{
	ulong counter;
	ulong microsecSum;
	ulong lastMicrosecs;

public:
	void interruptHandler() {
		ulong microsecs = micros();
		if (lastMicrosecs > 0)
			microsecSum += microsecs - lastMicrosecs;
		lastMicrosecs = microsecs;
		++counter;
	}

	void begin(uint pin, void (*interruptHandler)(), uint mode = RISING) {
		counter = 0;
		microsecSum = 0;
		lastMicrosecs = 0;
		pinMode(pin, INPUT);
		attachInterrupt(pin, interruptHandler, mode);
	}

	void reset() {
		noInterrupts();
		counter = 0;
		microsecSum = 0;
		lastMicrosecs = 0;
		interrupts();
	}

	float getPeriod() {
		noInterrupts();
		uint counter1 = counter;
		uint microsecSum1 = microsecSum;
		interrupts();
		if (counter1 < 2)	// needs 2 readings
			return 0.0f;
		return (float)microsecSum1 / counter1;
	}

	ulong getPeriodAndReset() {
		noInterrupts();
		ulong period = counter < 2 ? 0 : microsecSum / counter;
		counter = 0;
		microsecSum = 0;
		lastMicrosecs = 0;
		interrupts();
		return period;
	}

	ulong getCount() {
		noInterrupts();
		ulong counter1 = counter;
		interrupts();
		return counter1;
	}

	ulong getCountAndReset() {
		noInterrupts();
		ulong counter1 = counter;
		counter = 0;
		microsecSum = 0;
		lastMicrosecs = 0;
		interrupts();
		return counter1;
	}
};

