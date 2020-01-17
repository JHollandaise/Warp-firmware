/*
	Authored 2019-2020. Joseph Holland

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef WARP_BUILD_ENABLE_DEVHX711
#define WARP_BUILD_ENABLE_DEVHX711
#endif

// initialise pins and gain. By default use PTA5(SCK) and PTA8(DOUT) see gpio_pins.h
void initHX711(const uint32_t sckPinName, const uint32_t doutPinName, uint8_t gain);

// Check if HX711 is ready
// from the datasheet: When output data is not ready for retrieval, digital output pin DOUT is high. Serial clock
// input PD_SCK should be low. When DOUT goes to low, it indicates data is ready for retrieval.
bool isReadyHX711();

// Wait for the HX711 to become ready
void waitReadyHX711(uint16_t delay_ms);
//bool waitReadyRetryHX711(int retries = 3, unsigned long delay_ms = 0);
//bool waitReadyTimeoutHX711(unsigned long timeout = 1000, unsigned long delay_ms = 0);

// set the gain factor; takes effect only after a call to read()
// channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
// depending on the parameter, the channel is also set to either A or B
void setGainHX711(uint8_t gain);

// waits for the chip to be ready and returns a reading
int32_t readHX711();

void waitForStable(uint16_t delay_ms, uint16_t num, uint32_t tol);


// set the offsetHX711 value for tare weight; num = how many times to read the tare value
void tareHX711(uint16_t num);

// set mass calibration scale. Requires known calibration weight!
// num = how many times to read the tare/offset values
void calibrateHX711(uint16_t num, uint32_t cal_mass);

// return calibrated value
int32_t readCalibratedHX711();


//// puts the chip into power down mode
//void power_down();
//
//// wakes up the chip after power down mode
//void power_up();

// get data buffer from board
uint8_t shiftInMsbFirstHX711();




