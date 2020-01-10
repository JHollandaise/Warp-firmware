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
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "fsl_misc_utilities.h"
#include "devHX711.h"
#include "gpio_pins.h"
#include "fsl_gpio_driver.h"
#include "fsl_os_abstraction.h"
#include "SEGGER_RTT.h"


// pin definitions
uint32_t sckPinNameHX711;
uint32_t doutPinNameHX711;

// gain amplification code
uint8_t gainHX711;

// calibration values
double offsetHX711 = 0;
double scaleHX711 = 1;
double calMassHX711 = 1;

// initialise pins and gain factor. By default use PTA5(SCK) and PTA8(DOUT) see gpio_pins.h
// Channel selection is made by passing the appropriate gain:
// - With a gain factor of 64 or 128, channel A is selected
// - With a gain factor of 32, channel B is selected
// The library default is "128" (Channel A).
void initHX711(const uint32_t _sckPinName, const uint32_t _doutPinName, uint8_t gain)
{
    sckPinNameHX711 = _sckPinName;
    doutPinNameHX711 = _doutPinName;
    setGainHX711(gain);

    // ensure sck is low to power up board
    GPIO_DRV_ClearPinOutput(sckPinNameHX711);
}

void setGainHX711(uint8_t gain)
{
    switch (gain)
    {
        case 128:		// channel A, gain factor 128
            gainHX711 = 1;
            break;
        case 64:		// channel A, gain factor 64
            gainHX711 = 3;
            break;
        case 32:		// channel B, gain factor 32
            gainHX711 = 2;
            break;
        default:
            // this is bad, as no error checking. But user has no direct value access
            gainHX711 = 1;
    }

    GPIO_DRV_ClearPinOutput(sckPinNameHX711);
    readHX711();
}

int32_t readHX711()
{
    // Wait for the chip to become ready.
    waitReadyHX711(0);

    // Define structures for reading data into.
    uint32_t value = 0;
    uint8_t data[3] = {0,0,0};
    uint8_t filler = 0x00;

    // pulse the clock pin 24 times to read the data
    data[2] = shiftInMsbFirstHX711();
    data[1] = shiftInMsbFirstHX711();
    data[0] = shiftInMsbFirstHX711();

    // Set the channel and the gain factor for the next reading using the clock pin.
    for (uint16_t i = 0; i < gainHX711; i++) {
        GPIO_DRV_SetPinOutput(sckPinNameHX711);
        GPIO_DRV_ClearPinOutput(sckPinNameHX711);
    }

    // Replicate the most significant bit to pad out a 32-bit signed integer
    if ( data[2] & 0x80 ) {
        filler = 0xFF;
    } else {
        filler = 0x00;
    }

    // Construct a 32-bit signed integer
    value = ( (uint32_t)filler << 24
              | (uint32_t)data[2] << 16
              | (uint32_t)data[1] << 8
              | (uint32_t)data[0] );

    return (int32_t)value;
}

int32_t readCalibratedHX711()
{
    // additional 1000 times scaling puts result in mg to maintain precision
    return (int32_t)round( (readHX711() - offsetHX711) * calMassHX711 * 1000 / scaleHX711 );
}

void waitReadyHX711(uint16_t delay_ms)
{
    while(!isReadyHX711())
    {
        OSA_TimeDelay(delay_ms);
    }
}

bool isReadyHX711()
{
    return !(bool)GPIO_DRV_ReadPinInput(doutPinNameHX711);
}

uint8_t shiftInMsbFirstHX711()
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < 8; ++i) {
        GPIO_DRV_SetPinOutput(sckPinNameHX711);
        value |= GPIO_DRV_ReadPinInput(doutPinNameHX711) << (7 - i);
        GPIO_DRV_ClearPinOutput(sckPinNameHX711);
    }
    return value;
}



// takes num readings and calculates range, if smaller than tol returns. Otherwise wait
// delay_ms milliseconds
void waitForStable(uint16_t delay_ms, uint16_t num, uint32_t tol)
{
    OSA_TimeDelay(delay_ms);

    int32_t maxReading;
    int32_t minReading;
    int32_t currentVal;
    int16_t i = 0;

    while(true){
        for(i=0;i<num;i++){
            currentVal = readHX711();
            if(i==0){
                maxReading=currentVal;
                minReading=currentVal;
            } else {
                maxReading = currentVal>maxReading?currentVal:maxReading;
                minReading = currentVal<minReading?currentVal:maxReading;
            }
        }

        // check for success condition
        if(tol > abs(maxReading-minReading)) return;

    }
}

void tareHX711(uint16_t num)
{
    double accumulator = 0;

    // ensure no weight change
    waitForStable(100,10,300);

    for(int i=0;i<num;i++){
        accumulator += readHX711();
    }

    offsetHX711 = accumulator/num;
}

void calibrateHX711(uint16_t num, int32_t diff, uint16_t calMass)
{
    double accumulator = 0;

    // first ensure scale is zeroed correctly
    tareHX711(num);

    calMassHX711 = calMass;

    SEGGER_RTT_printf(0, "\rapply %dg mass", calMass);
    // wait for appropriate mass to be placed on scale
    while(diff>abs(readHX711() - (int32_t)offsetHX711)) OSA_TimeDelay(100);

    // wait for stability
    waitForStable(100,10,300);

    // generate summed value
    for(int i=0;i<num;i++){
        accumulator += (readHX711() - offsetHX711);
    }

    // scale is average value
    scaleHX711 = accumulator/num;
}