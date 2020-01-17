/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.

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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devHX711.h"


/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile WarpModeMask			gWarpMode			= kWarpModeDisableAdcOnSleep;
volatile uint32_t			gWarpMenuPrintDelayMilliseconds	= 10;

void					lowPowerPinStates(void);
uint16_t				read4digits(void);


void
lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705, PAN1326) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	/*
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
	*/

	/*
	 *	PTA3 and PTA4 are the EXTAL/XTAL
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	
	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);


	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	
	/*
	 *	PTB1 is connected to KL03_VDD. We have a choice of:
	 *		(1) Keep 'disabled as analog'.
	 *		(2) Set as output and drive high.
	 *
	 *	Pin state "disabled" means default functionality (ADC) is _active_
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	}
	else
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	}

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB3 (I2C pins) are true open-drain
	 *	and we purposefully leave them disabled.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);


	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB8 or PTB9
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB12
	 */
	
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);



	/*
	 *	Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
	 */
	
	
	
	/*
	 *	If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
	}
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinHX711_SCK);
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 *	Drive these chip selects high since they are active low:
	 */
    GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);

	/*
	 *	When the PAN1326 is installed, note that it has the
	 *	following pull-up/down by default:
	 *
	 *		HCI_RX / kWarpPinI2C0_SCL	: pull up
	 *		HCI_TX / kWarpPinI2C0_SDA	: pull up
	 *		HCI_RTS / kWarpPinSPI_MISO	: pull up
	 *		HCI_CTS / kWarpPinSPI_MOSI	: pull up
	 *
	 *	These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
	 *	so we really don't want to be driving them low. We
	 *	however also have to be careful of the I2C pullup and
	 *	pull-up gating. However, driving them high leads to
	 *	higher board power dissipation even when SSSUPPLY is off
	 *	by ~80mW on board #003 (PAN1326 populated).
	 *
	 *	In revB board, with the ISL23415 DCP pullups, we also
	 *	want I2C_SCL and I2C_SDA driven high since when we
	 *	send a shutdown command to the DCP it will connect
	 *	those lines to 25570_VOUT. 
	 *
	 *	For now, we therefore leave the SPI pins low and the
	 *	I2C pins (PTB3, PTB4, which are true open-drain) disabled.
	 */

	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	GPIO_DRV_ClearPinOutput(kWarpPinSW2);
	GPIO_DRV_ClearPinOutput(kWarpPinSW3);

}


int
main(void)
{
	/* menu parameters */

    // selected option
    uint8_t					key;
    // sensor reads before auto exit to menu
	uint16_t                numReadsHX711 = 0;
	// false is hex; true is decimal
    bool                    rawOutputTypeHX711 = false;
    // moving average size
    uint8_t                 averageReadingLength = 1;


    // stored values for moving average (initialised here to save initialising it multiple times)
    int32_t                storedReadings[20];

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();


	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(200);

	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);



	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	
	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	lowPowerPinStates();


	// load bar driver initialisation (default gain = 128)
    initHX711(kWarpPinHX711_SCK , kWarpPinHX711_DOUT,128);


	while (true)
	{
		/*
		 *	Do not, e.g., lowPowerPinStates() on each iteration, because we actually
		 *	want to use menu to progressively change the machine state with various
		 *	commands.
		 */

		/*
		 *	We break up the prints with small delays to allow us to use small RTT print
		 *	buffers without overrunning them when at max CPU speed.
		 */
		SEGGER_RTT_WriteString(0, "\r\n\n\n\n[ *\t\t\t\tJ\ta\tn\tk\t(rev. 1)\t\t\t* ]\n");
		SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t      Cambridge / egdirbmaC   \t\t\t\t  ]\n\n");
        SEGGER_RTT_WriteString(0, "\r[  \t\t HX711 Load Cell Testing Platform \t\t  ]\n\n");

        SEGGER_RTT_WriteString(0, "\rSelect:\n");
        SEGGER_RTT_WriteString(0, "\r- '1': Read Raw data.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '2': Read Calibrated data (mg).\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '3': Calibrate Sensor.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '4': Tare Sensor.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '5': Set Gain.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '6': Set Number of reads.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '7': Set raw output type.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- '8': Set moving average length.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

		SEGGER_RTT_WriteString(0, "\rEnter selection> ");
		key = SEGGER_RTT_WaitKey();
		switch (key)
		{
            case '1':
            case '2':
			{
			    int32_t (*readValueFunction)(void);

				// initial previous and current set to beginning of loop
                uint32_t currTime = OSA_TimeGetMsec();
				uint32_t prevTime;
                uint32_t elapsedTime = 0;

				int64_t accumulator = 0;


				if (key=='1')   readValueFunction = &readHX711;
				else            readValueFunction = &readCalibratedHX711;

				// TODO: overflow bug in accumulation (about 8 million reads later...)
                for(uint32_t i=0; (i<numReadsHX711)||(numReadsHX711==0); i++)
                {
                    // accumulator is at full capacity
                    if (i >= averageReadingLength)
                    {
                        // remove oldest value from accumulator
                        accumulator -= storedReadings[i%averageReadingLength];
                    }

                    // read new value into array
                    storedReadings[i%averageReadingLength] = readValueFunction();

                    // add new value to accumulator
                    accumulator += storedReadings[i%averageReadingLength];

                    // shift old time over
                    prevTime = currTime;
                    // update current time
                    currTime = OSA_TimeGetMsec();
                    // timer overflow correction factor
                    if(currTime < prevTime) elapsedTime += 65535;
                    elapsedTime += currTime - prevTime;

                    // some accuracy lost here as not working with floating points
                    if (rawOutputTypeHX711||(key=='2')){
                        // TODO: weird internal bug seems to require this to be done with two prints.
                        SEGGER_RTT_printf(0, "\n\r%u",elapsedTime);
                        SEGGER_RTT_printf(0, ", %d", accumulator/min(averageReadingLength,i+1));
                    }
                    else {
                        SEGGER_RTT_printf(0, "\n\r%u",elapsedTime);
                        SEGGER_RTT_printf(0, ", %X", accumulator/min(averageReadingLength,i+1));
                    }


                    // quit early if 'q' held
                    if (SEGGER_RTT_GetKey() == 'q') break;

                }
                break;
            }
		    case '3':
            {

                SEGGER_RTT_WriteString(0, "\r\nEnter chosen calibration mass (mg) and ensure scale is EMPTY (e.g '00500000')> ");
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                uint32_t calibrationMass = read4digits()*10000;
                calibrationMass += read4digits();

                SEGGER_RTT_WriteString(0, "\r\ntaring...");
                // tare over 100 values
                tareHX711(100);

                SEGGER_RTT_printf(0, "\r\napply %dmg mass (then press enter)", calibrationMass);
                // wait for any key
                SEGGER_RTT_WaitKey();
                SEGGER_RTT_WriteString(0, "\r\ncalibrating...");

                // calibrate over 100 value reads
                calibrateHX711(100,calibrationMass);
                break;
            }
		    case '4':
            {
                SEGGER_RTT_WriteString(0, "\r\ntaring...");
                // tare over 150 values
                tareHX711(100);
                break;
            }
            case '5':
            {
                SEGGER_RTT_WriteString(0, "\r\n'1'=32; '2'=64; '3'=128 > ");
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                key = SEGGER_RTT_WaitKey();
                uint8_t selectedGain= 0;
                switch (key)
                {
                    // pretty nasty fallthrough to save on lines.
                    case '3':
                        // 32+32+64 = 128
                        selectedGain += 64;
                    case '2':
                        // 32+32 = 64
                        selectedGain += 32;
                    case '1':
                        selectedGain += 32;

                    setGainHX711(selectedGain);
                    SEGGER_RTT_printf(0,"\r\nselected gain = %d\n", selectedGain);
                        break;
                    default:
                        SEGGER_RTT_WriteString(0, "\r\nInvalid entry!\n");
                }

                break;
            }
		    case '6':
            {
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                SEGGER_RTT_WriteString(0, "\r\nEnter number of value reads (e.g '1000' ) ('0000' is endless)> ");
                numReadsHX711 = read4digits();
                SEGGER_RTT_printf(0,"\r\nnumber of reads = %d\n", numReadsHX711);
                break;
            }
		    case '7':
            {
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                SEGGER_RTT_WriteString(0, "\r\n'1'=HEX; '2'=DEC > ");
                key = SEGGER_RTT_WaitKey();

                rawOutputTypeHX711 = false;
                switch (key)
                {
                    // another nasty fallthrough to save on print statements
                    case '1':
                        rawOutputTypeHX711 = !rawOutputTypeHX711;
                    case '2':
                        rawOutputTypeHX711 = !rawOutputTypeHX711;

                        SEGGER_RTT_printf(0, "\r\nSelected output: %s\n", rawOutputTypeHX711?"DEC":"HEX");
                        break;
                    default:
                        SEGGER_RTT_WriteString(0, "\r\nInvalid entry!\n");
                }
                break;
            }
            case '8':
            {
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
                SEGGER_RTT_WriteString(0, "\r\nEnter number of readings to average (e.g '1000' ) ('0020' is max)> ");
                uint16_t valueCheck;
                if((valueCheck = read4digits()) > 20) averageReadingLength = 20;
                else if(valueCheck==0) averageReadingLength = 1;
                else averageReadingLength = (uint8_t)valueCheck;

                SEGGER_RTT_printf(0, "\r\nMoving average length: %d\n", averageReadingLength);
                break;
            }

			/*
			 *	Ignore naked returns.
			 */
			case '\n':
			{
				SEGGER_RTT_WriteString(0, "\r\tPayloads make rockets more than just fireworks.");
				break;
			}

			default:
			{
				SEGGER_RTT_printf(0, "\r\tInvalid selection '%c' !\n", key);
			}
		}
	}

}

uint16_t
read4digits(void)
{
	uint8_t		digit1, digit2, digit3, digit4;

	digit1 = SEGGER_RTT_WaitKey();
	digit2 = SEGGER_RTT_WaitKey();
	digit3 = SEGGER_RTT_WaitKey();
	digit4 = SEGGER_RTT_WaitKey();

	return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}