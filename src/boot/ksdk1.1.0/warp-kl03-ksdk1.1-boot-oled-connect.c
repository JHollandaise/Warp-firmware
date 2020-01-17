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

// required for SPI communication to OLED
#include "fsl_spi_master_driver.h"
#include "fsl_mcglite_hal.h"

#include "gpio_pins.h"
#include "warp.h"
#include "menuStates.h"

#include "devHX711.h"


volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;


volatile uint32_t			gWarpSpiBaudRateKbps		= 200;
volatile WarpModeMask		gWarpMode			        = kWarpModeDisableAdcOnSleep;
volatile uint32_t			gWarpSpiTimeoutMicroseconds	= 5;
volatile uint32_t			gWarpMenuPrintDelayMilliseconds	= 10;

void					lowPowerPinStates(void);
uint16_t				read4digits(void);

void
enableSPIpins(void)
{
    CLOCK_SYS_EnableSpiClock(0);

    /*
     *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
     *
     */
    uint32_t			calculatedBaudRate;
    spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
    spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
    spiUserConfig.direction		= kSpiMsbFirst;
    spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
    SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
    SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void
disableSPIpins(void)
{
    SPI_DRV_MasterDeinit(0);

    CLOCK_SYS_DisableSpiClock(0);
}


// TODO: add pin states for pan1326 lp states
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
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);


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

	/*
	 *	HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

	/*
	 *	HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

	/*
	 *	HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

	/*
	 *	From PAN1326 manual, page 10:
	 *
	 *		"When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
	 */
	//GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}

int
main(void)
{

    // menu state: what the screen will display
    menuState 				currentMenuState = rawDecimal;
	uint16_t				menuI2cPullupValue = 32768;


	/* HX711 related menu parameters */
	uint8_t selectedGain = 128;
	uint16_t selectedCalibMass = 500;
    // moving average
    uint8_t                averageReadingLength = 1;
    // stored values for moving average (initialised here to save initialising it multiple times)
    int32_t                storedReadings[20];

	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;


    /*
     *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
     */
    OSA_Init();

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


	//OLED initialisation
	devSSD1331init();

	// load bar driver initialisation (default gain = 128)
    initHX711(kWarpPinHX711_SCK , kWarpPinHX711_DOUT,selectedGain);

	while (1)
	{
        // clear text area
        filledRectSSD1331(0,0,95,63,0x0000,0x0000);
		switch (currentMenuState)
		{
		    // data printing
            case rawDecimal:
            case rawHex:
			case calibratedHex:
			case calibratedmg:
			{
				int32_t (*readValueFunction)(void);

				// initial current reading set as 0
				int32_t currentReading = 0;
                int32_t lastReading;

				int64_t accumulator = 0;



                // print title text
                if (currentMenuState==rawDecimal) writeStringSSD1331(0,6,"Raw DEC",0xF373,0x0000);
                else if (currentMenuState==rawHex) writeStringSSD1331(0,6,"Raw HEX",0xF373,0x0000);
                else if (currentMenuState==calibratedmg) writeStringSSD1331(0,6,"Calibrated (mg)",0xF373,0x0000);
                else writeStringSSD1331(0,6,"Calibrated HEX",0xF373,0x0000);


				if ((currentMenuState==rawDecimal) || (currentMenuState==rawHex))
					readValueFunction = &readHX711;
				else            readValueFunction = &readCalibratedHX711;

				for(uint32_t i=0;true; i++)
				{
				    // shift old reading over
				    lastReading = currentReading;

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

					// compute current average
					currentReading = accumulator/min(averageReadingLength,i+1);

					// print result in decimal form
					if ((currentMenuState==rawDecimal) || (currentMenuState==calibratedmg)){
						// TODO: weird internal bug seems to require this to be done with two prints.
						updateNumSSD1331(0,30,lastReading,currentReading,0xE186,0x0000,"%d");
					}
					else {
                        updateNumSSD1331(0,30,lastReading,currentReading,0x0FF3,0x0000,"%X");
					}


					// switch display screen with button 1 (SW2)
					if(!GPIO_DRV_ReadPinInput(kWarpPinSW2)){

					    // button has to be released first
					    while(!GPIO_DRV_ReadPinInput(kWarpPinSW2));

                        currentMenuState=settingsTop;

						break;
					}
					// switch between display types with button 2 (SW3)
					if(!GPIO_DRV_ReadPinInput(kWarpPinSW3)){

                        // button has to be released first
                        while(!GPIO_DRV_ReadPinInput(kWarpPinSW3));

						switch (currentMenuState)
						{
							case rawDecimal:
								currentMenuState=rawHex;
								break;
							case rawHex:
								currentMenuState=calibratedmg;
								break;
							case calibratedmg:
								currentMenuState=calibratedHex;
								break;
							case calibratedHex:
								currentMenuState=rawDecimal;
								break;

						}
						break;
					}

				}
				break;
			}

            case settingsTop:
            {
                writeStringSSD1331(21,27,"Settings?",0x00FE,0x0000);

                while(true){
                    // switch display screen with button 1 (SW2)
                    if(!GPIO_DRV_ReadPinInput(kWarpPinSW2)){

                        // button has to be released first
                        while(!GPIO_DRV_ReadPinInput(kWarpPinSW2));

                        currentMenuState=rawDecimal;

                        break;
                    }

                    // Enter Settings menu
                    if(!GPIO_DRV_ReadPinInput(kWarpPinSW3)) {

                        // button has to be released first
                        while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));


                        currentMenuState = settingsTare;
                        break;
                    }
                }
                break;
            }

            case settingsTare:
            {
                writeStringSSD1331(34,27,"Tare?",0x053E,0x0000);

                while(true){
                    // switch settings screen with button 1 (SW2)
                    if(!GPIO_DRV_ReadPinInput(kWarpPinSW2)){

                        // button has to be released first
                        while(!GPIO_DRV_ReadPinInput(kWarpPinSW2));

                        currentMenuState=settingsCalibrate;

                        break;
                    }

                    // Tare Scale
                    if(!GPIO_DRV_ReadPinInput(kWarpPinSW3)) {

                        // button has to be released first
                        while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

                        writeStringSSD1331(22,27,"Taring...",0x053E,0x0000);

                        // tare over 100 values
                        tareHX711(100);

                        break;
                    }
                }
                break;
            }

		    case settingsCalibrate:
			{
				writeStringSSD1331(18, 27, "Calibrate?", 0x053E, 0x0000);

				while(true)
				{
					// switch settings screen with button 1 (SW2)
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW2)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW2));

						currentMenuState = settingsMovingAvg;

						break;
					}

					// Calibrate Scale
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW3)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

						writeStringSSD1331(6, 27, "Clear Any Mass", 0x053E, 0x0000);

						while(GPIO_DRV_ReadPinInput(kWarpPinSW3));

						// clear writing region
						filledRectSSD1331(0,27,95,37,0x0000,0x0000);
                        writeStringSSD1331(22,27,"Taring...",0x053E,0x0000);

                        // tare over 100 readings
                        tareHX711(100);


                        // button has to be released first
                        while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

                        // clear writing region
                        filledRectSSD1331(0,27,95,37,0x0000,0x0000);

                        writeStringSSD1331(1, 23, "Calib Mass? (g)", 0x053E, 0x0000);

                        // print current mass
                        writeNumSSD1331(36,33,selectedCalibMass,0x15AB,0x0000,"%d");

                        // calib selection loop selection
                        while(true)
                        {
                            // advance
                            if(!GPIO_DRV_ReadPinInput(kWarpPinSW3)) break;

                            // change selected mass
                            if(!GPIO_DRV_ReadPinInput(kWarpPinSW2)){

                                switch (selectedCalibMass)
                                {
                                    case 100:
                                        selectedCalibMass = 200;
                                        break;
                                    case 200:
                                        selectedCalibMass = 500;
                                        break;
                                    case 500:
                                        selectedCalibMass = 1000;
                                        break;
                                    case 1000:
                                        selectedCalibMass = 100;
                                        break;
                                }

                                // clear number region
                                filledRectSSD1331(0,33,96,41,0x0000,0x0000);
                                //print current mass
                                writeNumSSD1331(36,33,selectedCalibMass,0x15AB,0x0000,"%d");

                                // wait for button release
                                while(!GPIO_DRV_ReadPinInput(kWarpPinSW2));
                            }


                        }
                        // clear writing region
                        filledRectSSD1331(0,23,95,43,0x0000,0x0000);
                        writeStringSSD1331(6, 27, "Calibrating...", 0x053E, 0x0000);

                        // needs calib mass in mg
                        calibrateHX711(100,selectedCalibMass*1000);
						break;
					}
				}
				break;
			}

			case settingsMovingAvg:
			{
				writeStringSSD1331(5, 23, "Average Length", 0x053E, 0x0000);

				// print current Average length
				writeNumSSD1331(43,33,averageReadingLength,0x15AB,0x0000,"%d");

				while(true)
				{
					// switch settings screen with button 1 (SW2)
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW2)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW2));

						currentMenuState = settingsGain;

						break;
					}

					// Change Moving Average Length
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW3)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

						if(averageReadingLength==20) averageReadingLength = 1;
						else averageReadingLength++;

                        // clear number region
                        filledRectSSD1331(0,33,96,41,0x0000,0x0000);
						// print new average value
						writeNumSSD1331(43,33,averageReadingLength,0x15AB,0x0000,"%d");
					}
				}
				break;
			}

			case settingsGain:
			{
				writeStringSSD1331(36, 23, "Gain", 0x053E, 0x0000);

				// print current gain value
				writeNumSSD1331(39,33,selectedGain,0x15AB,0x0000,"%d");

				while(true)
				{
					// switch settings screen with button 1 (SW2)
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW2)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW2));

						currentMenuState = settingsExit;

						break;
					}

					// Change Gain
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW3)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

						switch (selectedGain)
						{
							case 32:
								selectedGain = 64;
								break;
							case 64:
								selectedGain = 128;
								break;
							case 128:
								selectedGain = 32;
								break;
						}

						setGainHX711(selectedGain);

						// clear number region
						filledRectSSD1331(0,30,96,40,0x0000,0x0000);
						// print new gain value
						writeNumSSD1331(39,33,selectedGain,0x15AB,0x0000,"%d");
					}
				}
				break;
			}

			case settingsExit:
			{
				writeStringSSD1331(33, 28, "Exit?", 0x053E, 0x0000);

				while(true)
				{
					// switch settings screen with button 1 (SW2)
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW2)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW2));

						currentMenuState = settingsTare;

						break;
					}

					// Exit Settings
					if (!GPIO_DRV_ReadPinInput(kWarpPinSW3)) {

						// button has to be released first
						while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

						currentMenuState = settingsTop;
						break;
					}
				}
				break;
			}
		}
	}

	return 0;
}

