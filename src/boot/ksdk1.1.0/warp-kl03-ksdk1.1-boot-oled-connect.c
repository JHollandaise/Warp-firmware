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

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"

#include "gpio_pins.h"
#include "warp.h"
#include "menuStates.h"

#define WARP_FRDMKL03

#include "devSSD1331.h"
#include "devHX711.h"
#include "devMMA8451Q.h"

//#define WARP_BUILD_BOOT_TO_CSVSTREAM


/*
*	BTstack includes WIP
*/
// #include "btstack_main.h"


#define						kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define						kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define						kWarpConstantStringErrorSanity		"\rSanity check failed!"

#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
volatile WarpI2CDeviceState			deviceMMA8451QState;
#endif
#ifdef WARP_BUILD_ENABLE_DEVINA219
volatile WarpI2CDeviceState			deviceINA219State;
#endif


/*
 *	TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t			i2cMasterState;
volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t		spiUserConfig;
volatile lpuart_user_config_t 			lpuartUserConfig;
volatile lpuart_state_t 			lpuartState;

/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t			gWarpI2cBaudRateKbps		= 200;
volatile uint32_t			gWarpUartBaudRateKbps		= 1;
volatile uint32_t			gWarpSpiBaudRateKbps		= 200;
volatile uint32_t			gWarpSleeptimeSeconds		= 0;
volatile WarpModeMask			gWarpMode			= kWarpModeDisableAdcOnSleep;
volatile uint32_t			gWarpI2cTimeoutMilliseconds	= 5;
volatile uint32_t			gWarpSpiTimeoutMicroseconds	= 5;
volatile uint32_t			gWarpMenuPrintDelayMilliseconds	= 10;
volatile uint32_t			gWarpSupplySettlingDelayMilliseconds = 1;

void					sleepUntilReset(void);
void					lowPowerPinStates(void);
int					char2int(int character);
uint8_t					readHexByte(void);
uint16_t				read4digits(void);


/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus				writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus				writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void					warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);


/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
					power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}

/*
 *	From KSDK power_manager_demo.c <<END>>>
 */



void
sleepUntilReset(void)
{
	while (1)
	{
#ifdef WARP_BUILD_ENABLE_DEVSI4705
		GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
#endif
		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);
#ifdef WARP_BUILD_ENABLE_DEVSI4705
		GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
#endif
		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}

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

void
enableI2Cpins(uint16_t pullupValue)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);


	/*
	 *	TODO: need to implement config of the DCP
	 */
	//...
}



void
disableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);	


	/*	Warp KL03_I2C0_SCL	--> PTB3	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);


	/*
	 *	TODO: need to implement clearing of the DCP
	 */
	//...

	/*
	 *	Drive the I2C pins low
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);


	CLOCK_SYS_DisableI2cClock(0);
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
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
#endif

	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinHX711_SCK);

#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
#endif

	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 *	Drive these chip selects high since they are active low:
	 */
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
    GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
#endif
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
#ifdef WARP_BUILD_ENABLE_DEVADXL362
	GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);
#endif

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



void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);
	warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
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

	rtc_datetime_t				warpBootDate;

	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure			powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};



	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);



	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;



	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

		/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM,
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);



	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);



	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);



	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;
	
	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;
	
	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;
	
	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);



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



	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

#ifdef WARP_BUILD_BOOT_TO_CSVSTREAM
	/*
	 *	Force to printAllSensors
	 */
	gWarpI2cBaudRateKbps = 300;
	warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
	enableSssupply(3000);
	enableI2Cpins(menuI2cPullupValue);
	printAllSensors(false /* printHeadersAndCalibration */, false /* hexModeFlag */, 0 /* menuDelayBetweenEachRun */, menuI2cPullupValue);
	/*
	 *	Notreached
	 */
#endif

	//OLED initialisation
	devSSD1331init();

	// load bar driver initialisation (default gain = 128)
    initHX711(kWarpPinHX711_SCK , kWarpPinHX711_DOUT,selectedGain);


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
	while (1)
	{
        // clear text area
        filledRectSSD1331(0,0,95,63,0x0000,0x000);
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
                if (currentMenuState==rawDecimal) writeStringSSD1331(0,6,"Raw DEC",0xFFFF,0x0000);
                else if (currentMenuState==rawHex) writeStringSSD1331(0,6,"Raw HEX",0xFFFF,0x0000);
                else if (currentMenuState==calibratedmg) writeStringSSD1331(0,6,"Calibrated (mg)",0xFFFF,0x0000);
                else writeStringSSD1331(0,6,"Calibrated HEX",0xFFFF,0x0000);


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
						updateNumSSD1331(0,30,lastReading,currentReading,0xF373,0x0000,"%d");
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
                writeStringSSD1331(21,27,"Settings?",0xFFFF,0x0000);

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
                writeStringSSD1331(34,27,"Tare?",0xFFFF,0x0000);

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

                        writeStringSSD1331(22,27,"Taring...",0xFFFF,0x0000);

                        // tare over 20 values
                        tareHX711(30);

                        break;
                    }
                }
                break;
            }

		    case settingsCalibrate:
			{
				writeStringSSD1331(18, 27, "Calibrate?", 0xFFFF, 0x0000);

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

						writeStringSSD1331(6, 27, "Clear Any Mass", 0xFFFF, 0x0000);

						while(GPIO_DRV_ReadPinInput(kWarpPinSW3));

						// clear writing region
						filledRectSSD1331(0,27,95,37,0x0000,0x0000);
                        writeStringSSD1331(22,27,"Taring...",0xFFFF,0x0000);

                        // tare over 30 readings
                        tareHX711(30);


                        // button has to be released first
                        while (!GPIO_DRV_ReadPinInput(kWarpPinSW3));

                        // clear writing region
                        filledRectSSD1331(0,27,95,37,0x0000,0x0000);

                        writeStringSSD1331(1, 23, "Calib Mass? (g)", 0xFFFF, 0x0000);

                        // print current mass
                        writeNumSSD1331(36,33,selectedCalibMass,0xFFFF,0x0000,"%d");

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
                                writeNumSSD1331(36,33,selectedCalibMass,0xFFFF,0x0000,"%d");
                            }

                        }
                        // clear writing region
                        filledRectSSD1331(0,23,95,43,0x0000,0x0000);
                        writeStringSSD1331(6, 27, "Calibrating...", 0xFFFF, 0x0000);

                        calibrateHX711(30,selectedCalibMass);
						break;
					}
				}
				break;
			}

			case settingsMovingAvg:
			{
				writeStringSSD1331(5, 23, "Average Length", 0xFFFF, 0x0000);

				// print current Average length
				writeNumSSD1331(43,33,averageReadingLength,0xFFFF,0x0000,"%d");

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
						writeNumSSD1331(43,33,averageReadingLength,0xFFFF,0x0000,"%d");
					}
				}
				break;
			}

			case settingsGain:
			{
				writeStringSSD1331(36, 23, "Gain", 0xFFFF, 0x0000);

				// print current gain value
				writeNumSSD1331(39,33,selectedGain,0xFFFF,0x0000,"%d");

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
						writeNumSSD1331(39,33,selectedGain,0xFFFF,0x0000,"%d");
					}
				}
				break;
			}

			case settingsExit:
			{
				writeStringSSD1331(33, 28, "Exit?", 0xFFFF, 0x0000);

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
#pragma clang diagnostic pop

	return 0;
}



int
char2int(int character)
{
	if (character >= '0' && character <= '9')
	{
		return character - '0';
	}

	if (character >= 'a' && character <= 'f')
	{
		return character - 'a' + 10;
	}

	if (character >= 'A' && character <= 'F')
	{
		return character - 'A' + 10;
	}

	return 0;
}







WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
						gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;
	
	enableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0		/* master instance */,
						NULL		/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength	/* transfer size */,
						1000		/* timeout in microseconds (unlike I2C which is ms) */);					
	disableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}
