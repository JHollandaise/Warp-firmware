#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"
#include "glcdfont.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 1),
};

const char widthSSD1331 = 96;
const char heightSSD1331 = 64;

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

// will send count bytes of payloadBuffer over SPI. So ensure to manually set payload BEFORE CALLING THIS!
static int
writeCommands(uint8_t count)
{
    spi_status_t status;

    /*
     *	Drive /CS low.
     */
    GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

    /*
     *	Drive DC low (command).
     */
    GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

    status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
                                            NULL		/* spi_master_user_config_t */,
                                            (const uint8_t * restrict)&payloadBytes[0],
                                            (uint8_t * restrict)&inBuffer[0],
                                            count		/* transfer size */,
                                            1000		/* timeout in microseconds (unlike I2C which is ms) */);

    /*
     *	Drive /CS high
     */
    GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return status;
}

static int
writeData(uint8_t dataByte)
{
    spi_status_t status;

    /*
     *	Drive /CS low.
     */
    GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

    /*
     *	Drive DC high (data).
     */
    GPIO_DRV_SetPinOutput(kSSD1331PinDC);

    payloadBytes[0] = dataByte;
    status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
                                            NULL		/* spi_master_user_config_t */,
                                            (const uint8_t * restrict)&payloadBytes[0],
                                            (uint8_t * restrict)&inBuffer[0],
                                            1		/* transfer size */,
                                            1000		/* timeout in microseconds (unlike I2C which is ms) */);

    /*
     *	Drive /CS high
     */
    GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	// OLED CS
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	// OLED DC signal
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	// OLED RST signal
	PORT_HAL_SetMuxMode(PORTB_BASE, 1u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(15);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");



//	/*
//	 *	Read the manual for the SSD1331 (SSD1331_1.2.pdf) to figure
//	 *	out how to fill the entire screen with the brightest shade
//	 *	of green.
//	 */
//	 // enter "draw rectangle mode"
//	 writeCommand(0x22);
//	 // set starting column coords
//	 writeCommand(0);
//	 // set starting row coords
//	 writeCommand(0);
//	 // set end column coords;
//	 writeCommand(95);
//	 // set end row coords
//	 writeCommand(63);
//
//	 // set outline colour
//	 // no colour C (blue)
//	 writeCommand(00);
//	 // 255 (full value) colour for B (green)
//	 writeCommand(0xFF);
//	 // no colour A (red)
//	 writeCommand(00);
//
//	 // set fill colour
//	 // no colour C (blue)
//	 writeCommand(00);
//	 // 255 (full value) colour for B (green)
//	 writeCommand(0xFF);
//	 // no colour A (red)
//	 writeCommand(00);

	return 0;
}

void drawPixelSSD1331(uint8_t x, uint8_t y, uint16_t colour)
{
    payloadBytes[0] = kSSD1331CommandSETCOLUMN;
    payloadBytes[1] = x;
    payloadBytes[2] = x;

    payloadBytes[3] = kSSD1331CommandSETROW;
    payloadBytes[4] = y;
    payloadBytes[5] = y;

    writeCommands(6);
    // colour
    writeData((unsigned char) colour >> 8);
    writeData((unsigned char) colour);
}


void filledRectSSD1331(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint16_t colorline,uint16_t colorfill)
{

    // ensure fill is enabled (currently not needed as fill is never disabled after init)
//    payloadBytes[0] = kSSD1331CommandFILL;
//    payloadBytes[1] = 1;
//    writeCommands(2);


    payloadBytes[0] = kSSD1331CommandDRAWRECT;
    payloadBytes[1] = x1;
    payloadBytes[2] = y1;
    payloadBytes[3] = x2;
    payloadBytes[4] = y2;
    payloadBytes[5] = (uint8_t)((colorline>> 11) << 1);    // Outline Blue
    payloadBytes[6] = (uint8_t)((colorline>> 5 ) & 0x3F);  // Outline Green
    payloadBytes[7] = (uint8_t)((colorline<< 1 ) & 0x3F);  // Outline Red
    payloadBytes[8] = (uint8_t)((colorfill>> 11) << 1);    // fill Blue
    payloadBytes[9] = (uint8_t)((colorfill>> 5 ) & 0x3F);  // fill Green
    payloadBytes[10]= (uint8_t)((colorfill<< 1 ) & 0x3F);  // fill Red

    writeCommands(11);
}

// Draw a character
/**************************************************************************/
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ascii)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
    @param    bg 16-bit 5-6-5 Color to fill background with (if same as color,
   no background)
*/
/**************************************************************************/
void drawCharSSD1331(uint8_t x, uint8_t y, const char c,
							uint16_t color, uint16_t bg)
{


    if ((x >= widthSSD1331) ||              // Clip right
        (y >= heightSSD1331) ||             // Clip bottom
        ((x + 5) < 0) || // Clip left
        ((y + 4) < 0))   // Clip top
        return;

    for (int8_t i = 0; i < 5; i++) { // Char bitmap = 5 columns
        uint8_t line = basicFont[c * 5 + i];
        for (int8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 1)
                drawPixelSSD1331(x + i, y + j, color);
            else if (color != bg)
                drawPixelSSD1331(x + i, y + j, bg);
        }
    }


}

void writeStringSSD1331(uint8_t x, uint8_t y, const char* charArray, uint16_t color, uint16_t bg)
{
    for(const char* c = charArray; *c != '\0'; c++){
        drawCharSSD1331(x,y,*c,color,bg);
        x+=6;
    }
}

void updateStringSSD1331(uint8_t x, uint8_t y, const char* oldCharArray, const char* newCharArray,
        uint16_t color, uint16_t bg)
{

    uint8_t oldLength = strlen(oldCharArray);
    uint8_t newLength = strlen(newCharArray);
    uint8_t i=0;
    // loop through shared length array
    for(; (i < oldLength)&&(i < newLength); i++){
        if(*(newCharArray+i) != *(oldCharArray+i))
            drawCharSSD1331(x,y,*(newCharArray+i),color,bg);
        x+=6;
    }

    // need to clear characters
    if (oldLength > newLength) {
        for(;*(oldCharArray+i) != '\0';i++){
            drawCharSSD1331(x,y,' ',color,bg);
            x+=6;
        }
    }
    // need to add additional characters
    else if (newLength > oldLength) {
        for(;*(newCharArray+i) != '\0';i++){
            drawCharSSD1331(x,y,*(newCharArray+i),color,bg);
            x+=6;
        }
    }

}

void writeNumSSD1331(uint8_t x, uint8_t y, const int32_t number,
                      uint16_t color, uint16_t bg, const char * format)
{
    char numberArray[15];
    sprintf(numberArray,format, number);

    writeStringSSD1331(x,y, numberArray, color, bg);
}

void updateNumSSD1331(uint8_t x, uint8_t y, const int32_t oldNum, const int32_t newNum,
        uint16_t color, uint16_t bg, const char * format)
{
    char oldNumArray[15];
    char newNumArray[15];
    sprintf(oldNumArray,format, oldNum);
    sprintf(newNumArray,format, newNum);

    updateStringSSD1331(x,y, oldNumArray, newNumArray, color,bg);
}
