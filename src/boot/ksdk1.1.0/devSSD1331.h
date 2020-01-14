/*
 *	See https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino for the Arduino driver.
 */

typedef enum
{
	kSSD1331ColororderRGB		= 1,
	kSSD1331DelaysHWFILL		= 3,
	kSSD1331DelaysHWLINE		= 1,
} SSD1331Constants;

typedef enum
{
	kSSD1331CommandDRAWLINE		= 0x21,
	kSSD1331CommandDRAWRECT		= 0x22,
	kSSD1331CommandCLEAR		= 0x25,
	kSSD1331CommandFILL		= 0x26,
	kSSD1331CommandSETCOLUMN	= 0x15,
	kSSD1331CommandSETROW		= 0x75,
	kSSD1331CommandCONTRASTA	= 0x81,
	kSSD1331CommandCONTRASTB	= 0x82,
	kSSD1331CommandCONTRASTC	= 0x83,
	kSSD1331CommandMASTERCURRENT	= 0x87,
	kSSD1331CommandSETREMAP		= 0xA0,
	kSSD1331CommandSTARTLINE	= 0xA1,
	kSSD1331CommandDISPLAYOFFSET	= 0xA2,
	kSSD1331CommandNORMALDISPLAY	= 0xA4,
	kSSD1331CommandDISPLAYALLON	= 0xA5,
	kSSD1331CommandDISPLAYALLOFF	= 0xA6,
	kSSD1331CommandINVERTDISPLAY	= 0xA7,
	kSSD1331CommandSETMULTIPLEX	= 0xA8,
	kSSD1331CommandSETMASTER	= 0xAD,
	kSSD1331CommandDISPLAYOFF	= 0xAE,
	kSSD1331CommandDISPLAYON	= 0xAF,
	kSSD1331CommandPOWERMODE	= 0xB0,
	kSSD1331CommandPRECHARGE	= 0xB1,
	kSSD1331CommandCLOCKDIV		= 0xB3,
	kSSD1331CommandPRECHARGEA	= 0x8A,
	kSSD1331CommandPRECHARGEB	= 0x8B,
	kSSD1331CommandPRECHARGEC	= 0x8C,
	kSSD1331CommandPRECHARGELEVEL	= 0xBB,
	kSSD1331CommandVCOMH		= 0xBE,
} SSD1331Commands;
int	devSSD1331init(void);
void drawPixelSSD1331(uint8_t x, uint8_t y, uint16_t colour);
void drawCharSSD1331(uint8_t x, uint8_t y, const char c, uint16_t color, uint16_t bg);
void writeStringSSD1331(uint8_t x, uint8_t y, const char* charString, uint16_t color, uint16_t bg);
void updateStringSSD1331(uint8_t x, uint8_t y, const char* oldCharArray, const char* newCharArray,
        uint16_t color, uint16_t bg);
void writeNumSSD1331(uint8_t x, uint8_t y, const int32_t number,
                      uint16_t color, uint16_t bg, const char * format);
void updateNumSSD1331(uint8_t x, uint8_t y, const int32_t oldNum, const int32_t newNum,
                          uint16_t color, uint16_t bg, const char * format);
void filledRectSSD1331(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint16_t colorline,uint16_t colorfill);