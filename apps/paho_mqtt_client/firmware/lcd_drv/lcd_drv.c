#include "lcd_drv.h"

/*
 * This 'driver'is a merge of two  libs so it's a bit of a jumble
 * with hardware specific statements in at least two source files.
 * Because the PIC32MK version will likely only be used with graphic
 * displays much of the code is unused and will one-day be removed
 * after functional testing is complete.
 * 10/15/2021
 * uses SPI1 with a 15MHz clock for the LCD chip. 
 */

static void delay_us(uint32_t);

void init_lcd_drv(LCD_DVR_STATE init_type)
{
	switch (init_type) {
	case D_MISC:
	case D_INIT: // send the GLCD buffer data via DMA
		CSB_SetHigh(); // select SPI GLCD display, DOGXL240 @15MHz SPI clock
		delay_us(IS_DELAYPOWERUP); // > 400ms power up delay
		lcd_init();
		OledInit();
		OledSetCharUpdate(0); // manual LCD screen updates for speed
		OledMoveTo(bmp_x, bmp_y); // position image
		OledPutBmp(bmp_size, bmp_size, (uint8_t *) foo_map); // upload bitmap image from C array
		delay_us(BMP_DELAY); // show image for a bit
		break;
	default:
		break;
	}
}

/*
 * microsecond busy wait delay, 90 seconds MAX
 * Careful, uses core timer
 */
void delay_us(uint32_t us)
{
	CORETIMER_DelayUs(us);
}