/*
 * ECET 311 - Embedded Systems 1
 * File: lcd.c
 * Author: Willy Bido
 *
 * Description:
 * This file is a LCD driver for LCDs based on the HD44780 or compatible
 * dot matrix liquid crystal display (LCD) display controllers.
 * This file is designed to drive the LCD when wired in 4-bit mode.
 * 
 * Hardware Configuration Constraints:
 * - LCD signals LCD_D7-LCD_D4 must be hooked up to the 4 MSB's of a PORT
 * 
 */

#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

/*
 * Initialize the LCD
 * This function preforms the following tasks
 *  - Configures the IO ports for the LCD
 *  - Preforms a software resets on the LCD
 *  - Configures the LCD for the desired mode of operation
 * To accomplish this, this function calls
 *  - lcd_io_init()
 *  - lcd_reset()
 *  - lcd_configure()
 */
void lcd_init(void)
{
	lcd_io_init();
	lcd_reset();
	lcd_configure();
}

/*
 * Configure the micro controller IO Ports for communication
 * with the LCD controller.
 * - Pins connected to LCD_D7 through LCD_D4 are configured as outputs
 * - Pin connected to LCD_E is configured as an output
 * - Pin connected to LCD_RS is configured as an output
 */
void lcd_io_init(void)
{
	// Fill in your function code here
	// Use the Define Names from lcd.h and not the PORT, DDR, or PIN names
	LCD_DATA_DDR |= (1 << LCD_DATA_D7_PIN) | (1 << LCD_DATA_D6_PIN) | (1 << LCD_DATA_D5_PIN) | (1 << LCD_DATA_D4_PIN);
	
	LCD_E_DDR |= (1 << LCD_E_PIN);
	LCD_RS_DDR |= (1 << LCD_RS_PIN);
}

/*
 * Reset the LCD controller
 * Preforms the HD44780 software reset routine
 * Puts the LCD in 4-Bit mode
 */
void lcd_reset()
{
	// Fill in your function code here

	// Make sure that as you write data to the LCD_D7-LCD_D4 pins
	// that your code only effects the data pins and that the
	// state of the RS and E signals remains as desired. Consider
	// using the simulator to check your code. Comment out delays
	// while simulating if you try this.

	// Select instruction register (Use the macro from lcd.h)
	// Wait 20ms for the LCD controller to boot
	// Send 0x30
	lcd_rs_low();
	
	lcd_e_high();
	_delay_ms(20);
	LCD_DATA_PORT = 0x30 | (LCD_DATA_PORT & 0x0F);
	lcd_e_low();
	
	// Wait 5ms for instruction to execute
	// Send 0x30
	lcd_e_high();
	_delay_ms(5);
	LCD_DATA_PORT = 0x30 | (LCD_DATA_PORT & 0x0F);
	lcd_e_low();
	
	// Wait 200us for instruction to execute
	// Send 0x30
	lcd_e_high();
	_delay_us(200);
	LCD_DATA_PORT = 0x30 | (LCD_DATA_PORT & 0x0F);
	lcd_e_low();
	
	// Wait 100us for the instruction to execute
	// Send 0x20 to put the LCD in 4-Bit mode
	// Wait 1ms for instruction to execute
	lcd_e_high();
	_delay_us(100);
	LCD_DATA_PORT = 0x20 | (LCD_DATA_PORT & 0x0F);
	lcd_e_low();
	_delay_ms(1);
}

/*
 * Configures the LCD to operate as:
 *  - 4-bit mode, 2 lines, 5x8 font	
 *  - Auto-increment DDRAM, no display shift
 *  - Display on, cursor off 
 */
void lcd_configure(void)
{
	lcd_send_instruction(LCD_INSTRUCTION_4BIT_2LINE_5X8);	  // 4 Bit Mode, 2 Lines, 5x8 Font
	lcd_send_instruction(LCD_INSTRUCTION_AUTO_INC_NO_SHIFT);  // Auto-increment, No Display Shift
	lcd_send_instruction(LCD_INSTRUCTION_DISP_ON_CURSOR_OFF); // Display on, no cursor
}
/*
 * Send an instruction to the LCD controller
 * Write an 8-bit instruction to the LCD instruction register in 4 bit mode
 */
void lcd_send_instruction(uint8_t data)
{
	// Fill in your function code here

	// Make sure that as you write data to the LCD_D7-LCD_D4 pins
	// that your code only effects the data pins and that the
	// state of the RS and E signals remains as desired. Consider
	// using the simulator to check your code. Comment out delays
	// while simulating if you try this.

	// Select the instruction register
	lcd_rs_low();
	// Set E to logic high
	lcd_e_high();
	// Place the 4 MSB's of data on LCD_D7-LCD_D4 (MSB->LSB)
	//LCD_DATA_PORT = (data & 0xF0);
	
	LCD_DATA_PORT &= 0x0f;
	LCD_DATA_PORT |= (data & 0xf0);
	
	
	// Set E to logic low
	lcd_e_low();
	// Set E to logic high
	lcd_e_high();
	// Place the 4 LSB's of data on LCD_D7-LCD_D4 (MSB->LSB)
	LCD_DATA_PORT &= 0x0f;
	LCD_DATA_PORT |= (data << 4) & 0xf0;//sends 4 LSB bits
	
	
	// Set E to logic low
	lcd_e_low();
	// Set E to logic high
	lcd_e_high();
	// Delay 400us to allow for the instruction to execute
	_delay_us(400);
}

/*
 * Set the DDRAM write address (cursor position) on the LCD controller
 * Uses one call to lcd_send_instruction()
 */
void lcd_set_ddram_address(uint8_t address)
{
	lcd_send_instruction(0x80 + address);
}

/*
 * Set the DDRAM write address (cursor position) to 0x00
 * Uses one call to lcd_send_instruction()
 */
void lcd_return_home(void)
{
	lcd_send_instruction(LCD_INSTRUCTION_RETURN_HOME);
	_delay_ms(2);
}

/*
 * Send data to the LCD controller data register
 * Write an 8-bit value to the LCD data register in 4 bit mode
 */
void lcd_send_data(uint8_t data)
{
	// Fill in your function code here

	// Make sure that as you write data to the LCD_D7-LCD_D4 pins
	// that your code only effects the data pins and that the
	// state of the RS and E signals remains as desired. Consider
	// using the simulator to check your code. Comment out delays
	// while simulating if you try this.

	// Select the data register
	lcd_rs_high();
	// Set E to logic high
	lcd_e_high();
	// Place the 4 MSB's of data on LCD_D7-LCD_D4 (MSB->LSB)
	//LCD_DATA_PORT = (data & 0xF0);
	
	LCD_DATA_PORT &= 0x0f;
	LCD_DATA_PORT |= (data & 0xf0);
	
	
	// Set E to logic low
	lcd_e_low();
	// Set E to logic high
	lcd_e_high();
	// Place the 4 LSB's of data on LCD_D7-LCD_D4 (MSB->LSB)
	LCD_DATA_PORT &= 0x0f;
	LCD_DATA_PORT |= (data << 4) & 0xf0;//sends 4 LSB bits
	
	
	// Set E to logic low
	lcd_e_low();
	// Set E to logic high
	lcd_e_high();
	// Delay 400us to allow for the instruction to execute
	_delay_us(400);
}

/*
 * Clear the display
 * Send the clear display instruction to the LCD controller
 * Waits the additional time required for the clear display instruction
 * to finish executing.
 * Uses one call to lcd_send_instruction()
 */
void lcd_clear_display(void)
{
	lcd_send_instruction(LCD_INSTRUCTION_CLEAR_DISPLAY);
	_delay_ms(2);
}

/*
 * Set the DDRAM write address (cursor position) to the address to
 * write x characters right and y rows down from the top left of the display
 * Uses one call to lcd_set_ddram_address()
 */
void lcd_go_to_xy(uint8_t x, uint8_t y)
{
	if ((y == 0) && (x < LCD_LINE_LENGTH))
		lcd_set_ddram_address(LCD_START_LINE1 + x);
	else if ((y == 1) && (x < LCD_LINE_LENGTH))
		lcd_set_ddram_address(LCD_START_LINE2 + x);
}

/*
 * Write one character to the LCD data register at the cursor
 * position currently stored on the LCD controller.
 * Uses one call to lcd_send_data()
 */
void lcd_putc(char c)
{
	lcd_send_data(c);
}

/*
 * Write a string of characters to the LCD data register starting at
 * the cursor position currently stored on the LCD controller.
 * Repeatedly calls lcd_putc()
 */
void lcd_puts(const char *s)
{
	register char c;
	while ((c = *s++))
	{
		lcd_putc(c);
	}
}

/*
 * Write an array of characters to the LCD starting at the
 * cursor position currently stored in the LCD controller
 * Repeatedly calls lcd_putc()
 */
void lcd_put_array(uint8_t *array, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	{
		lcd_putc(array[i]);
	}
}
