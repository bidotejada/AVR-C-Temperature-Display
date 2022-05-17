/*
 * ECET 311 - Embedded Systems 1
 * File: lcd.h
 * Description: Header file for lcd.c
 */

// LCD Physical Connection

#define LCD_RS_PIN	2		// The port pin number connected to the RS pin
#define	LCD_RS_PORT	PORTD		// The port the RS pin is attached to
#define LCD_RS_DDR	DDRD		// The Data Direction register responsible for the RS pin

#define LCD_E_PIN	3		// The port pin number connected to the RS pin
#define LCD_E_PORT	PORTD		// The port the RS pin is attached to
#define LCD_E_DDR	DDRD		// The Data Direction register responsible for the RS pin

#define LCD_DATA_PORT	PORTD	// The port the data lines on the LCD are connected to 
#define LCD_DATA_DDR	DDRD	// The data direction register for LCD_DATA_PORT
#define LCD_DATA_D7_PIN	7	// The port pin number connected to the LCD D7 pin
#define LCD_DATA_D6_PIN	6	// The port pin number connected to the LCD D6 pin
#define LCD_DATA_D5_PIN	5	// The port pin number connected to the LCD D5 pin
#define LCD_DATA_D4_PIN	4	// The port pin number connected to the LCD D4 pin

// LCD Parameter  Information
#define LCD_NUM_LINES	2	// The number of visible lines of the display
#define LCD_DISP_LENGTH	16	// The number of visible characters per line of the display
#define LCD_LINE_LENGTH	40	// The internal number of characters on each line, including invisible characters
#define LCD_START_LINE1	0x00	// The DDRAM address of the first character on line 1
#define LCD_START_LINE2	0x40	// The DDRAM address of the first character on line 2

// LCD Instruction Codes
#define LCD_INSTRUCTION_CLEAR_DISPLAY			0b00000001	// The instruction to clear the display
#define LCD_INSTRUCTION_RETURN_HOME				0b00000010	// The instruction to set DDRAM address to 0 and reset display shift
#define LCD_INSTRUCTION_AUTO_INC_NO_SHIFT		0b00000110	// The instruction to enable auto increment and set display shift to none

#define LCD_INSTRUCTION_DISP_ON_CURSOR_OFF		0b00001100	// 12-The instruction to turn the display on and turn the cursor off
#define LCD_INSTRUCTION_DISP_ON_CURSOR_ON		0b00001110	// 14-The instruction to turn the display on and turn the cursor on
#define LCD_INSTRUCTION_DISP_ON_CURSOR_BLINK	0b00001111	// 15-The instruction to turn the display on and make the cursor blink
#define LCD_INSTRUCTION_DISP_OFF_CURSOR_OFF		0b00001000	// The instruction to turn the display off

#define LCD_INSTRUCTION_4BIT_2LINE_5X8			0b00101000	// The instruction to set the LCD into 4-bit mode, 2 lines, 5x8 font
#define	LCD_INSTRUCTION_DISP_SHIFT_RIGHT		0b00011100	// The instruction to shift the display right by one column
#define LCD_INSTRUCTION_DISP_SHIFT_LEFT			0b00011000	// The instruction to shift the display left by one column

// Macro Definitions
#define lcd_e_high() 	LCD_E_PORT |= (1 << LCD_E_PIN);   // Macro to set the LCD E pin high (Delete: Should set LCD_E and only LCD_E high)
#define lcd_e_low()  	LCD_E_PORT &= ~(1 << LCD_E_PIN);   // Macro to set the LCD E pin low (Delete: Should set LCD_E and only LCD_E low)
#define lcd_rs_high()   LCD_RS_PORT |= (1 << LCD_RS_PIN);	// Macro to set the LCD RS pin high (Delete: Should set LCD_RS and only LCD_RS high)
#define lcd_rs_low()    LCD_RS_PORT &= ~(1 << LCD_RS_PIN);	// Macro to set the LCD RS pin low (Delete: Should set LCD_RS and only LCD_RS low)

// Prototype Functions
// LCD Initialization and Setup
void	lcd_init(void);
void	lcd_io_init(void);
void	lcd_reset(void);
void	lcd_configure(void);

// LCD Instruction Functions
void	lcd_send_instruction(uint8_t data);
void	lcd_set_ddram_address(uint8_t address);
void	lcd_clear_display(void);
void	lcd_return_home(void);

// LCD Data Functions
void	lcd_send_data(uint8_t data);
void	lcd_go_to_xy(uint8_t x, uint8_t y);
void	lcd_putc(char c);
void	lcd_puts(const char *s);
void	lcd_put_array(uint8_t *array, uint8_t size);