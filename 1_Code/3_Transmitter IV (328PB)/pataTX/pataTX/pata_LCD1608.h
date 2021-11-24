/*
 * pata_LCD1608.h
 *
 * Created: 2021-04-25 오후 10:48:45
 *  Author: yoopata
 */ 


#ifndef PATA_LCD1608_H_
#define PATA_LCD1608_H_

#include <stdint.h>

//#define SLAVE_ADDR	0x27
#define SLAVE_ADDR	0x3F
#define EN			0x04
#define BCKLIGHT	0x08
#define RS			0x01
#define FNC_SET		0x20

// Command
#define CMD_CLR		0x01
#define CMD_HOME	0x02
#define CMD_MODE	0x04
#define CMD_ONOFF	0x08
#define CMD_SHIFT	0x10
#define CMD_FUNC	0x20
#define CGRAM_ADDR	0x40
#define DDRAM_ADDR	0x80

// Entry Mode
#define DISP_SHIFT	0x01
#define CURSOR_INC	0x02

// Display Control
#define BLINK_ON	0x01
#define CURSOR_ON	0x02
#define DISPLAY_ON	0x04

// Cursor/Display Shift
#define CURSOR_R	0x04
#define CURSOR_SHIFT	0x08

// Function
#define TWO_LINE	0x08
#define DATA_8BIT	0x10

/**************************************************************/
void LCD_CMD(uint8_t cmd);
void LCD_write(uint8_t data);
void LCD_string(char* string, uint8_t length);
void LCD_init();

void LCD_NWL();
void LCD_CLEAR();
void LCD_home();
void LCD_custom_character(uint8_t* data, uint8_t addr);





#endif /* PATA_LCD1608_H_ */