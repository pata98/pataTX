/*
 * pata_LCD1608.c
 *
 * Created: 2021-09-14 오전 1:10:44
 *  Author: yoopata
 */ 

#include "pata_LCD1608.h"
#include "I2C_328PB.h"
#include <util/delay.h>


/**************************************************************/
/*                       LCD Command                          */
/**************************************************************/
void LCD_CMD(uint8_t cmd)
{
	uint8_t tx;
	i2c_start(SLAVE_ADDR<<1);
	
	tx = cmd & 0xF0;
	i2c_write(tx);
	i2c_write(tx|EN);
	i2c_write(tx);
	
	tx = (cmd<<4) & 0xF0;
	i2c_write(tx);
	i2c_write(tx|EN);
	i2c_write(tx);
	
	i2c_stop();
	_delay_us(500);
	
}
void LCD_write(uint8_t data)
{
	uint8_t tx;
	i2c_start(SLAVE_ADDR<<1);
	
	tx = (data & 0xF0)|RS|BCKLIGHT;
	i2c_write(tx);
	i2c_write(tx|EN);
	i2c_write(tx);
	
	tx = ((data<<4) & 0xF0)|RS|BCKLIGHT;
	i2c_write(tx);
	i2c_write(tx|EN);
	i2c_write(tx);
	
	i2c_stop();
	_delay_us(50);
	
}
void LCD_string(char* string, uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		LCD_write(string[i]);
	}
	
}


void LCD_init()
{
	i2c_init();
	_delay_ms(40);
	
	i2c_start(SLAVE_ADDR<<1);
	// Set to 4-bit mode
	i2c_write(0x30);
	i2c_write(0x30|EN);
	i2c_write(0x30);
	_delay_ms(5);
	
	i2c_write(0x30|EN);
	i2c_write(0x30);
	_delay_us(120);
	
	i2c_write(0x30|EN);
	i2c_write(0x30);
	_delay_us(120);
	
	i2c_write(0x20);
	i2c_write(0x20|EN);
	i2c_write(0x20);
	_delay_us(120);
	
	i2c_stop();
	
	
	LCD_CMD(FNC_SET|TWO_LINE);	_delay_us(60);	// Function Set
	LCD_CMD(CMD_ONOFF);			_delay_us(60);	// Display On-Off
	LCD_CMD(CMD_CLR);			_delay_ms(5);	// Clear Display
	LCD_CMD(0x04|CURSOR_INC);	_delay_us(60);	// Entry Mode Set
	
	LCD_CMD(CMD_ONOFF|DISPLAY_ON);
	_delay_us(60);
	
}


void LCD_NWL()
{
	LCD_CMD(0xC0);
}
void LCD_CLEAR()
{
	LCD_CMD(0x01);
	_delay_ms(1);
}
void LCD_home()
{
	LCD_CMD(CMD_HOME);
	_delay_ms(2);
}
void LCD_custom_character(uint8_t* data, uint8_t addr)
{
	LCD_CMD(CGRAM_ADDR + (addr*8));
	
	for (uint8_t i=0; i<8; i++)
	{
		LCD_write(data[i]);
	}
}