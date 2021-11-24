/*
 * pata_UART.c
 *
 * Created: 2018-08-30 오후 12:05:03
 *  Author: LEFT
 */ 
#include "UART_328PB.h"
#include <avr/io.h>

volatile uint8_t data = 0;
volatile uint8_t data_m[10] = {0};
uint8_t ubrr = 0;


void UART_init(uint8_t baud, uint8_t mode)
{
	switch (baud)
	{
		case 1:
			ubrr = 51;
			break;
		case 2:
			ubrr = 34;
			break;
		case 3:
			ubrr = 25;
			break;
		case 4:
			ubrr = 12;
			break;
	}
	UBRR0H = (ubrr >> 8);
	UBRR0L = ubrr;
	
	if (mode & TX)
		UCSR0B = (1 << TXEN0);
	if ((mode & RX) || (mode & RXI))
		UCSR0B |= (1 << RXEN0);
	if (mode & RXI)
		UCSR0B |= (1 << RXCIE0);
	
	DDRD &= ~(1<<0);
	DDRD |= (1<<1);
}

void UART_tx(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void UART_tx_m(uint8_t *data, uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = data[i];
	}
}

uint8_t UART_rx()
{
	while (!(UCSR0A & (1<<RXC0)));
	data = UDR0;
	
	return data;
}

void UART_rx_m(uint8_t *data, uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		while(!(UCSR0A & (1<<RXC0)));
		UDR0 = data[i];
	}
}

void UART_NWL()
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = 0x0A;
}