/*
 * pata_UART328PB.h
 *
 * Created: 2018-08-30 오전 11:56:35
 *  Author: pata
 *
 */

#ifndef UART_328PB_H_
#define UART_328PB_H_
#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <stdint.h>

#define TX		0x01
#define RX		0x02
#define TXRX	0x03
#define RXI		0x04
#define TXRXI	0x05

#define WRITE	0
#define READ	1

#define BAUD_9600	1
#define BAUD_14k	2
#define BAUD_19k	3
#define BAUD_38k	4


/*******************UART*********************/
// UART initialize
void UART_init(uint8_t baud, uint8_t mode);

// Standard TX/RX
void UART_tx(uint8_t data);
void UART_tx_m(uint8_t *data, uint8_t length);
uint8_t UART_rx();
void UART_rx_m(uint8_t *data, uint8_t length);
void UART_NWL();


#endif /* UART_328PB_H_ */