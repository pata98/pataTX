  /*********************************************************************************

  Transmitter IV.c

	Created: 2017-09-06 오전 11:23:05
	Author : pata
	
	Edit history:
	2020.01.18 - pwrdet, pwrtolg pin change
	2020.10.01 - ???
	2021.01.14 - Comment rewritten, overall pin map changed for v2.0 board, change switch input method
	2021.09.14 - mcu changed to Atmega328PB
	2021.11.20 - EEPROM data map changed (switch function assginment memory sw[] added)
	
	
	Pata RC Transmitter MK.IV
	
	FEATURE :										
	- 5ch	:	throttle (1)
				rudder	 (2)
				elevator (3)
				aileron  (4)
				CH5	 (5)
	
	-  Reverse mixing
	-  Elevon mixing
  	-  Dual rate
	-  Servo delay
	-  8 model memory
  	-  Digital real-time trim
	-  Transmitter battery level check & low voltage alarm
	-  LCD screen




									=======	Atmega328PB pin map =======
					 Telemetry
					     CE		TX		RX	   Reset	SCL	   SDA	  ADC_Thr  PWRDET
						             		
						PD2=====PD1=====PD0=====PC6=====PC5=====PC4=====PC3=====PC2
				    =															   =
	Module	SS	PD3																	 PC1	ADC_Ail
				 =																	  =
	   PWRTOGL  PD4																	 PC0	ADC_Rud
				 =																	  =
		  AUX	PE0																	 PE3   ADC_Ele
				 =																	  =
		  VCC	VCC																 	 GND	GND
				 =																	  =
		  GND	GND								 ATMEGA328PB						 AREF	AREF
				 =																	  =
		  PO4	PE1																	 PE2	Battery ADC
				 =																	  =
		  PI4	PB6																	 AVCC	AVCC
				 =																	  =
		  PI3	PB7																	 PB5	SCK
				 =																	  =
						PD5		PD6		PD7		PB0		PB1		PB2		PB3		PB4
	 
						PO1	    PO2     PO3		PI1	   Buzzer	PI2		MOSI	MISO




  								=======	Switch map =======
  
  											OUT
	
  								PO1				PO2				PO3			PO4
 	
  					PI1		Rud trim -		Ail trim -			CH5			SW2+
 
 					PI2		Rud trim +		Ail trim +		   Rud DR		SW2-
 			IN
  					PI3		Ele trim -		  Menu 2		   Ele DR		SW3+
  
  					PI4		ELe trim +		  Menu 1		    SW1			SW3-
  



						======= EEPROM memory adress map =======

			 1~15: model 1		    16~30: model 2			 31~45: model 3
				1: trim[0]				16: trim[0]				31: trim[0]
				2: trim[1]				17: trim[1]				32: trim[1]
				3: trim[3]				18: trim[3]				33: trim[3]
				4: trim[4]				19: trim[4]				34: trim[4]
				5: mixing_val_1			20: mixing_val_1		35: mixing_val_1
				6: mixing_val_2			21: mixing_val_2		36: mixing_val_2
				7: mixing_val_3			22: mixing_val_3		37: mixing_val_3
				8: sw[0]				23: sw[0]				38: sw[0]
				9: sw[1]				24: sw[1]				39: sw[1]
				10: sw[2]				25: sw[2]				40: sw[2]
				11:sw[3]				26: sw[3]				41: sw[3]
				12:sw[4]				27: sw[4]				42: sw[4]
				13:sw[5]				28: sw[5]				43: sw[5]
				14:sw[6]				29: sw[6]				44: sw[6]
				15:sw[7]				30: sw[7]				45: sw[7]
				
			 46~60: model 4			 61~75: model 5			 76~90: model 6
				46: trim[0]				61: trim[0]				76: trim[0]
				47: trim[1]				62: trim[1]				77: trim[1]
				48: trim[3]				63: trim[3]				78: trim[3]
				49: trim[4]				64: trim[4]				79: trim[4]
				50: mixing_val_1		65: mixing_val_1		80: mixing_val_1
				51: mixing_val_2		66: mixing_val_2		81: mixing_val_2
				52: mixing_val_3		67: mixing_val_3		82: mixing_val_3
				53: sw[0]				68: sw[0]				83: sw[0]
				54: sw[1]				69: sw[1]				84: sw[1]
				55: sw[2]				70: sw[2]				85: sw[2]
				56: sw[3]				71: sw[3]				86: sw[3]
				57: sw[4]				72: sw[4]				87: sw[4]
				58: sw[5]				73: sw[5]				88: sw[5]
				59: sw[6]				74: sw[6]				89: sw[6]
				60: sw[7]				75: sw[7]				90: sw[7]
				
	 		91~105: model 7		   106~120: model 8		
				91: trim[0]				106: trim[0]			
				92: trim[1]				107: trim[1]			
				93: trim[2]				108: trim[2]
				94: trim[4]				109: trim[4]		
				95: mixing_val_1		110: mixing_val_1	
				96: mixing_val_2		111: mixing_val_2
				97: mixing_val_3		112: mixing_val_3	
				98: sw[0]				113: sw[0]
				99: sw[1]				114: sw[1]
				100:sw[2]				115: sw[2]
				101:sw[3]				116: sw[3]
				102:sw[4]				117: sw[4]
				103:sw[5]				118: sw[5]
				104:sw[6]				119: sw[6]
				105:sw[7]				120: sw[7]
				
			121	: pre_model			122	: volume			123	: runtime_set
		
		
	
*********************************************************************************/ 

#define F_CPU 8000000UL 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "UART_328PB.h"
#include "pata_LCD1608.h"


////////// Pin Number ////////////////////////////////////////////////////
#define MODULE		0x08
#define TELE		0x04
#define BUZZER		0x02
#define PWRTOGL		0x10
#define PWRDET		0x04

#define PI1			0x01
#define PI2			0x04
#define PI3			0x80
#define PI4			0x40
#define PO1			0x20
#define PO2			0x01
#define PO3			0x80
#define PO4			0x02
#define AUX			0x01


////////// Status ////////////////////////////////////////////////////////
#define MAX			255
#define ON			1
#define OFF			0
#define STBY		2
#define RUNTIME30	1800
#define RUNTIME15	900
#define RUNTIME10	600
#define RUNTIME10s	10
#define MENU_NUM	12

// Gimbal range
#define THROTTLE_MIN	50
#define THROTTLE_MAX	210

// Battery level
#define HIGH		4
#define MID			3
#define LOW			190		// 3.7 V (At ADC Pin, 3.8 V)
#define CRIT		180		// 3.5 V (At ADC Pin, 3.6 V)
#define TRIM_LIM	9		// Maximum number of trim
#define ALPHA		0.97

// Battery charger
#define CHG_30MAH	0x04
#define CHG_85MAH	0x08
#define CHG_120MAH	0x0C
#define CHG_150MAH	0x10
#define CHG_300MAH	0x14

// RF Mode
#define JF24		1
#define NRF			2
#define UART		3
#define BLE			4

// Channel number
#define CHANNEL		5		// Total channel number
#define AILERON		0
#define ELEVATOR	1
#define THROTTLE	2
#define RUDDER		3
#define CH5			4
#define CH5_SW		0x20


////////// Bit Mask //////////////////////////////////////////////////////
// tel_toggle
#define BIT1		0x01
#define BIT2		0x02
#define BIT3		0x04
#define BIT4		0x08

// switch_val
#define SW1			0x01	// BIT0
#define SW2			0x02
#define SW3			0x04
#define SW4			0x08
#define SW5			0x10
#define SW6			0x20
#define SW7			0x40
#define SW8			0x80	// BIT7

// battery_alarm
#define ALARM_LOW	0x01	// BIT 0
#define ALARM_CRIT	0x02
#define ALARM_TOGL	0x04

// buzzer_setting
#define PERIOD		0		// buzzer_setting[0]
#define SETTING		1		// buzzer_setting[1]
// buzzer_setting[SETTING]
#define REPEAT0		0x01	// BIT 0
#define REPEAT1		0x02		
#define REPEAT2		0x04
#define BUZZER_SET	0x08
#define INIT		0x10	
#define STAT		0x20	// BIT 4

// mixing_val_1
#define REVERSE_AIL	0x01	// BIT 0
#define REVERSE_ELE	0x02
#define REVERSE_RUD	0x04
#define REVERSE_CH5 0x08
#define DIFF_THRUST	0x10
#define ELEVON		0x20
#define VTAIL		0x40
#define FLAPERON	0x80	// BIT 7

// mixing_val_2
#define AIL_DR		0x01	// BIT 0
#define ELE_DR		0x02
#define THR_DR		0x04
#define RUD_DR		0x08
#define CH5_DR		0x10
#define MODEL0		0x20
#define MODEL1		0x40
#define MODEL2		0x80	// BIT 7
#define DR_MASK		0x1F

// mixing_val_3
#define RF_MODE_1	0x01	// BIT 0
#define RF_MODE_2	0x02
#define RF_MODE_3	0x04
#define	RF_MODE_4	0x08
#define SERVO_DEL_1	0x10
#define SERVO_DEL_2	0x20
#define USER_MIX	0x40	// BIT 6
#define RF_MODE_MASK 0x0F	// RF_MODE bit mask

// batCharger
//#define ON			0x01	// BIT 0
#define BATCHARG_STAT	0x02
#define BATCHARG_CAP1	0x04
#define BATCHARG_CAP2	0x08
#define BATCHARG_CAP3	0x10	// BIT 5
#define BATCHARG_CAP_MASK	0x1C

////////// Function Prototype ////////////////////////////////////////////
// Setting
void init();

// Menu	
void model_sel();
uint8_t menu();

// Input
void stick_read();
void trim_read();
void switch_read();
uint8_t menu_read();
void make_val();
void set_switch();
uint8_t transmitter_bat_chk();

// RF
void make_ppm();
void JF24_RF();

// Interface
void buzzer(uint16_t freq);
void buzzer_switch();
void power_off();
void LCD_guage(uint8_t val, uint8_t position);



////////// Variables /////////////////////////////////////////////////////
// Timer
volatile uint8_t timer_24ms	= 0;
volatile uint8_t timer_8ms_Ch5Delay = 0;	// servo 1 delay timer
volatile uint8_t timer_8ms_buzzer = 0;
volatile uint8_t timer_8ms = 0;
volatile uint16_t runtime = 0;
volatile uint16_t runtime_set;
volatile uint8_t timer_8ms_power = 0;

// Telemetry - Used for JF24 RF Module
volatile uint8_t receiver_bat = 0;		// receiver battery level
volatile uint8_t tel_timer	= 0;
volatile uint8_t tel_flg	= OFF;
volatile uint8_t tel_toggle = OFF;

// Channel value
uint8_t channel[CHANNEL] = {0};
int8_t  trim[CHANNEL]	 = {0};
uint8_t channel_mix_temp = 0;

// Switch
uint8_t button_tog[9] = {OFF};		// Check toggle button is pressed
uint8_t buzzer_volume;
uint8_t buzzer_setting[2] = {0};
uint16_t buzzer_freq;
uint8_t power_sw_toggle = OFF;

// Model setting value
volatile uint8_t switch_val = 0;
uint8_t mixing_val_1, mixing_val_2;
volatile uint8_t mixing_val_3;
uint8_t sw[8];

// Menu index
uint8_t pre_model ,cur_model;
int8_t  menu_input;
uint8_t batCharger = OFF;

// Battery
float   TX_Bat_est; 
uint8_t TX_bat_mes;
uint8_t TX_bat_cnt_1 = 0;
uint8_t battery_alarm = ALARM_TOGL;
uint8_t buzzer_flg = OFF;

// LCD
// Custom character
//	lcd_guage1		lcd_guage2		lcd_guage3		lcd_guage4		lcd_guage5		lcd_guage6		lcd_guage7		lcd_guage8
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	|=		  |		|= =	  |		|= = =	  |		|= = = =  |		|  = = = =|		|	 = = =|		|	   = =|		|		 =|
//	 - - - - -		 - - - - -		 - - - - -		 - - - - -		 - - - - -		 - - - - -		 - - - - -		 - - - - -

uint8_t lcd_bat[3];
char* lcd1 = "MODELX  X  X.XXV";
char* lcd2 = "A   E   R       ";
uint8_t lcd_guage0[] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
uint8_t lcd_guage1[] = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
uint8_t lcd_guage2[] = {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C};
uint8_t lcd_guage3[] = {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E};
uint8_t lcd_guage4[] = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
uint8_t lcd_guage5[] = {0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07};
uint8_t lcd_guage6[] = {0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03};
uint8_t lcd_guage7[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};



/****************************************************/
/*     Main											*/
/*													*/
/****************************************************/
int main(void)
{
	////////// Read EEPROM Setting Data //////////////////////////////////////
	model_sel();
	
	
	////////// Initialize ////////////////////////////////////////////////////
	init();
	
	LCD_init();
	LCD_custom_character(lcd_guage0, 0);
	LCD_custom_character(lcd_guage1, 1);
	LCD_custom_character(lcd_guage2, 2);
	LCD_custom_character(lcd_guage3, 3);
	LCD_custom_character(lcd_guage4, 4);
	LCD_custom_character(lcd_guage5, 5);
	LCD_custom_character(lcd_guage6, 6);
	LCD_custom_character(lcd_guage7, 7);
	
	UART_init(BAUD_14k, TXRX);
	
	sei();
	
	
	////////// Power On //////////////////////////////////////////////////////
	// Turn Transmitter on if button is pressed for 0.5 s
	LCD_home();
	LCD_string("    PATA TX     ", 16);
	LCD_NWL();
	for (uint8_t i = 0; i < 16; i++)
	{
		LCD_write(0xFF);
		_delay_ms(31);
		
		if (PINC & PWRDET)
			power_off();
	}
	power_sw_toggle = STBY;
	PORTD |= PWRTOGL;
	LCD_CLEAR();
	
	
	////////// Throttle Safety Check//////////////////////////////////////////
	// If Throttle is not 0 (>5), do not boot the transmitter
	while (1)
	{
		stick_read();
		if (channel[THROTTLE] < THROTTLE_MIN)
		{
			power_sw_toggle = STBY;
			break;
		}
		else
		{
			LCD_string(" THROTTLE HIGH! ", 16);
			LCD_NWL();
			LCD_string(" MENU TO IGNORE ", 16);
			buzzer_freq = 1000;
			buzzer_setting[PERIOD] = 50;
			buzzer_setting[SETTING] |= BUZZER_SET;
			
		}
		// If menu buttons are pressed, ignore throttle lock
		if (menu_read())
		{
			power_sw_toggle = STBY;
			break;
		}		
		
		////////// Power Off//////////////////////////////////////////////////////
		// Button release detect
		// Without this, transmitter would be turned on and turned off immediately
		if ((power_sw_toggle == STBY) && (PINC & PWRDET))
			power_sw_toggle = OFF;
			
		// Power off
		if (!(PINC & PWRDET) && (power_sw_toggle == OFF))
		{
			power_sw_toggle = ON;
			timer_8ms_power = 0;
		}
		else if ((timer_8ms_power > 125) && (power_sw_toggle == ON))
		{
			// Power off only if button is pressed for 1 s
			if (PINC & PWRDET)
				power_sw_toggle = OFF;
			else
				power_off();
			
		}
		
		buzzer_switch();
		PORTD |= PWRTOGL;
		
	} // while (1)
	
	
	////////// Booting Music /////////////////////////////////////////////////
	buzzer(0);
	_delay_ms(100);
	buzzer(1000);
	_delay_ms(100);
	buzzer(0);
	_delay_ms(50);
	buzzer(1000);
	_delay_ms(100);
	buzzer(0);
	
	
	// TX battery check
	//TX_Bat_est = transmitter_bat_chk();
	TX_Bat_est = transmitter_bat_chk();
	
	// Main loop
	LCD_CLEAR();
	while (1)
	{
		////////// Enter Menu Setting Mode ///////////////////////////////////////
		// If M1 & M2 is pressed together, enter menu setting mode
		// Since there are two display modes, limit input to two
		if (menu_read())
		{
			buzzer(1500);
			_delay_ms(150);
			buzzer(0);
			menu();
		}
		// Menu input limit
		// 0: main page
		// 1: Monitor page - (aileron, elevator, throttle, rudder)
		// 2: Monitor page - (throttle, rudder, CH5)
		if (menu_input < 0)
			menu_input = 0;
		else if (menu_input > 2)
			menu_input = 2;
			
		
		////////// Power managing ////////////////////////////////////////////////
		// If power switch is off, save trim values and turn transmitter off
		if ((power_sw_toggle == STBY) && (PINC & PWRDET))
			power_sw_toggle = OFF;
		if (!(PINC & PWRDET) && (power_sw_toggle == OFF))
		{
			power_sw_toggle = ON;
			timer_8ms_power = 0;
		}
		else if ((timer_8ms_power > 125) && (power_sw_toggle == ON))
		{
			// Power off only if button is pressed for 1 s
			if (PINC & PWRDET)
				power_sw_toggle = OFF;
			else
				power_off();
		}
		
		// Low Battery Shutdown
		// If battery is lower than 3.7 V, ring buzzer 5 times
		// If battery is lower than 3.5 V, the device is powered off.
		TX_bat_cnt_1++;
		if (TX_bat_cnt_1 > 50)
		{
			TX_bat_cnt_1 = 0;
			TX_bat_mes = transmitter_bat_chk();
			TX_Bat_est = ALPHA*TX_Bat_est + (1-ALPHA)*TX_bat_mes;	// Exponentially weighted moving average filter
			
			// Battery 3.7 V, alarm 4 times
			if (((TX_Bat_est) < LOW) && (battery_alarm & ALARM_TOGL))
			{
				battery_alarm &= ~ALARM_TOGL;
				
				// buzzer setting
				buzzer_freq = 2000;
				buzzer_setting[PERIOD] = 50;
				buzzer_setting[SETTING] = BUZZER_SET|4;	// Repeat 5 times
					
			}
			// Battery 3.5 V, turn off
			else if ((TX_Bat_est) < CRIT)
			{
				buzzer(2000);
				_delay_ms(1000);
				buzzer(0);
				
				//power_off();
			}
			
		}
		
		// If runtime > RUMTIME, ring buzzer 2 times
		// If runtime is longer than 15 min, turn the device off
// 		if (runtime > runtime_set)
// 			power_off();
		if ((runtime > RUNTIME10) && (buzzer_flg == OFF))
		{
			buzzer_flg = ON;			
			buzzer_freq = 2000;
			buzzer_setting[PERIOD] = 31;
			buzzer_setting[SETTING] |= BUZZER_SET|1;	// Repeat 2 times
		}
		
		
		////////// Read Inputs & Process Data ////////////////////////////////////
		stick_read();
		switch_read();
		trim_read();
		set_switch();
		make_val();
		
		
		////////// Transmit Data /////////////////////////////////////////////////
		// PPM mode
		if ((mixing_val_3 & RF_MODE_MASK) == JF24)
		{
			JF24_RF();
		}
		// Debug mode (UART mode)
		else if ((mixing_val_3 & RF_MODE_MASK) == UART)
		{
			UART_NWL();
			UART_NWL();
			UART_tx_m(channel, 5);
			_delay_ms(10);
		}
		// NRF24l01 module mode
		else if ((mixing_val_3 & RF_MODE_MASK) == NRF)
		{
			asm("nop"); // Todo
		}
		// BLE module mode
		else if ((mixing_val_3 & RF_MODE_MASK) == BLE)
		{
			asm("nop"); // Todo
		}
		else
		{
			asm("nop");
		}
		
		
		////////// LCD ///////////////////////////////////////////////////////////
		// Main Page //////////////////////
		if (menu_input == 0)
		{
			lcd1 = "MODELX  X  X.XXV";
			lcd2 = "A   E   R       ";
			
			// First line
			lcd1[5] = cur_model + 49;
			switch (mixing_val_3 & RF_MODE_MASK)
			{
				case JF24:
					lcd1[8] = 'J';
					break;
				
				case UART:
					lcd1[8] = 'U';
					break;
				
				case NRF:
					lcd1[8] = 'N';
					break;
				
				case BLE:
					lcd1[8] = 'B';
					break;
				
				default:
					lcd1[8] = 'D';
					break;
			}
			
			// TX battery voltage - calculate each digit
			uint8_t temp_TX_Bat_est;
			temp_TX_Bat_est = (uint8_t)TX_Bat_est;
			
			lcd_bat[0] = (5*temp_TX_Bat_est)>>8;
			lcd_bat[1] = 5*temp_TX_Bat_est/25.6 - lcd_bat[0]*10;
			lcd_bat[2] = (float)(5*temp_TX_Bat_est/2.56 - lcd_bat[1]*10 - lcd_bat[0]*100);
			
			lcd1[11] = lcd_bat[0] + 48;
			lcd1[13] = lcd_bat[1] + 48;
			lcd1[14] = lcd_bat[2] + 48;
			
			
			// Second line
			if (trim[AILERON] >= 0)
			{
				lcd2[1] = trim[AILERON] + 48;
				lcd2[2] = ' ';
			}
			else
			{
				lcd2[1] = '-';
				lcd2[2] = 48 - trim[AILERON];
			}
			
			if (trim[ELEVATOR] >= 0)
			{
				lcd2[5] = trim[ELEVATOR] + 48;
				lcd2[6] = ' ';
			}
			else
			{
				lcd2[5] = '-';
				lcd2[6] = 48 - trim[ELEVATOR];
			}
			
			if (trim[RUDDER] >= 0)
			{
				lcd2[9] = trim[RUDDER] + 48;
				lcd2[10] = ' ';
			}
			else
			{
				lcd2[9] = '-';
				lcd2[10] = 48 - trim[RUDDER];
			}
		}
		
		// Channel Monitor Page ///////////
		else if (menu_input == 1)
		{
			lcd1 = "A       E       ";
			lcd2 = "T       R       ";
			
			// Aileron
			LCD_guage(channel[AILERON],  0);
			LCD_guage(channel[ELEVATOR], 1);
			LCD_guage(channel[THROTTLE], 2);
			LCD_guage(channel[RUDDER],   3);
			
		}
		else if (menu_input == 2)
		{
			lcd1 = "T       R       ";
			lcd2 = "G               ";
			
			// Aileron
			LCD_guage(channel[THROTTLE], 0);
			LCD_guage(channel[RUDDER],   1);
			LCD_guage(channel[CH5],     2);
			
		}
		
		
		
		// Print
		LCD_home();
		LCD_string(lcd1, 16);
		LCD_NWL();
		LCD_string(lcd2, 16);
		
		
		////////// Buzzer ////////////////////////////////////////////////////////
		buzzer_switch();
	
	}//while(1)

}//main



/****************************************************/
/*      8ms timer									*/
/*													*/
/****************************************************/
ISR (TIMER0_OVF_vect)
{
	timer_8ms++;
	timer_24ms++;
	tel_timer++;
	timer_8ms_buzzer++;
	timer_8ms_power++;
	TCNT0 = 5;
	
	// Servo Delay
	if (mixing_val_3 & SERVO_DEL_1)
	{
		if (channel_mix_temp & CH5_SW)
		{
			// CH5 down
			if (timer_8ms_Ch5Delay > 5)
				timer_8ms_Ch5Delay--;
		}
		else
		{			
			// CH5 up
			if (timer_8ms_Ch5Delay < 250)
				timer_8ms_Ch5Delay++;
		}
	}
	
	// 1s counter
	if (timer_8ms > 123)
	{
		runtime++;
		if (runtime > runtime_set)
			power_off();
		timer_8ms = 0;
	}
	
}


/****************************************************************/
/*      JF24 RF Telemetry										*/
/*																*/
/*	   This function measures the time that battery				*/
/*	   monitor pin is high. Duty cycle of Bat monitor			*/
/*	   led in JF module is proportional to receiver				*/
/*	   battery level. Length of time that the pin is			*/
/*	   high is as follows:										*/
/*																*/
/*																*/
/*		_|``````|_____________									*/
/*			a			b										*/
/*																*/	
/*	Battery level:												*/
/*																*/
/*	High		4.1V ~ 4.2V		a: 95ms		b: 666ms			*/
/*	Mid			3.9V ~ 4.0V		a: 286ms	b: 476ms			*/	
/*	Low			3.7V ~ 3.8V		a: 476ms	b: 286ms			*/
/*	Critical	3.4V ~ 3.6V		a: 572ms	b: 190ms			*/
/*																*/
/*	8ms timer:													*/
/*																*/
/*	High		a: 11		b: 83								*/
/*	Mid			a: 35		b: 59								*/
/*	Low			a: 59		b: 35								*/
/*	Critical	a: 71		b: 23								*/
/*																*/
/*																*/
/*		In order to receive only useful data, garbage			*/
/*	   data is ignored by detecting initializing signal.		*/
/*	   Initializing signal: 761ms of low						*/
/*																*/
/*		|'|_|''|_|'|___________________|```````|______			*/
/*						Init: 761ms								*/
/*																*/
/****************************************************************/
ISR (INT0_vect)
{
	// Wait for initialize signal
	if (tel_toggle & BIT1)
	{
		// initiate timer for the first time only
		if (!(tel_toggle & BIT2))
		{
			tel_timer = 0;
			tel_toggle |= BIT2;
		}
		
		// wait 752ms
		if (tel_timer > 94)	 // 94 * 8 = 752ms
			tel_toggle |= BIT1;
	}
	
	// Measuring routine
	else
	{
		// Check the pin is high or low
		if (tel_flg == OFF)
			tel_flg = ON;
		else
			tel_flg = OFF;
		
		// Measure time the pin is high
		if (tel_flg == ON)
		{
			if (tel_timer > 10)
				receiver_bat = CRIT;
			if (tel_timer > 32)
				receiver_bat = LOW;
			if (tel_timer > 56)
				receiver_bat = MID;
			if (tel_timer > 69)
				receiver_bat = HIGH;
		}
	}
	tel_timer = 0;
	
}


/****************************************************/
/*      Initialize MCU setting						*/
/*													*/
/****************************************************/
void init()
{
	// Pin IO setting
	DDRB	= 0x2A;
	DDRC	= 0x30;
	DDRE	= 0x03;
	PORTB   = 0xC5;		// Button matrix pull up
	
	if ((mixing_val_3 & RF_MODE_MASK) == JF24)
	{
		DDRD = 0xFA;
		
		// External interrupt setting
		// Enable external pin interrupt when using JF24 RF module for telemetry
		EICRA = (1 << ISC00);	// any change of logic
		EIMSK = (1 << INT0);	// using INT0
	}
	else
	{
		DDRD	= 0xFE;
	}
	PORTC   = PWRDET;	// Power detect pull up
	PORTD   = PWRTOGL;	// SW power on
	
	
	// Timer0 interrupt setting
	TCCR0B	= (1 << CS02);	// prescalar: 256
	TIMSK0	= (1 << TOIE0);	// start interrupt
	
	
	// Buzzer PWM
	// Phase Correct PWM mode - to prevent glitch
	TCCR1A  = (1 << WGM11) |
			  (1 << COM1A1)| (1 << COM1A0);	// OC1A non-inverting mode
	TCCR1B  = (1 << WGM13) | (1 << CS11);   // prescalar: 8
	
}


/****************************************************/ 
/*		Model setting								*/
/*													*/
/*	- model_sel										*/
/*	1. Recall previous model						*/
/*													*/
/*	- menu											*/
/*	1. Model select									*/
/*	2. Channel assignment change					*/
/*	3. Mixing										*/
/*	4. Reverse										*/
/*	5. Dual rate setting							*/
/*													*/
/****************************************************/
void model_sel()
{
	uint8_t pre_model_addr;
	
	// Recall latest model data
	pre_model = eeprom_read_byte((uint8_t*)121);
	cur_model = pre_model;
	
	if (pre_model < 8)
	{
		pre_model_addr = pre_model * 15 + 1;	// latest model address
		
		// Setting initializing
		trim[AILERON]  = eeprom_read_byte((const uint8_t*)pre_model_addr      );
		trim[ELEVATOR] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 1));
		trim[RUDDER]   = eeprom_read_byte((const uint8_t*)(pre_model_addr + 2));
		trim[CH5]	   = eeprom_read_byte((const uint8_t*)(pre_model_addr + 3));
		mixing_val_1 = eeprom_read_byte((const uint8_t*)(pre_model_addr + 4));
		mixing_val_2 = eeprom_read_byte((const uint8_t*)(pre_model_addr + 5));
		mixing_val_3 = eeprom_read_byte((const uint8_t*)(pre_model_addr + 6));
		sw[0] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 7));
		sw[1] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 8));
		sw[2] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 9));
		sw[3] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 10));
		sw[4] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 11));
		sw[5] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 12));
		sw[6] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 13));
		sw[7] = eeprom_read_byte((const uint8_t*)(pre_model_addr + 14));
		
		buzzer_volume= eeprom_read_byte((const uint8_t*)122);
		runtime_set = eeprom_read_word((const uint16_t*)123);
		
	}
	
	// Initial EEPROM
	// Only used when new firmware is downloaded
	else
	{
		eeprom_update_byte((uint8_t*)1, 0x00);	// trim[AILERON] = 0
		eeprom_update_byte((uint8_t*)2, 0x00);	// trim[ELEVATOR] = 0
		eeprom_update_byte((uint8_t*)3, 0x00);	// trim[RUDDER] = 0
		eeprom_update_byte((uint8_t*)4, 0x00);	// trim[CH5] = 0
		eeprom_update_byte((uint8_t*)5, 0x00);	// mixing_val_1 = 0
		eeprom_update_byte((uint8_t*)6, 0x00);	// mixing_val_2 = 0
		eeprom_update_byte((uint8_t*)7, UART);	// mixing_val_3 = UART mode
		eeprom_update_byte((uint8_t*)8, 0x00);	// sw[0]
		eeprom_update_byte((uint8_t*)9, 0x00);	// sw[1]
		eeprom_update_byte((uint8_t*)10, 0x00);	// sw[2]
		eeprom_update_byte((uint8_t*)11, 0x00);	// sw[3]
		eeprom_update_byte((uint8_t*)12, 0x00);	// sw[4]
		eeprom_update_byte((uint8_t*)13, 0x00);	// sw[5]
		eeprom_update_byte((uint8_t*)14, 0x00);	// sw[6]
		eeprom_update_byte((uint8_t*)15, 0x00);	// sw[7]
		
		eeprom_update_byte((uint8_t*)121, 0x00);	  // Set model #0 current model
		eeprom_update_byte((uint8_t*)122, 50);		  // Buzzer volume
		eeprom_update_word((uint16_t*)123, RUNTIME15); // RUNTIMR 15 min
		
	}
	
}
uint8_t menu()
{
	int8_t menu_input_temp;
	char* menu_display_h;
	char* menu_display_l;
	uint8_t enter = 0;
	uint8_t addr;
	uint8_t pre_addr, pre_mod;
	menu_input = 0;
	LCD_CLEAR();
	
	while (1)
	{
		// Cursor input
		enter = menu_read();
		if (menu_input < 0)
			menu_input = MENU_NUM;
		else if (menu_input > MENU_NUM)
			menu_input = 0;


		////////// Set model number //////////////////////////////////////////////
		if (menu_input == 0)
		{
			// Display code 
			menu_display_h = ">Model Number   ";
			menu_display_l = " Mixing         ";
			pre_mod = cur_model;
			
			// Enter Model Number mode
			if (enter)
			{
				menu_display_h = " Model Number   ";
				menu_display_l = " Model     Exit ";
				
				menu_display_l[7] = cur_model + 49;
				
				LCD_home();
				LCD_string(menu_display_h, 16);
				
				while (1)
				{
					// Cursor limit
					enter = menu_read();
					if (menu_input < 0)	
						menu_input = 0;
					else if (menu_input > 1)
						menu_input = 1;
					
					// Model number select
					if (menu_input == 0)
					{
						menu_display_l[0]  = '>';
						menu_display_l[10] = ' ';
						
						if (enter)
						{
							menu_input = cur_model;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 7)
									menu_input = 7;
								
								cur_model = menu_input;
								menu_display_l[7] = menu_input + 49;
								
								if (enter)
								{
									menu_input = 0;
									break;
								}
								
								// Display
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								buzzer_switch();
								
							}
						}
					}
					// Exit
					else if (menu_input == 1)
					{
						menu_display_l[0]  = ' ';
						menu_display_l[10] = '>';
						
						if (enter)
						{
							// Save previous model data
							pre_addr = pre_mod * 15 + 1;
							
							eeprom_update_byte((uint8_t*)pre_addr,       trim[AILERON]);
							eeprom_update_byte((uint8_t*)(pre_addr + 1), trim[ELEVATOR]);
							eeprom_update_byte((uint8_t*)(pre_addr + 2), trim[RUDDER]);
							eeprom_update_byte((uint8_t*)(pre_addr + 3), trim[CH5]);
							
							eeprom_update_byte((uint8_t*)(pre_addr + 4), mixing_val_1);
							eeprom_update_byte((uint8_t*)(pre_addr + 5), mixing_val_2);
							eeprom_update_byte((uint8_t*)(pre_addr + 6), mixing_val_3);
							
							eeprom_update_byte((uint8_t*)(pre_addr + 7),  sw[0]);
							eeprom_update_byte((uint8_t*)(pre_addr + 8),  sw[1]);
							eeprom_update_byte((uint8_t*)(pre_addr + 9),  sw[2]);
							eeprom_update_byte((uint8_t*)(pre_addr + 10), sw[3]);
							eeprom_update_byte((uint8_t*)(pre_addr + 11), sw[4]);
							eeprom_update_byte((uint8_t*)(pre_addr + 12), sw[5]);
							eeprom_update_byte((uint8_t*)(pre_addr + 13), sw[6]);
							eeprom_update_byte((uint8_t*)(pre_addr + 14), sw[7]);
							
							eeprom_update_byte((uint8_t*)121, cur_model);
							
							// Call new model data
							addr = cur_model * 15 + 1;
							
							trim[AILERON]  = eeprom_read_byte((const uint8_t*)addr      );
							trim[ELEVATOR] = eeprom_read_byte((const uint8_t*)(addr + 1));
							trim[RUDDER]   = eeprom_read_byte((const uint8_t*)(addr + 2));
							trim[CH5]      = eeprom_read_byte((const uint8_t*)(addr + 3));
							
							mixing_val_1 = eeprom_read_byte((const uint8_t*)(addr + 4));
							mixing_val_2 = eeprom_read_byte((const uint8_t*)(addr + 5));
							mixing_val_3 = eeprom_read_byte((const uint8_t*)(addr + 6));
							
							sw[0] = eeprom_read_byte((const uint8_t*)(addr + 7));
							sw[1] = eeprom_read_byte((const uint8_t*)(addr + 8));
							sw[2] = eeprom_read_byte((const uint8_t*)(addr + 9));
							sw[3] = eeprom_read_byte((const uint8_t*)(addr + 10));
							sw[4] = eeprom_read_byte((const uint8_t*)(addr + 11));
							sw[5] = eeprom_read_byte((const uint8_t*)(addr + 12));
							sw[6] = eeprom_read_byte((const uint8_t*)(addr + 13));
							sw[7] = eeprom_read_byte((const uint8_t*)(addr + 14));
							
							menu_input = 0;
							break;
						}
					}
					
					
					// Display
					LCD_home();
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					
					buzzer_switch();
					
				} // while
			} // if (enter)
		} // if (menu_input == 0)
		
		
		////////// Mixing setting ////////////////////////////////////////////////
		else if (menu_input == 1)
		{
			// Display code
			menu_display_h = " Model Number   ";
			menu_display_l = ">Mixing         ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				
				while (1)
				{
					// Cursor limit
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 5)
						menu_input = 5;
					
					
					// Display & stat /////////////
					if (menu_input == 0)
					{
						// Display init
						menu_display_h = ">Elevon         ";
						menu_display_l = " Vtail          ";
						
						if ((enter) && !(mixing_val_1 & VTAIL))
						{
							if (mixing_val_1 & ELEVON)
								mixing_val_1 &= ~ELEVON;
							else
								mixing_val_1 |= ELEVON;
						}
						
						if (mixing_val_1 & ELEVON)
							menu_display_h[10] = 'V';
						else
							menu_display_h[10] = ' ';
						if (mixing_val_1 & VTAIL)
							menu_display_l[10] = 'V';
						else
							menu_display_l[10] = ' ';
						
					}
					else if (menu_input == 1)
					{
						// Display init
						menu_display_h = " Elevon         ";
						menu_display_l = ">Vtail          ";
						
						if ((enter) && !(mixing_val_1 & ELEVON))
						{
							if (mixing_val_1 & VTAIL)
								mixing_val_1 &= ~VTAIL;
							else
								mixing_val_1 |= VTAIL;
						}
						
						if (mixing_val_1 & ELEVON)
							menu_display_h[10] = 'V';
						else
							menu_display_h[10] = ' ';
						if (mixing_val_1 & VTAIL)
							menu_display_l[10] = 'V';
						else
							menu_display_l[10] = ' ';
							
					}
					else if (menu_input == 2)
					{
						// Display init
						menu_display_h = " Vtail          ";
						menu_display_l = ">Diff Thrust    ";
						
						if (enter)
						{
							if (mixing_val_1 & DIFF_THRUST)
								mixing_val_1 &= ~DIFF_THRUST;
							else
								mixing_val_1 |= DIFF_THRUST;
						}
						
						if (mixing_val_1 & VTAIL)
							menu_display_h[10] = 'V';
						else
							menu_display_h[10] = ' ';
						if (mixing_val_1 & DIFF_THRUST)
							menu_display_l[13] = 'V';
						else
							menu_display_l[13] = ' ';
						
					}
					else if (menu_input == 3)
					{
						// Display init
						menu_display_h = " Diff Thrust    ";
						menu_display_l = ">Flaperon       ";
						
						if (enter)
						{
							if (mixing_val_1 & FLAPERON)
								mixing_val_1 &= ~FLAPERON;
							else
								mixing_val_1 |= FLAPERON;
						}
						
						if (mixing_val_1 & DIFF_THRUST)
							menu_display_h[13] = 'V';
						else
							menu_display_h[13] = ' ';
						if (mixing_val_1 & FLAPERON)
							menu_display_l[10] = 'V';
						else
							menu_display_l[10] = ' ';
						
					}
					else if (menu_input == 4)
					{
						// Display init
						menu_display_h = " Flaperon       ";
						menu_display_l = ">User           ";
						
						if (enter)
						{
							if (mixing_val_3 & USER_MIX)
								mixing_val_3 &= ~USER_MIX;
							else
								mixing_val_3 |= USER_MIX;
						}
						
						if (mixing_val_1 & FLAPERON)
							menu_display_h[10] = 'V';
						else
							menu_display_h[10] = ' ';
						if (mixing_val_3 & USER_MIX)
							menu_display_l[10] = 'V';
						else
							menu_display_l[10] = ' ';
						
					}
					else if (menu_input == 5)
					{
						// Display init
						menu_display_h = " User           ";
						menu_display_l = ">Exit           ";
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
						
						if (mixing_val_1 & USER_MIX)
							menu_display_h[10] = 'V';
						else
							menu_display_h[10] = ' ';
						
					}
					
					
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
					
				} // while (1)
			} // if (enter)
		} // else if (menu_input == 1)
	
		
		////////// Channel Reverse ///////////////////////////////////////////////
		else if (menu_input == 2)
		{
			// Display
			menu_display_h = " Mixing         ";
			menu_display_l = ">Reverse        ";
			
			if (enter)
			{
				// Cursor init
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				
				while (1)
				{
					// Cursor limit
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 4)
						menu_input = 4;
					
					
					// Display init & stat ////////////
					if (menu_input < 4)
					{
						// Display init
						menu_display_h = " Ail     Ele    ";
						menu_display_l = " Rud     CH5    ";
						
						// Aileron
						if (mixing_val_1 & REVERSE_AIL)
							menu_display_h[5] = 'R';
						else
							menu_display_h[5] = 'N';
						// Elevator
						if (mixing_val_1 & REVERSE_ELE)
							menu_display_h[14] = 'R';
						else
							menu_display_h[14] = 'N';
						// Rudder
						if (mixing_val_1 & REVERSE_RUD)
							menu_display_l[5] = 'R';
						else
							menu_display_l[5] = 'N';
						// CH5
						if (mixing_val_1 & REVERSE_CH5)
							menu_display_l[14] = 'R';
						else
							menu_display_l[14] = 'N';
					}
					else
					{
						// Display init
						menu_display_h = " Rud     CH5    ";
						menu_display_l = " Exit           ";
						
						// Aileron
						if (mixing_val_1 & REVERSE_RUD)
							menu_display_h[5] = 'R';
						else
							menu_display_h[5] = 'N';
						// Elevator
						if (mixing_val_1 & REVERSE_CH5)
							menu_display_h[14] = 'R';
						else
							menu_display_h[14] = 'N';
							
					}
					
					
					// Cursor move & val set //////////
					// Aileron
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_h[0] = '>';
						
						if (enter)
						{
							if (mixing_val_1 & REVERSE_AIL)
								mixing_val_1 &= ~REVERSE_AIL;
							else
								mixing_val_1 |= REVERSE_AIL;
						}
					}
					else
						menu_display_h[0] = ' ';
					
					
					// Elevator
					if (menu_input == 1)
					{
						// Cursor move
						menu_display_h[8] = '>';
						
						if (enter)
						{
							if (mixing_val_1 & REVERSE_ELE)
								mixing_val_1 &= ~REVERSE_ELE;
							else
								mixing_val_1 |= REVERSE_ELE;
						}
					}
					else
						menu_display_h[8] = ' ';
					
					
					// Rudder
					if (menu_input == 2)
					{
						// Cursor move
						menu_display_l[0] = '>';
						
						if (enter)
						{
							if (mixing_val_1 & REVERSE_RUD)
								mixing_val_1 &= ~REVERSE_RUD;
							else
								mixing_val_1 |= REVERSE_RUD;
						}
					}
					else
						menu_display_l[0] = ' ';
					
					
					// CH5
					if (menu_input == 3)
					{
						// Cursor move
						menu_display_l[8] = '>';
						
						if (enter)
						{
							if (mixing_val_1 & REVERSE_CH5)
								mixing_val_1 &= ~REVERSE_CH5;
							else
								mixing_val_1 |= REVERSE_CH5;
						}
					}
					else
						menu_display_l[8] = ' ';
					
					
					// Exit
					if (menu_input == 4)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
							
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					
					/*
					// Test
					if (menu_input == 5)
					{
						// Cursor move
						menu_display_l[0] = ' ';
						menu_display_l[8] = '>';
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					*/
						
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
					
					
				} // while
			} // if (enter)
		} // else if (menu_input == 2)
		
		
		////////// Dual Rate /////////////////////////////////////////////////////
		else if (menu_input == 3)
		{
			// Display
			menu_display_h = " Reverse        ";
			menu_display_l = ">Dual Rate      ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)		// Since max # of channel is 5
						menu_input = 0;
					else if (menu_input > 4)
						menu_input = 4;
					
					switch_read();
					set_switch();
					
					// Display init & stat ////////////
					if (menu_input < 4)
					{
						// Display init
						menu_display_h = " Ail     Ele    ";
						menu_display_l = " Rud     CH5    ";
						
						// Aileron
						if (mixing_val_2 & AIL_DR)
							menu_display_h[5] = 'D';
						else
							menu_display_h[5] = 'N';
						// Elevator
						if (mixing_val_2 & ELE_DR)
							menu_display_h[14] = 'D';
						else
							menu_display_h[14] = 'N';
						// Rudder
						if (mixing_val_2 & RUD_DR)
							menu_display_l[5] = 'D';
						else
							menu_display_l[5] = 'N';
						// CH5
						if (mixing_val_2 & CH5_DR)
							menu_display_l[14] = 'D';
						else
							menu_display_l[14] = 'N';
					}
					else
					{
						// Display init
						menu_display_h = " Rud     CH5   ";
						menu_display_l = " Exit           ";
						
						// Aileron
						if (mixing_val_2 & RUD_DR)
							menu_display_h[5] = 'D';
						else
							menu_display_h[5] = 'N';
						// Elevator
						if (mixing_val_2 & CH5_DR)
							menu_display_h[14] = 'D';
						else
							menu_display_h[14] = 'N';
						
					}
					menu_display_h[15] = ' ';
					
					
					// Cursor move & val set //////////
					// Aileron
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_h[0] = '>';
						
						if (enter)
						{
							if (mixing_val_2 & AIL_DR)
								mixing_val_2 &= ~AIL_DR;
							else
								mixing_val_2 |= AIL_DR;
						}
					}
					else
						menu_display_h[0] = ' ';
					
					
					// Elevator
					if (menu_input == 1)
					{
						// Cursor move
						menu_display_h[8] = '>';
						
						if (enter)
						{
							if (mixing_val_2 & ELE_DR)
								mixing_val_2 &= ~ELE_DR;
							else
								mixing_val_2 |= ELE_DR;
						}
					}
					else
						menu_display_h[8] = ' ';
					
					
					// Rudder
					if (menu_input == 2)
					{
						// Cursor move
						menu_display_l[0] = '>';
						
						if (enter)
						{
							if (mixing_val_2 & RUD_DR)
								mixing_val_2 &= ~RUD_DR;
							else
								mixing_val_2 |= RUD_DR;
						}
					}
					else
						menu_display_l[0] = ' ';
				
					
					// CH5
					if (menu_input == 3)
					{
						// Cursor move
						menu_display_l[8] = '>';
						
						if (enter)
						{
							if (mixing_val_2 & CH5_DR)
								mixing_val_2 &= ~CH5_DR;
							else
								mixing_val_2 |= CH5_DR;
						}
					}
					else
						menu_display_l[8] = ' ';
					
					// Exit
					if (menu_input == 4)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					
					
					// LCD drive
					LCD_home();
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					
					
					// Buzzer drive
					buzzer_switch();
					
					
				} // while (1)
			}// if (enter)
		} // else if (menu_input == 3)
		
		
		////////// Trim //////////////////////////////////////////////////////////
		else if (menu_input == 4)
		{
			// Display
			menu_display_h = " Dual Rate      ";
			menu_display_l = ">Trim           ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = cur_model;
				LCD_CLEAR();
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)		// Since max # of models is 8, limit model # input
						menu_input = 0;
					else if (menu_input > 5)
						menu_input = 5;
					

					// Display & stat /////////////////
					if (menu_input < 4)
					{
						// Display init
						menu_display_h = " Ail     Ele    ";
						menu_display_l = " Thr     Rud    ";
						
						// Aileron
						if (trim[AILERON] < 0)
						{
							menu_display_h[5] = '-';
							menu_display_h[6] = 48 - trim[AILERON];
						}
						else
						{
							menu_display_h[5] = ' ';
							menu_display_h[6] = trim[AILERON] + 48;
						}
						// Elevator
						if (trim[ELEVATOR] < 0)
						{	
							menu_display_h[13] = '-';
							menu_display_h[14] = 48 - trim[ELEVATOR];
						}
						else
						{	
							menu_display_h[13] = ' ';
							menu_display_h[14] = trim[ELEVATOR] + 48;
						}
						// Throttle
						if (trim[THROTTLE] < 0)
						{
							menu_display_l[5] = '-';
							menu_display_l[6] = 48 - trim[THROTTLE];
						}
						else
						{	
							menu_display_l[5] = ' ';
							menu_display_l[6] = trim[THROTTLE] + 48;
						}
						// Rudder
						if (trim[RUDDER] < 0)
						{	
							menu_display_l[13] = '-';
							menu_display_l[14] = 48 - trim[RUDDER];
						}
						else
						{	
							menu_display_l[13] = ' ';
							menu_display_l[14] = trim[RUDDER] + 48;
						}
						
					}
					else
					{
						// Display init
						menu_display_h = " Thr     Rud    ";
						menu_display_l = " CH5     Exit   ";
						
						// Throttle
						if (trim[THROTTLE] < 0)
						{
							menu_display_h[5] = '-';
							menu_display_l[6] = 48 - trim[THROTTLE];
						}
						else
						{
							menu_display_h[5] = ' ';
							menu_display_h[6] = trim[THROTTLE] + 48;
						}
						// Rudder
						if (trim[RUDDER] < 0)
						{
							menu_display_h[13] = '-';
							menu_display_h[14] = 48 - trim[RUDDER];
						}
						else
						{
							menu_display_h[13] = ' ';
							menu_display_h[14] = trim[RUDDER] + 48;
						}
						
						// CH5
						if (trim[CH5] < 0)
						{
							menu_display_l[5] = '-';
							menu_display_l[6] = 48 - trim[CH5];
						}
						else
						{
							menu_display_l[5] = ' ';
							menu_display_l[6] = trim[CH5] + 48;
						}
						
					}
					
					
					// Cursor move & val set //////////
					// Aileron
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_h[0] = '>';
						
						if (enter)
						{
							// menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								
								// if menu button is pressed
								if (menu_input)
								{
									trim[AILERON] -= menu_input;
									
									// Trim limit
									if (trim[AILERON] > TRIM_LIM)
										trim[AILERON] = TRIM_LIM;
									else if (trim[AILERON] < -TRIM_LIM)
										trim[AILERON] = -TRIM_LIM;
										
									menu_input = 0;
								}
								
								// Exit
								if (enter)
								{
									//menu_input = 0;
									break;
								}
								// Display
								if (trim[AILERON] < 0)
								{
									menu_display_h[5] = '-';
									menu_display_h[6] = 48 - trim[AILERON];
								}
								else
								{
									menu_display_h[5] = ' ';
									menu_display_h[6] = trim[AILERON] + 48;
								}
								
								// LCD drive
								LCD_string(menu_display_h, 16);
								LCD_home();
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
							
						} //if (enter)
					} //if (menu_input == 0)
					else
						menu_display_h[0] = ' ';
					
					// Elevator
					if (menu_input == 1)
					{
						// Cursor move
						menu_display_h[8] = '>';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								
								// if menu button is pressed
								if (menu_input)
								{
									trim[ELEVATOR] -= menu_input;
									
									// Trim limit
									if (trim[ELEVATOR] > TRIM_LIM)
										trim[ELEVATOR] = TRIM_LIM;
									else if (trim[ELEVATOR] < -TRIM_LIM)
										trim[ELEVATOR] = -TRIM_LIM;
									
									menu_input = 0;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 1;
									break;
								}
								// Display
								if (trim[ELEVATOR] < 0)
								{
									menu_display_h[13] = '-';
									menu_display_h[14] = 48 - trim[ELEVATOR];
								}
								else
								{
									menu_display_h[13] = ' ';
									menu_display_h[14] = trim[ELEVATOR] + 48;
								}
								
								// LCD drive
								LCD_string(menu_display_h, 16);
								LCD_home();
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
							
						} //if(enter)
					}
					else
						menu_display_h[8] = ' ';
					
					// Throttle
					if (menu_input == 2)
					{
						// Cursor move
						menu_display_l[0] = '>';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								
								// if menu button is pressed
								if (menu_input)
								{
									trim[THROTTLE] -= menu_input;
									
									// Trim limit
									if (trim[THROTTLE] > TRIM_LIM)
										trim[THROTTLE] = TRIM_LIM;
									else if (trim[THROTTLE] < -TRIM_LIM)
										trim[THROTTLE] = -TRIM_LIM;
									
									menu_input = 0;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 2;
									break;
								}
								// Display
								if (trim[THROTTLE] < 0)
								{
									menu_display_l[5] = '-';
									menu_display_l[6] = 48 - trim[THROTTLE];
								}
								else
								{
									menu_display_l[5] = ' ';
									menu_display_l[6] = trim[THROTTLE] + 48;
								}
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
							
						} // if(enter)
					} // if(menu_input == 2)
					else
						menu_display_l[0] = ' ';
					
					// Rudder
					if (menu_input == 3)
					{
						// Cursor move
						menu_display_l[8] = '>';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								
								// if menu button is pressed
								if (menu_input)
								{
									trim[RUDDER] -= menu_input;
									
									// Trim limit
									if (trim[RUDDER] > TRIM_LIM)
										trim[RUDDER] = TRIM_LIM;
									else if (trim[RUDDER] < -TRIM_LIM)
										trim[RUDDER] = -TRIM_LIM;
									
									menu_input = 0;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 3;
									break;
								}
								// Display
								if (trim[RUDDER] < 0)
								{
									menu_display_l[13] = '-';
									menu_display_l[14] = 48 - trim[RUDDER];
								}
								else
								{
									menu_display_l[13] = ' ';
									menu_display_l[14] = trim[RUDDER] + 48;
								}
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
							
						} // if(enter)
					} // if (menu_input == 3)
					else
						menu_display_l[8] = ' ';
					
					// CH5
					if (menu_input == 4)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								
								// if menu button is pressed
								if (menu_input)
								{
									trim[CH5] -= menu_input;
									
									// Trim limit
									if (trim[CH5] > TRIM_LIM)
										trim[CH5] = TRIM_LIM;
									else if (trim[CH5] < -TRIM_LIM)
										trim[CH5] = -TRIM_LIM;
									
									menu_input = 0;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 4;
									break;
								}
								// Display
								if (trim[CH5] < 0)
								{
									menu_display_l[5] = '-';
									menu_display_l[6] = 48 - trim[CH5];
								}
								else
								{
									menu_display_l[5] = ' ';
									menu_display_l[6] = trim[CH5] + 48;
								}
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
							
						} //if (enter)
					} //if (menu_input == 4)
					
					// Exit
					if (menu_input == 5)
					{
						// Cursor move
						menu_display_l[0] = ' ';
						menu_display_l[8] = '>';
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					
					
					// LCD drive
					LCD_home();
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					
					
					// Buzzer drive
					buzzer_switch();
					
				} // while
			} // if (enter)
		} // else if (menu_input == 4)
		
		
		////////// Servo delay ///////////////////////////////////////////////////
		else if (menu_input == 5)
		{
			// Display
			menu_display_h = " Trim           ";
			menu_display_l = ">Servo Delay    ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				// Display init
				menu_display_h = " CH5     CH6    ";
				menu_display_l = " Exit           ";
				
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 2)
						menu_input = 2;
					
					
					// CH5
					if (mixing_val_3 & SERVO_DEL_1)
						menu_display_h[6] = 'D';
					else
						menu_display_h[6] = ' ';
					// test
					if (mixing_val_3 & SERVO_DEL_2)
						menu_display_h[14] = 'D';
					else
						menu_display_h[14] = ' ';
					
					
					// Cursor move & val set //////////
					// CH5
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_h[0] = '>';
						
						if (enter)
						{
							if (mixing_val_3 & SERVO_DEL_1)
								mixing_val_3 &= ~SERVO_DEL_1;
							else
								mixing_val_3 |= SERVO_DEL_1;
						}
					}
					else
						menu_display_h[0] = ' ';
					
					
					// Ch6
					if (menu_input == 1)
					{
						// Cursor move
						menu_display_h[8] = '>';
						
						if (enter)
						{
							if (mixing_val_3 & SERVO_DEL_2)
								mixing_val_3 &= ~SERVO_DEL_2;
							else
								mixing_val_3 |= SERVO_DEL_2;
						}
					}
					else
						menu_display_h[8] = ' ';
					
					
					// Exit
					if (menu_input == 2)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					else
						menu_display_l[0] = ' ';
					
					
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
					
				} // while (1)
			}// if (enter)
		} // else if (menu_input == 5)
		
		
		////////// On board airplane battery charger control /////////////////////
		else if (menu_input == 6)
		{
			// Display
			menu_display_h = " Servo Delay    ";
			menu_display_l = ">Bat Charger    ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				
				// Display init
				menu_display_h = "Battery Charger ";
				menu_display_l = " 30 mah     Exit";
				
				LCD_CLEAR();
				LCD_string(menu_display_h, 16);
				
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 2)
						menu_input = 2;
					
					// Display & stat /////////////////
					if (batCharger & ON)
					{
						menu_display_l[8]  = 'O';
						menu_display_l[9]  = 'n';
						menu_display_l[10] = ' ';
					}
					else
					{
						menu_display_l[8]  = 'O';
						menu_display_l[9]  = 'f';
						menu_display_l[10] = 'f';
					}
					
					switch (batCharger&BATCHARG_CAP_MASK)
					{
						case CHG_30MAH:
							menu_display_l[1] = '3';
							menu_display_l[2] = '0';
							menu_display_l[3] = ' ';
							break;
						
						case CHG_85MAH:
							menu_display_l[1] = '8';
							menu_display_l[2] = '5';
							menu_display_l[3] = ' ';
							break;
						
						case CHG_120MAH:
							menu_display_l[1] = '1';
							menu_display_l[2] = '2';
							menu_display_l[3] = '0';
							break;
						
						case CHG_150MAH:
							menu_display_l[1] = '1';
							menu_display_l[2] = '5';
							menu_display_l[3] = '0';
							break;
						
						case CHG_300MAH:
							menu_display_l[1] = '3';
							menu_display_l[2] = '0';
							menu_display_l[3] = '0';
							break;
						
					}
					
					
					// Setting
					if (menu_input == 0)
					{
						menu_display_l[0]  = '>';
						menu_display_l[7]  = ' ';
						menu_display_l[11] = ' ';
						
						if (enter)
						{
							while(1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 4)
									menu_input = 4;
								
								switch (menu_input)
								{
									case 0:
										batCharger &= ~BATCHARG_CAP_MASK;
										batCharger |= CHG_30MAH;
									
										menu_display_l[1] = '3';
										menu_display_l[2] = '0';
										menu_display_l[3] = ' ';
										break;
									
									case 1:
										batCharger &= ~BATCHARG_CAP_MASK;
										batCharger |= CHG_85MAH;
										
										menu_display_l[1] = '8';
										menu_display_l[2] = '5';
										menu_display_l[3] = ' ';
										break;
									
									case 2:
										batCharger &= ~BATCHARG_CAP_MASK;
										batCharger |= CHG_120MAH;
										
										menu_display_l[1] = '1';
										menu_display_l[2] = '2';
										menu_display_l[3] = '0';
										break;
									
									case 3:
										batCharger &= ~BATCHARG_CAP_MASK;
										batCharger |= CHG_150MAH;
										
										menu_display_l[1] = '1';
										menu_display_l[2] = '5';
										menu_display_l[3] = '0';
									break;
									
									case 4:
										batCharger &= ~BATCHARG_CAP_MASK;
										batCharger |= CHG_300MAH;
										
										menu_display_l[1] = '3';
										menu_display_l[2] = '0';
										menu_display_l[3] = '0';
										break;
									
								}
								
								if (enter)
								{
									menu_input = 0;
									break;
								} //if (enter)
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
								
							} //while(1)
							
						}  // if (enter)
						
					} //if (menu_input == 0)
					else if (menu_input == 1)
					{
						menu_display_l[0]  = ' ';
						menu_display_l[7]  = '>';
						menu_display_l[11] = ' ';
						
						if (enter)
						{
							if (batCharger & ON)
								batCharger &= ~ON;
							else
								batCharger |= ON;
						}
						
					}
					else if (menu_input == 2)
					{
						// Cursor move
						menu_display_l[7]  = ' ';
						menu_display_l[11] = '>';
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					
					
					// Charger comm
					// i2c
					
					
					// LCD drive
					LCD_home();
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					
					// Buzzer drive
					buzzer_switch();
					
				}// while(1)
			} // if(enter)
			
		} // else if (menu_input == 6)
		
		
		////////// Servo/Actuator tester /////////////////////////////////////////
		else if (menu_input == 7)
		{
			// Display
			menu_display_h = " Bat Charger    ";
			menu_display_l = ">Servo Test     ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				menu_display_h = "Servo test (Rud)";
				lcd2 =           "          >Exit ";
				
				while (1)
				{
					stick_read();
					LCD_guage(channel[RUDDER], 2);
					
					if (menu_read())
					{
						menu_input = menu_input_temp;
						break;
					}
					
					// Communication with BO board
					//i2c
					
					
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(lcd2, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
				}
			}
		} // else if (menu_input == 7)
		
		
		////////// Mute //////////////////////////////////////////////////////////
		else if (menu_input == 8)
		{
			// Display
			menu_display_h = " Servo Test     ";
			menu_display_l = ">Speaker Mute   ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				menu_display_h = " Speaker Volume ";
				menu_display_l = "          Exit  ";
				
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 1)
						menu_input = 1;
					
					// Value display
					if (buzzer_volume)
					{
						menu_display_l[1] = 'O';
						menu_display_l[2] = 'n';
						menu_display_l[3] = ' ';
					}
					else
					{
						menu_display_l[1] = 'O';
						menu_display_l[2] = 'f';
						menu_display_l[3] = 'f';
						
					}
					
					
					// Volume On-Off
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_l[0] = '>';
						
						if (enter)
						{
							if (buzzer_volume)
								buzzer_volume = 0;
							else
								buzzer_volume = 50;
						}
					}
					else
						menu_display_l[0] = ' ';
					
					// Exit
					if (menu_input == 1)
					{
						// Cursor move
						menu_display_l[9] = '>';
										
						if (enter)
						{
							menu_input = 8;
							break;
						}
					}
					else
						menu_display_l[9] = ' ';
					
					
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
					
				} // while (1)
			}// if (enter)
		} // else if (menu_input == 8)
		
		
		
		////////// Power saving mode time set ////////////////////////////////////
		else if (menu_input == 9)
		{
			// Display
			menu_display_h = " Speaker Mute   ";
			menu_display_l = ">Power off time ";
			
			if (enter)
			{
				menu_input = 0;
				LCD_CLEAR();
				menu_display_h = " Power off time ";
				menu_display_l = "           Exit ";
				
				switch (runtime_set)
				{
					case RUNTIME10s:
						menu_display_l[1] = '1';
						menu_display_l[2] = '0';
						menu_display_l[3] = ' ';
						menu_display_l[4] = 's';
						menu_display_l[5] = 'e';
						menu_display_l[6] = 'c';
						break;
					
					case RUNTIME10:
						menu_display_l[1] = '1';
						menu_display_l[2] = '0';
						menu_display_l[3] = ' ';
						menu_display_l[4] = 'm';
						menu_display_l[5] = 'i';
						menu_display_l[6] = 'n';
						break;
					
					case RUNTIME15:
						menu_display_l[1] = '1';
						menu_display_l[2] = '5';
						menu_display_l[3] = ' ';
						menu_display_l[4] = 'm';
						menu_display_l[5] = 'i';
						menu_display_l[6] = 'n';
						break;
					
					case RUNTIME30:
						menu_display_l[1] = '3';
						menu_display_l[2] = '0';
						menu_display_l[3] = ' ';
						menu_display_l[4] = 'm';
						menu_display_l[5] = 'i';
						menu_display_l[6] = 'n';
						break;
					
					default:
						menu_display_l[1] = 'E';
						menu_display_l[2] = 'R';
						menu_display_l[3] = 'R';
						break;
					
				} //switch (runtime_set)
				
				
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 1)
						menu_input = 1;
					
					if (menu_input == 0)
					{
						menu_display_l[0]  = '>';
						menu_display_l[10] = ' ';
						
						if (enter)
						{
							switch (runtime_set)
							{
								case RUNTIME30:
									menu_input = 0;
									break;
								case RUNTIME15:
									menu_input = 1;
									break;
								case RUNTIME10:
									menu_input = 2;
									break;
								case RUNTIME10s:
									menu_input = 3;
									break;
								
							}
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 3)
									menu_input = 3;
								
								switch (menu_input)
								{
									case 3:
										runtime_set = RUNTIME10s;
										menu_display_l[1] = '1';
										menu_display_l[2] = '0';
										menu_display_l[3] = ' ';
										menu_display_l[4] = 's';
										menu_display_l[5] = 'e';
										menu_display_l[6] = 'c';
										break;
									
									case 2:
										runtime_set = RUNTIME10;
										menu_display_l[1] = '1';
										menu_display_l[2] = '0';
										menu_display_l[3] = ' ';
										menu_display_l[4] = 'm';
										menu_display_l[5] = 'i';
										menu_display_l[6] = 'n';
										break;
									
									case 1:
										runtime_set = RUNTIME15;
										menu_display_l[1] = '1';
										menu_display_l[2] = '5';
										menu_display_l[3] = ' ';
										menu_display_l[4] = 'm';
										menu_display_l[5] = 'i';
										menu_display_l[6] = 'n';
										break;
										
									case 0:
										runtime_set = RUNTIME30;
										menu_display_l[1] = '3';
										menu_display_l[2] = '0';
										menu_display_l[3] = ' ';
										menu_display_l[4] = 'm';
										menu_display_l[5] = 'i';
										menu_display_l[6] = 'n';
										break;
										
									
								} //switch (menu_input)
								
								if (enter)
								{
									menu_input = 0;
									break;
								}
								
								
								// LCD drive
								LCD_string(menu_display_h, 16);
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								LCD_home();
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
						} //if (enter)
						
					}
					
					// Exit
					else if (menu_input == 1)
					{
						menu_display_l[0]  = ' ';
						menu_display_l[10] = '>';
						
						if (enter)
						{
							menu_input = 9;
							break;
						}
					} //if (menu_input == 1)
					
					
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
					
				} //while (1)
			} //if (enter)
		} //else if (menu_input == 9)
		
		////////// RF mode ///////////////////////////////////////////////////////
		else if (menu_input == 10)
		{
			uint8_t mixing_val_3_temp;
			mixing_val_3_temp = (mixing_val_3 & (~RF_MODE_MASK));
			// Display
			menu_display_h = " Power off time ";
			menu_display_l = ">Output mode    ";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = 0;
				LCD_CLEAR();
				
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 4)
						menu_input = 4;
					
					
					// Display init & stat ////////////
					if (menu_input < 4)
					{
						// Display init
						menu_display_h = " JF24    NRF    ";
						menu_display_l = " UART    BLE    ";
						
						// Aileron
						if ((mixing_val_3 & RF_MODE_MASK) == JF24)
							menu_display_h[6] = 'V';
						else
							menu_display_h[6] = ' ';
						// Elevator
						if ((mixing_val_3 & RF_MODE_MASK) == NRF)
							menu_display_h[13] = 'V';
						else
							menu_display_h[13] = ' ';
						// Rudder
						if ((mixing_val_3 & RF_MODE_MASK) == UART)
							menu_display_l[6] = 'V';
						else
							menu_display_l[6] = ' ';
						// CH5
						if ((mixing_val_3 & RF_MODE_MASK) == BLE)
							menu_display_l[13] = 'V';
						else
							menu_display_l[13] = ' ';
					}
					else
					{
						// Display init
						menu_display_h = " UART    BLE    ";
						menu_display_l = " Exit           ";
						
						// Aileron
						if ((mixing_val_3 & RF_MODE_MASK) == UART)
							menu_display_h[6] = 'V';
						else
							menu_display_h[6] = ' ';
						// Elevator
						if ((mixing_val_3 & RF_MODE_MASK) == BLE)
							menu_display_h[13] = 'V';
						else
							menu_display_h[13] = ' ';
						
					}
					
					
					// Cursor move & val set //////////
					// Jiffa - PPM
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_h[0] = '>';
						
						if (enter)
						{
							mixing_val_3 = JF24|mixing_val_3_temp;
						}
					}
					else
						menu_display_h[0] = ' ';
					
					
					// NRF
					if (menu_input == 1)
					{
						// Cursor move
						menu_display_h[8] = '>';
						
						if (enter)
						{
							mixing_val_3 = NRF|mixing_val_3_temp;
						}
					}
					else
						menu_display_h[8] = ' ';
					
					
					// UART
					if (menu_input == 2)
					{
						// Cursor move
						menu_display_l[0] = '>';
						
						if (enter)
						{
							mixing_val_3 = UART|mixing_val_3_temp;
						}
					}
					else
						menu_display_l[0] = ' ';
					
					
					// BLE
					if (menu_input == 3)
					{
						// Cursor move
						menu_display_l[8] = '>';
						
						if (enter)
						{
							mixing_val_3 = BLE|mixing_val_3_temp;
						}
					}
					else
						menu_display_l[8] = ' ';
					
					
					// Exit
					if (menu_input == 4)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
							
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					
						
					// LCD drive
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					LCD_home();
					
					// Buzzer drive
					buzzer_switch();
					
				} // while (1)
			}// if (enter)
		} // else if (menu_input == 9)
		
		
		////////// Switch allocation /////////////////////////////////////////////
		else if (menu_input == 11)
		{
			// Display
			menu_display_h = " Output mode    ";
			menu_display_l = ">Switch allocate";
			
			if (enter)
			{
				menu_input_temp = menu_input;
				menu_input = cur_model;
				LCD_CLEAR();
				while (1)
				{
					enter = menu_read();
					if (menu_input < 0)
						menu_input = 0;
					else if (menu_input > 8)
						menu_input = 8;			// 7 switches
					
					
					// Display & stat /////////////////
					if (menu_input < 4)
					{
						// Display init
						menu_display_h = " SW1-    SW2-   ";
						menu_display_l = " SW3-    SW4-   ";
						
						// SW1 initial value
						switch (sw[0])
						{
							case 0:
								menu_display_h[5] = 'N';
								menu_display_h[6] = 'C';
								menu_display_h[7] = ' ';
								break;
							
							case CH5_SW:
								menu_display_h[5] = 'C';
								menu_display_h[6] = 'H';
								menu_display_h[7] = '5';
								break;
							
							case AIL_DR:
								menu_display_h[5] = 'D';
								menu_display_h[6] = 'R';
								menu_display_h[7] = 'A';
								break;
							
							case ELE_DR:
								menu_display_h[5] = 'D';
								menu_display_h[6] = 'R';
								menu_display_h[7] = 'E';
								break;
							
							case THR_DR:
								menu_display_h[5] = 'D';
								menu_display_h[6] = 'R';
								menu_display_h[7] = 'T';
								break;
							
							case RUD_DR:
								menu_display_h[5] = 'D';
								menu_display_h[6] = 'R';
								menu_display_h[7] = 'R';
								break;
							
							case CH5_DR:
								menu_display_h[5] = 'D';
								menu_display_h[6] = 'R';
								menu_display_h[7] = '5';
								break;
						}
						// SW2 initial value
						switch (sw[1])
						{
							case 0:
								menu_display_h[13] = 'N';
								menu_display_h[14] = 'C';
								menu_display_h[15] = ' ';
								break;
							
							case CH5_SW:
								menu_display_h[13] = 'C';
								menu_display_h[14] = 'H';
								menu_display_h[15] = '5';
								break;
							
							case AIL_DR:
								menu_display_h[13] = 'D';
								menu_display_h[14] = 'R';
								menu_display_h[15] = 'A';
								break;
							
							case ELE_DR:
								menu_display_h[13] = 'D';
								menu_display_h[14] = 'R';
								menu_display_h[15] = 'E';
								break;
							
							case THR_DR:
								menu_display_h[13] = 'D';
								menu_display_h[14] = 'R';
								menu_display_h[15] = 'T';
								break;
							
							case RUD_DR:
								menu_display_h[13] = 'D';
								menu_display_h[14] = 'R';
								menu_display_h[15] = 'R';
								break;
							
							case CH5_DR:
								menu_display_h[13] = 'D';
								menu_display_h[14] = 'R';
								menu_display_h[15] = '5';
								break;
						}
						// SW3 initial value
						switch (sw[2])
						{
							case 0:
								menu_display_l[5] = 'N';
								menu_display_l[6] = 'C';
								menu_display_l[7] = ' ';
								break;
							
							case CH5_SW:
								menu_display_l[5] = 'C';
								menu_display_l[6] = 'H';
								menu_display_l[7] = '5';
								break;
								
							case AIL_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'A';
								break;
							
							case ELE_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'E';
								break;
							
							case THR_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'T';
								break;
							
							case RUD_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'R';
								break;
							
							case CH5_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = '5';
								break;
						}
						// SW4 initial value
						switch (sw[3])
						{
							case 0:
								menu_display_l[13] = 'N';
								menu_display_l[14] = 'C';
								menu_display_l[15] = ' ';
								break;
							
							case CH5_SW:
								menu_display_l[13] = 'C';
								menu_display_l[14] = 'H';
								menu_display_l[15] = '5';
								break;
							
							case AIL_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'A';
								break;
							
							case ELE_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'E';
								break;
							
							case THR_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'T';
								break;
							
							case RUD_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'R';
								break;
							
							case CH5_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = '5';
								break;
						}
						
						
					}
					else if (menu_input < 6)
					{
						// Display init
						menu_display_h = " SW3-    SW4-   ";
						menu_display_l = " SW5-    SW6-   ";
						
						// SW5 initial value
						switch (sw[4])
						{
							case 0:
								menu_display_l[5] = 'N';
								menu_display_l[6] = 'C';
								menu_display_l[7] = ' ';
								break;
							
							case CH5_SW:
								menu_display_l[5] = 'C';
								menu_display_l[6] = 'H';
								menu_display_l[7] = '5';
								break;
							
							case AIL_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'A';
								break;
							
							case ELE_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'E';
								break;
							
							case THR_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'T';
								break;
							
							case RUD_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'R';
								break;
							
							case CH5_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = '5';
								break;
						}
						// SW6 initial value
						switch (sw[5])
						{
							case 0:
								menu_display_l[13] = 'N';
								menu_display_l[14] = 'C';
								menu_display_l[15] = ' ';
								break;
							
							case CH5_SW:
								menu_display_l[13] = 'C';
								menu_display_l[14] = 'H';
								menu_display_l[15] = '5';
								break;
							
							case AIL_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'A';
								break;
							
							case ELE_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'E';
								break;
							
							case THR_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'T';
								break;
							
							case RUD_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'R';
								break;
							
							case CH5_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = '5';
								break;
						}
						
					}
					else if (menu_input < 8)
					{
						// Display init
						menu_display_h = " SW5-    SW6-   ";
						menu_display_l = " SW7-    SW8-   ";
						
						// SW7 initial value
						switch (sw[6])
						{
							case 0:
								menu_display_l[5] = 'N';
								menu_display_l[6] = 'C';
								menu_display_l[7] = ' ';
								break;
							
							case CH5_SW:
								menu_display_l[5] = 'C';
								menu_display_l[6] = 'H';
								menu_display_l[7] = '5';
								break;
							
							case AIL_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'A';
								break;
							
							case ELE_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'E';
								break;
							
							case THR_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'T';
								break;
							
							case RUD_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = 'R';
								break;
							
							case CH5_DR:
								menu_display_l[5] = 'D';
								menu_display_l[6] = 'R';
								menu_display_l[7] = '5';
								break;
						}
						// SW8 initial value
						switch (sw[7])
						{
							case 0:
								menu_display_l[13] = 'N';
								menu_display_l[14] = 'C';
								menu_display_l[15] = ' ';
								break;
							
							case CH5_SW:
								menu_display_l[13] = 'C';
								menu_display_l[14] = 'H';
								menu_display_l[15] = '5';
								break;
							
							case AIL_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'A';
								break;
							
							case ELE_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'E';
								break;
							
							case THR_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'T';
								break;
							
							case RUD_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = 'R';
								break;
							
							case CH5_DR:
								menu_display_l[13] = 'D';
								menu_display_l[14] = 'R';
								menu_display_l[15] = '5';
								break;
						}
						
					}
					else
					{
						// Display init
						menu_display_h = " SW7-    SW8-   ";
						menu_display_l = " Exit           ";
					}
					

					// Display & stat /////////////
					if (menu_input == 0)
					{
						// Cursor move
						menu_display_h[0] = '>';
						menu_display_h[8] = ' ';
						
						if (enter)
						{
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_h[5] = 'N';
									menu_display_h[6] = 'C';
									menu_display_h[7] = ' ';
									sw[0] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_h[5] = 'C';
									menu_display_h[6] = 'H';
									menu_display_h[7] = '5';
									sw[0] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_h[5] = 'D';
									menu_display_h[6] = 'R';
									menu_display_h[7] = 'A';
									sw[0] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_h[5] = 'D';
									menu_display_h[6] = 'R';
									menu_display_h[7] = 'E';
									sw[0] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_h[5] = 'D';
									menu_display_h[6] = 'R';
									menu_display_h[7] = 'R';
									sw[0] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_h[5] = 'D';
									menu_display_h[6] = 'R';
									menu_display_h[7] = '5';
									sw[0] = CH5_DR;
								}
								
								
								// Exit
								if (enter)
								{
									menu_input = 0;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_string(menu_display_h, 16);
								LCD_NWL();
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
							
						} // if(enter)
					} // if(menu_input == 0)
					
					else if (menu_input == 1)
					{
						// Cursor move
						menu_display_h[8] = '>';
						menu_display_h[0] = ' ';
						menu_display_l[0] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								// if menu button is pressed
								if (menu_input == 0)
								{
									menu_display_h[13] = 'N';
									menu_display_h[14] = 'C';
									menu_display_h[15] = ' ';
									sw[1] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_h[13] = 'C';
									menu_display_h[14] = 'H';
									menu_display_h[15] = '5';
									sw[1] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_h[13] = 'D';
									menu_display_h[14] = 'R';
									menu_display_h[15] = 'A';
									sw[1] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_h[13] = 'D';
									menu_display_h[14] = 'R';
									menu_display_h[15] = 'E';
									sw[1] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_h[13] = 'D';
									menu_display_h[14] = 'R';
									menu_display_h[15] = 'R';
									sw[1] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_h[13] = 'D';
									menu_display_h[14] = 'R';
									menu_display_h[15] = '5';
									sw[1] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 1;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_string(menu_display_h, 16);
								LCD_NWL();
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 1)
					
					else if (menu_input == 2)
					{
						// Cursor move
						menu_display_l[0] = '>';
						menu_display_h[8] = ' ';
						menu_display_l[8] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_l[5] = 'N';
									menu_display_l[6] = 'C';
									menu_display_l[7] = ' ';
									sw[2] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_l[5] = 'C';
									menu_display_l[6] = 'H';
									menu_display_l[7] = '5';
									sw[2] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'A';
									sw[2] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'E';
									sw[2] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'R';
									sw[2] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = '5';
									sw[2] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 2;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 2)
					
					else if (menu_input == 3)
					{
						// Cursor move
						menu_display_l[8] = '>';
						menu_display_l[0] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_l[13] = 'N';
									menu_display_l[14] = 'C';
									menu_display_l[15] = ' ';
									sw[3] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_l[13] = 'C';
									menu_display_l[14] = 'H';
									menu_display_l[15] = '5';
									sw[3] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'A';
									sw[3] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'E';
									sw[3] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'R';
									sw[3] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = '5';
									sw[3] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 3;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 3)
					
					else if (menu_input == 4)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_l[5] = 'N';
									menu_display_l[6] = 'C';
									menu_display_l[7] = ' ';
									sw[4] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_l[5] = 'C';
									menu_display_l[6] = 'H';
									menu_display_l[7] = '5';
									sw[4] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'A';
									sw[4] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'E';
									sw[4] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'R';
									sw[4] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = '5';
									sw[4] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 4;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 4)
					
					else if (menu_input == 5)
					{
						// Cursor move
						menu_display_l[8] = '>';
						menu_display_l[0] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_l[13] = 'N';
									menu_display_l[14] = 'C';
									menu_display_l[15] = ' ';
									sw[5] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_l[13] = 'C';
									menu_display_l[14] = 'H';
									menu_display_l[15] = '5';
									sw[5] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'A';
									sw[5] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'E';
									sw[5] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'R';
									sw[5] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = '5';
									sw[5] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 5;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 5)
					
					else if (menu_input == 6)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_l[5] = 'N';
									menu_display_l[6] = 'C';
									menu_display_l[7] = ' ';
									sw[6] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_l[5] = 'C';
									menu_display_l[6] = 'H';
									menu_display_l[7] = '5';
									sw[5] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'A';
									sw[5] = AIL_DR ;
								}
								else if (menu_input == 3)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'E';
									sw[5] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = 'R';
									sw[5] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_l[5] = 'D';
									menu_display_l[6] = 'R';
									menu_display_l[7] = '5';
									sw[5] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 6;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 6)
					
					else if (menu_input == 7)
					{
						// Cursor move
						menu_display_l[8] = '>';
						menu_display_l[0] = ' ';
						
						if (enter)
						{
							menu_input = 0;
							
							while (1)
							{
								enter = menu_read();
								if (menu_input < 0)
									menu_input = 0;
								else if (menu_input > 5)
									menu_input = 5;
								
								
								if (menu_input == 0)
								{
									menu_display_l[13] = 'N';
									menu_display_l[14] = 'C';
									menu_display_l[15] = ' ';
									sw[7] = 0;
								}
								else if (menu_input == 1)
								{
									menu_display_l[13] = 'C';
									menu_display_l[14] = 'H';
									menu_display_l[15] = '5';
									sw[7] = CH5_SW;
								}
								else if (menu_input == 2)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'A';
									sw[7] = AIL_DR;
								}
								else if (menu_input == 3)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'E';
									sw[7] = ELE_DR;
								}
								else if (menu_input == 4)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = 'R';
									sw[7] = RUD_DR;
								}
								else if (menu_input == 5)
								{
									menu_display_l[13] = 'D';
									menu_display_l[14] = 'R';
									menu_display_l[15] = '5';
									sw[7] = CH5_DR;
								}
								
								// Exit
								if (enter)
								{
									menu_input = 7;
									break;
								}
								
								
								// LCD drive
								LCD_home();
								LCD_NWL();
								LCD_string(menu_display_l, 16);
								
								// Buzzer drive
								buzzer_switch();
								
								
							} //while (1)
						} // if(enter)
						
					} // else if (menu_input == 7)
					
					// Exit
					if (menu_input == 8)
					{
						// Cursor move
						menu_display_h[8] = ' ';
						menu_display_l[0] = '>';
						menu_display_l[8] = ' ';
						
						if (enter)
						{
							menu_input = menu_input_temp;
							break;
						}
					}
					
					
					// LCD drive
					LCD_home();
					LCD_string(menu_display_h, 16);
					LCD_NWL();
					LCD_string(menu_display_l, 16);
					
					// Buzzer drive
					buzzer_switch();
					
					
				} // while (1)
			} //if (enter)
		} //else if (menu_input == MENU_NUM)
		
		////////// Exit //////////////////////////////////////////////////////////
		else if (menu_input == MENU_NUM)
		{
			// Display
			menu_display_h = " Switch allocate";
			menu_display_l = ">Exit           ";
			
			// Key input
			if (enter)
			{
				uint8_t addr_temp;
				
				// Save settings
				addr_temp = cur_model * 15 + 1;
				
				eeprom_update_byte((uint8_t*)addr_temp,       trim[AILERON]);
				eeprom_update_byte((uint8_t*)(addr_temp + 1), trim[ELEVATOR]);
				eeprom_update_byte((uint8_t*)(addr_temp + 2), trim[RUDDER]);
				eeprom_update_byte((uint8_t*)(addr_temp + 3), trim[CH5]);
				
				eeprom_update_byte((uint8_t*)(addr_temp + 4), mixing_val_1);
				eeprom_update_byte((uint8_t*)(addr_temp + 5), mixing_val_2);
				eeprom_update_byte((uint8_t*)(addr_temp + 6), mixing_val_3);
				
				eeprom_update_byte((uint8_t*)(addr_temp + 7),  sw[0]);
				eeprom_update_byte((uint8_t*)(addr_temp + 8),  sw[1]);
				eeprom_update_byte((uint8_t*)(addr_temp + 9),  sw[2]);
				eeprom_update_byte((uint8_t*)(addr_temp + 10), sw[3]);
				eeprom_update_byte((uint8_t*)(addr_temp + 11), sw[4]);
				eeprom_update_byte((uint8_t*)(addr_temp + 12), sw[5]);
				eeprom_update_byte((uint8_t*)(addr_temp + 13), sw[6]);
				eeprom_update_byte((uint8_t*)(addr_temp + 14), sw[7]);
				
				eeprom_update_byte((uint8_t*)121, cur_model);
				eeprom_update_byte((uint8_t*)122, buzzer_volume);
				eeprom_update_word((uint16_t*)123, runtime_set);
				
				menu_input = 0;
				return 0;
				
			} // if (enter)
		} // else if (menu_input == MENU_NUM)
		
		
		////////// Power & Interface /////////////////////////////////////////////
		// Power off
		if (!(PINC & PWRDET) && (power_sw_toggle == OFF))
		{
			power_sw_toggle = ON;
			timer_8ms_power = 0;
		}
		else if ((timer_8ms_power > 125) && (power_sw_toggle == ON))
		{
			// Power off only if button is pressed for 1 s
			if (PINC & PWRDET)
				power_sw_toggle = OFF;
			else
				power_off();
			
		}
		
		// Speaker
		buzzer_switch();
		
		// LCD
		LCD_home();
		LCD_string(menu_display_h, 16);
		LCD_NWL();
		LCD_string(menu_display_l, 16);
		
		
	} // while (1)
} // void menu()


/****************************************************/
/*			Input									*/
/*													*/
/*													*/
/****************************************************/
void stick_read()
{
	// Aileron
	ADMUX = (1 << REFS0) | (1 << ADLAR) | 1;			// external vref, choose adc pin. read only 8 bit
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (6 << ADPS0);	// start adc read, adc prescalar 64
	while (ADCSRA & (1 << ADSC));
	channel[AILERON] = ADCH;
	
	// Elevator
	ADMUX = (1 << REFS0) | (1 << ADLAR) | 7;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (6 << ADPS0);
	while (ADCSRA & (1 << ADSC));
	channel[ELEVATOR] = ADCH;
	
	// Throttle
	ADMUX = (1 << REFS0) | (1 << ADLAR) | 3;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (6 << ADPS0);
	while (ADCSRA & (1 << ADSC));
	channel[THROTTLE] = ADCH;
	
	// Rudder
	ADMUX = (1 << REFS0) | (1 << ADLAR) | 0;
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (6 << ADPS0);
	while (ADCSRA & (1 << ADSC));
	channel[RUDDER] = ADCH;
	
	
}
void trim_read() 
{
	PORTD |= PO3;
	PORTD &= ~PO1;
	PORTE |= PO2|PO4;
	_delay_us(60);
	
	////////// CH1 (Rudder) //////////////////////////////////////////////////
	// CH1 (Rudder) trim -
	if (!(PINB & PI1))
	{
		// buzzer frequency
		if (trim[RUDDER] != 1)
			buzzer_freq = 1000 + ((trim[RUDDER]-1)<<4);
		else
			buzzer_freq = 3000;	// if trim is 0, high pitch sound
		
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[0] = ON;
	}
	else if ((PINB & PI1) && (button_tog[0] == ON))
	{
		// buzzer reset
		buzzer_setting[SETTING] &= ~STAT;	// reset STAT for single input
		
		trim[RUDDER]--;
		button_tog[0] = OFF;
	}
	
	// CH1 (Rudder) trim +
	if (!(PINB & PI2))
	{
		// buzzer frequency
		if (trim[RUDDER] != -1)
			buzzer_freq = 1000 + ((trim[RUDDER]+1)<<4);
		else
			buzzer_freq = 3000;	// if trim is 0, high pitch sound
		
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[1] = ON;
	}
	else if ((PINB & PI2) && (button_tog[1] == ON))
	{
		// buzzer reset
		buzzer_setting[SETTING] &= ~STAT;	// reset STAT for single input
		
		trim[RUDDER]++;
		button_tog[1] = OFF;
	}
	
	
	////////// CH2 (Elevator) ////////////////////////////////////////////////
	// CH2 (Elevator) trim-
	if (!(PINB & PI3))
	{
		// buzzer frequency
		if (trim[ELEVATOR] != 1)
			buzzer_freq = 1000 + ((trim[ELEVATOR]-1)<<4);
		else
			buzzer_freq = 3000;	// if trim is 0, high pitch sound
		
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[2] = ON;
	}
	else if ((PINB & PI3) && (button_tog[2] == ON))
	{
		// buzzer reset
		buzzer_setting[SETTING] &= ~STAT;	// reset STAT for single input
		
		trim[ELEVATOR]--;
		button_tog[2] = OFF;
	}
	
	// CH2 (Elevator) trim+
	if (!(PINB & PI4))
	{
		// buzzer frequency
		if (trim[ELEVATOR] != -1)
			buzzer_freq = 1000 + ((trim[ELEVATOR]+1)<<4);
		else
			buzzer_freq = 3000;	// if trim is 0, high pitch sound
		
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[3] = ON;
	}
	else if ((PINB & PI4) && (button_tog[3] == ON))
	{
		// buzzer reset
		buzzer_setting[SETTING] &= ~STAT;	// reset STAT for single input
		
		trim[ELEVATOR]++;
		button_tog[3] = OFF;
	}
	
	
	PORTD |= PO1|PO3;
	PORTE &= ~PO2;
	PORTE |= PO4;
	_delay_us(60);
	
	////////// CH3 (Aileron) /////////////////////////////////////////////////
	// CH3 (Aileron) trim -
	if (!(PINB & PI1))
	{
		// buzzer frequency
		if (trim[AILERON] != 1)
			buzzer_freq = 1000 + ((trim[AILERON]-1)<<4);
		else
			buzzer_freq = 3000;	// if trim is 0, high pitch sound
		
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[4] = ON;
	}
	else if ((PINB & PI1) && (button_tog[4] == ON))
	{
		// buzzer reset
		buzzer_setting[SETTING] &= ~STAT;	// reset STAT for single input
		
		trim[AILERON]--;
		button_tog[4] = OFF;
	}
	
	// CH2 (Aileron) trim +
	if (!(PINB & PI2))
	{
		// buzzer frequency
		if (trim[AILERON] != -1)
			buzzer_freq = 1000 + ((trim[AILERON]+1)<<4);
		else
			buzzer_freq = 3000;	// if trim is 0, high pitch sound
		
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[5] = ON;
	}
	else if ((PINB & PI2) && (button_tog[5] == ON))
	{
		// buzzer reset
		buzzer_setting[SETTING] &= ~STAT;	// reset STAT for single input
		
		trim[AILERON]++;
		button_tog[5] = OFF;
	}
	
	PORTD |= PO1|PO3;
	PORTE |= PO2|PO4;
}
void switch_read()
{
	PORTD |= PO1;
	PORTD &= ~PO3;
	PORTE |= PO2|PO4;
	_delay_us(60);
	
	// SW1
	if (PINB & PI4)	switch_val &= ~SW1;
	else			switch_val |= SW1;
	// SW2
	if (PINB & PI3)	switch_val &= ~SW2;
	else			switch_val |= SW2;
	// SW3
	if (PINB & PI1)	switch_val &= ~SW3;
	else			switch_val |= SW3;
	// SW4
	if (PINB & PI2)	switch_val &= ~SW4;
	else			switch_val |= SW4;
	
	PORTD |= PO1|PO2|PO3;
	PORTE &= ~PO4;
	_delay_us(60);
	
	// SW5
	if (PINB & PI4)	switch_val &= ~SW5;
	else			switch_val |= SW5;
	// SW6
	if (PINB & PI3)	switch_val &= ~SW6;
	else			switch_val |= SW6;
	// SW7
	if (PINB & PI2)	switch_val &= ~SW7;
	else			switch_val |= SW7;
	// SW8
	if (PINB & PI1)	switch_val &= ~SW8;
	else			switch_val |= SW8;
	
	PORTD |= PO1|PO2|PO3;
	
}
uint8_t menu_read()
{
	PORTD |= PO1|PO3;
	PORTE &= ~PO2;
	PORTE |= PO4;
	_delay_us(60);
	
	////////// Multi-input /////////////////////////////////////////
	if (!(PINB & PI3) && !(PINB & PI4))
	{
		// buzzer
		buzzer_freq = 1000;
		buzzer_setting[PERIOD]= 15;
		if (!(buzzer_setting[SETTING] & STAT))
			buzzer_setting[SETTING] |= BUZZER_SET|STAT;
		
		button_tog[6] = OFF;
		button_tog[7] = OFF;
		button_tog[8] = ON;
	}
	else if (((PINB & PI3) && (PINB & PI4)) && (button_tog[8] == ON))
	{
		buzzer_setting[SETTING] &= ~STAT;
		
		button_tog[8] = OFF;
		
		PORTD |= PO1|PO2|PO3;
		return 1;
	}
	
	////////// Single-input ////////////////////////////////////////
	else if (!button_tog[8])	
	{
		if (!(PINB & PI3))	// Menu 1
		{
			buzzer_freq = 1000;
			buzzer_setting[PERIOD]= 15;
			if (!(buzzer_setting[SETTING] & STAT))
				buzzer_setting[SETTING] |= BUZZER_SET|STAT;
			
			button_tog[6] = ON;
		}
		else if ((PINB & PI3) && (button_tog[6] == ON))
		{
			// buzzer reset
			buzzer_setting[SETTING] &= ~STAT;
			
			menu_input++;
			button_tog[6] = OFF;
		}
		
		if (!(PINB & PI4))   // Menu 2
		{
			buzzer_freq = 1000;
			buzzer_setting[PERIOD]= 15;
			if (!(buzzer_setting[SETTING] & STAT))
				buzzer_setting[SETTING] |= BUZZER_SET|STAT;
			
			button_tog[7] = ON;
		}
		else if ((PINB & PI4) && (button_tog[7] == ON))
		{
			// buzzer reset
			buzzer_setting[SETTING] &= ~STAT;
			
			menu_input--;
			button_tog[7] = OFF;
		}
	}
	
	PORTD |= PO1|PO3;
	PORTE |= PO2|PO4;
	return 0;
}
uint8_t transmitter_bat_chk()
{
	ADMUX = (1 << REFS0) | (1 << ADLAR) | 6;	// external vref, choose adc pin(6). read only 8 bit
	ADCSRA = (1 << ADEN) | (1 << ADSC) | (7 << ADPS0);	// start adc read, adc prescalar 128
	while (ADCSRA & (1 << ADSC));	// wait till conversion is finished
	TX_bat_mes = ADCH;
	
	return TX_bat_mes;
	
}


/****************************************************/
/*      Make channel value							*/
/*													*/
/*													*/
/****************************************************/
void set_switch()
{
	uint8_t ail_dr = 0, ele_dr = 0, thr_dr = 0, rud_dr = 0, ch5_dr = 0, ch5_sw = 0;
	
	
	for (uint8_t k=0; k<8; k++)
	{
		switch (sw[k])
		{
			case AIL_DR:
				ail_dr = (1<<k);
				break;
			
			case ELE_DR:
				ele_dr = (1<<k);
				break;
			
			case THR_DR:
				thr_dr = (1<<k);
				break;
			
			case RUD_DR:
				rud_dr = (1<<k);
				break;
			
			case CH5_DR:
				ch5_dr = (1<<k);
				break;
			
			case CH5_SW:
				ch5_sw = (1<<k);
				break;
			
		} // switch (sw[i])
	} //for (uint8_t k=0; k<8; k++)
	
	// Aileron DR
	if (ail_dr)
	{
		if (switch_val & ail_dr)
			mixing_val_2 |= AIL_DR;
		else
			mixing_val_2 &= ~AIL_DR;
	}
	// Elevator DR
	if (ele_dr)
	{
		if (switch_val & ele_dr)
			mixing_val_2 |= ELE_DR;
		else
			mixing_val_2 &= ~ELE_DR;
	}
	// Throttle DR
	if (thr_dr)
	{
		if (mixing_val_2 & thr_dr)
			mixing_val_2 |= THR_DR;
		else
			mixing_val_2 &= ~THR_DR;
	}
	// Rudder DR
	if (rud_dr)
	{
		if (switch_val & rud_dr)
			mixing_val_2 |= RUD_DR;
		else
			mixing_val_2 &= ~RUD_DR;
	}
	// CH5 DR
	if (ch5_dr)
	{
		if (switch_val & ch5_dr)
			mixing_val_2 |= CH5_DR;
		else
			mixing_val_2 &= ~CH5_DR;
	}
	// Aileron DR
	if (ch5_sw)
	{
		if (switch_val & ch5_sw)
			channel_mix_temp |= CH5_SW;
		else
			channel_mix_temp &= ~CH5_SW;
	}
	
	
}
void make_val()
{   
	uint8_t ail_temp, rud_temp;
	int16_t channel_temp;

	////////// Servo Delay /////////////////////////////////////////
	if (channel_mix_temp & CH5_SW)
	{
		// Delayed
		if (mixing_val_3 & SERVO_DEL_1)
			channel[CH5] = timer_8ms_Ch5Delay;
		// No delay
		else
			channel[CH5] = 0;
	}
	else
	{
		// Delayed
		if (mixing_val_3 & SERVO_DEL_1)
			channel[CH5] = timer_8ms_Ch5Delay;
		// No delay
		else
			channel[CH5] = 0xFF;
	}
	
	////////// Elevon Mixing ///////////////////////////////////////
	// 1. Substitute elevator value to aileron
	// 2. Add & subtract applied aileron input
	if (mixing_val_1 & ELEVON)
	{
		ail_temp = channel[AILERON];
		// elevator value
		channel[AILERON] = channel[ELEVATOR];
		
		// aileron value #1
		channel_temp = channel[ELEVATOR] + (ail_temp -127);
		// upper/lower limit
		if (channel_temp > 255)
			channel[ELEVATOR] = 255;
		else if (channel_temp < 0)
			channel[ELEVATOR] = 0;
		else
			channel[ELEVATOR] = channel_temp;
		
		// aileron value #2
		channel_temp = channel[AILERON] - (ail_temp -127);
		// upper & lower limit
		if (channel_temp > 255)
			channel[AILERON] = 255;
		else if (channel_temp < 0)
			channel[AILERON] = 0;
		else
			channel[AILERON] = channel_temp;
			
	} //if (mixing_val_1 & ELEVON)
	
	////////// Vtail Mixing ////////////////////////////////////////
	if (mixing_val_1 & VTAIL)
	{
		rud_temp = channel[RUDDER];
		// elevator value
		channel[RUDDER] = channel[ELEVATOR];
		
		// rudder value #1
		channel_temp = channel[ELEVATOR] + (rud_temp -127);
		// upper/lower limit
		if (channel_temp > 255)
			channel[ELEVATOR] = 255;
		else if (channel_temp < 0)
			channel[ELEVATOR] = 0;
		else
			channel[ELEVATOR] = channel_temp;
		
		// rudder value #2
		channel_temp = channel[RUDDER] - (rud_temp -127);
		// upper & lower limit
		if (channel_temp > 255)
			channel[RUDDER] = 255;
		else if (channel_temp < 0)
			channel[RUDDER] = 0;
		else
			channel[RUDDER] = channel_temp;
		
	} //if (mixing_val_1 & VTAIL)
	
	////////// Differential Thrust /////////////////////////////////
	if (mixing_val_1 & DIFF_THRUST)
	{
		rud_temp = channel[RUDDER];
		// Throttle value
		channel[RUDDER] = channel[THROTTLE];
		
		// rudder value #1
		channel_temp = channel[THROTTLE] + (rud_temp -127);
		// upper/lower limit
		if (channel_temp > 255)
			channel[THROTTLE] = 255;
		else if (channel_temp < 0)
			channel[THROTTLE] = 0;
		else
			channel[THROTTLE] = channel_temp;
		
		// rudder value #2
		channel_temp = channel[RUDDER] - (rud_temp -127);
		// upper & lower limit
		if (channel_temp > 255)
			channel[RUDDER] = 255;
		else if (channel_temp < 0)
			channel[RUDDER] = 0;
		else
			channel[RUDDER] = channel_temp;
		
	} // if (mixing_val_1 & DIFF_THRUST)
	
	////////// Trim Upper&Lower Limit //////////////////////////////
	// One click : 4
	for (uint8_t n = 0; n < 4; n++)
	{
		// Trim limit
		if (trim[n] > TRIM_LIM)
			trim[n] = TRIM_LIM;
		else if (trim[n] < -TRIM_LIM)
			trim[n] = -TRIM_LIM;

		// Trim adjust & Upper, lower limit
		channel_temp = channel[n] + (trim[n]<<2);
		
		if ((channel_temp >= 0) && (channel_temp <= 255))
			channel[n] = channel_temp;
		else if (channel_temp > 255)
			channel[n] = 255;
		else
			channel[n] = 0;
		
	} //for (uint8_t n = 0; n < 4; n++)
	
	////////// Dual Rate////////////////////////////////////////////
	// 50% original input
	if (mixing_val_2 & AIL_DR)
		channel[AILERON] = 127 + ((channel[AILERON] - 127)>>1);
	if (mixing_val_2 & ELE_DR)
		channel[ELEVATOR] = 127 + ((channel[ELEVATOR] - 127)>>1);
	if (mixing_val_2 & RUD_DR)
		channel[RUDDER] = 127 + ((channel[RUDDER] - 127)>>1);
	if (mixing_val_2 & CH5_DR)
		channel[CH5]    = 127 + ((channel[CH5] - 127)>>1);
	
	////////// Reverse Mixing //////////////////////////////////////
	if (mixing_val_1 & REVERSE_RUD)
		channel[RUDDER] = 255 - channel[RUDDER];
	if (mixing_val_1 & REVERSE_ELE)
		channel[ELEVATOR] = 255 - channel[ELEVATOR];
	if (mixing_val_1 & REVERSE_AIL)
		channel[AILERON] = 255 - channel[AILERON];
	if (mixing_val_1 & REVERSE_CH5)
		channel[CH5] = 255 - channel[CH5];
		
}


/****************************************************/
/*			Transmit data							*/
/*													*/
/*	make_ppm	: Make RC PPM signal - 				*/
/*	JF24_RF		: Transmit data using JF24 module	*/
/*				 Compatible with other RC TX modules*/
/*													*/
/****************************************************/
void make_ppm()
{
	for (uint8_t n = 0; n < CHANNEL; n++)
	{
		PORTD &= ~MODULE;	// channel division low
		_delay_us(400);
		
		PORTD |= MODULE;	// start signal
		_delay_us(570);		// min value
	
		if (channel[n] > 127)
		{
			_delay_us(530);
			channel[n] -= 127;
		}
		if (channel[n] > 63)
		{
			_delay_us(265);
			channel[n] -= 63;
		}
		if (channel[n] > 31)
		{
			_delay_us(132);
			channel[n] -= 31;
		}
		if (channel[n] > 15)
		{
			_delay_us(66);
			channel[n] -= 15;
		}
		if (channel[n] > 7)
		{
			_delay_us(33);
			channel[n] -= 7;
		}
		if (channel[n] > 3)
		{
			_delay_us(16);
			channel[n] -= 3;
		}
		if (channel[n] > 1)
		{
			_delay_us(8);
			channel[n] -= 1;
		}
		if (channel[n] == 1)
		{
			_delay_us(4);
		}
		
	}
	
	PORTD &= ~MODULE;	// channel division low	
	
}
void JF24_RF()
{
	// Making 24ms
	while (timer_24ms < 3);
	timer_24ms = 0;
	
	// Transmitting PPM signal
	make_ppm();
	
}


/************************************************************/
/*			Buzzer											*/
/*															*/
/*	buzzer_switch: Buzzer driver function with set&forget	*/
/*				  method (Only need to set parameters once)	*/
/*				  Can set buzzer pitch, on-off time	period	*/
/*				  and number of repetition.					*/
/*															*/
/*	There are three control parameters:						*/
/*	- buzzer_setting[PERIOD]								*/
/*	- buzzer_setting[SETTING]								*/
/*	- buzzer_freq											*/
/*															*/
/*	* buzzer_setting[PERIOD]								*/
/*	- input range : 0~255 - (input value * 8) ms period		*/
/*															*/
/*	* buzzer_setting[SETTING]								*/
/*	[NA][NA][STAT][INT][SET][REPEAT1][REPEAT1][REPEAT0]		*/
/*	- REPEAT2:0 : Repetition time							*/
/*	- SET       : Set to 1 to enable ringing. Become 0		*/
/*				  after finish ringing						*/
/*	- INIT		: Buzzer timer initialization flag(internal)*/
/*	- STAT		: Prevent trim button duplicate input		*/
/*															*/
/*	* buzzer_freq											*/
/*	- input range: 0~65,535 - (input value) Hz pitch		*/
/*															*/
/************************************************************/
void buzzer(uint16_t freq)
{
	// Frequency
	ICR1 = 500000 / freq;
	
	// Volume - On/Off
	if (buzzer_volume)
	{
		if (freq)
			OCR1A = (ICR1 >> 1);	// Duty: 50%
		else
			OCR1A = 0;
	}
	
	
}
void buzzer_switch()
{	
	// Start ringing routine if SET bit (buzzer_setting[SETTING]) is 1
	if (buzzer_setting[SETTING] & BUZZER_SET)
	{
		// Initialize buzzer timer
		if (buzzer_setting[SETTING] & INIT)
		{
			buzzer_setting[SETTING] &= ~INIT;
			timer_8ms_buzzer = 0;
		}
		// Buzzer On-Off
		else if (timer_8ms_buzzer < buzzer_setting[PERIOD])
		{
			buzzer(buzzer_freq);
		}
		else if (timer_8ms_buzzer < (buzzer_setting[PERIOD]<<1))
		{
			buzzer(0);
		}
		// Repetition
		else if (timer_8ms_buzzer > (buzzer_setting[PERIOD]<<1))
		{
			if (buzzer_setting[SETTING] & 0x07)
			{
				// repeat the cycle 'repeat' times
				// ex) if repeat==0: ring 1 time
				//        repeat==1: ring 2 times
				timer_8ms_buzzer = 0;
				buzzer_setting[SETTING]--;
			}
			else
			{
				// finish the cycle
				buzzer_setting[SETTING] |= INIT;
				buzzer_setting[SETTING] &= ~BUZZER_SET;
			}
		} //else if (timer_8ms_buzzer > (buzzer_setting[PERIOD]<<1))
	} //if (buzzer_setting[SETTING] & BUZZER_SET)
	else
	{
		buzzer(0);
		
	}
	
}


/****************************************************/
/*			LCD										*/
/*													*/
/*													*/
/****************************************************/
void LCD_guage(uint8_t val, uint8_t position)
{
	if (position < 2)
	{
		// 1st digit
		if (val < 17)
			lcd1[8*position + 1] = 0xFF;
		else if (val < 30)
			lcd1[8*position + 1] = 4;
		else if (val < 43)
			lcd1[8*position + 1] = 6;
		else if (val >= 43)
			lcd1[8*position + 1] = ' ';

		// 2nd digit
		if (val < 56)
			lcd1[8*position + 2] = 0xFF;
		else if (val < 69)
			lcd1[8*position + 2] = 5;
		else if (val < 82)
			lcd1[8*position + 2] = 7;
		else if (val >= 82)
			lcd1[8*position + 2] = ' ';

		// 3rd digit
		if (val < 82)
			lcd1[8*position + 3] = 0xFF;
		else if (val < 95)
			lcd1[8*position + 3] = 4;
		else if (val < 108)
			lcd1[8*position + 3] = 6;
		else if (val >= 108)
			lcd1[8*position + 3] = ' ';

		// 4th digit
		if (val < 121)
			lcd1[8*position + 4] = 2;
		else if (val < 134)
			lcd1[8*position + 4] = '|';
		else if (val >= 134)
			lcd1[8*position + 4] = 5;

		// 5th digit
		if (val >= 173)
			lcd1[8*position + 5] = 0xFF;
		else if (val >= 160)
			lcd1[8*position + 5] = 3;
		else if (val >= 147)
			lcd1[8*position + 5] = 1;
		else if (val < 147)
			lcd1[8*position + 5] = ' ';

		// 6th digit
		if (val >= 199)
			lcd1[8*position + 6] = 0xFF;
		else if (val >= 186)
			lcd1[8*position + 6] = 2;
		else if (val >= 173)
			lcd1[8*position + 6] = 0;
		else if (val < 173)
			lcd1[8*position + 6] = ' ';

		// 7th digit
		if (val >= 238)
			lcd1[8*position + 7] = 0xFF;
		else if (val >= 225)
			lcd1[8*position + 7] = 3;
		else if (val >= 212)
			lcd1[8*position + 7] = 1;
		else if (val < 212)
			lcd1[8*position + 7] = ' ';
	}
	else
	{
		// 1st digit
		if (val < 17)
			lcd2[8*(position-2) + 1] = 0xFF;
		else if (val < 30)
			lcd2[8*(position-2) + 1] = 4;
		else if (val < 43)
			lcd2[8*(position-2) + 1] = 6;
		else if (val >= 43)
			lcd2[8*(position-2) + 1] = ' ';

		// 2nd digit
		if (val < 56)
			lcd2[8*(position-2) + 2] = 0xFF;
		else if (val < 69)
			lcd2[8*(position-2) + 2] = 5;
		else if (val < 82)
			lcd2[8*(position-2) + 2] = 7;
		else if (val >= 82)
			lcd2[8*(position-2) + 2] = ' ';

		// 3rd digit
		if (val < 82)
			lcd2[8*(position-2) + 3] = 0xFF;
		else if (val < 95)
			lcd2[8*(position-2) + 3] = 4;
		else if (val < 108)
			lcd2[8*(position-2) + 3] = 6;
		else if (val >= 108)
			lcd2[8*(position-2) + 3] = ' ';

		// 4th digit
		if (val < 121)
			lcd2[8*(position-2) + 4] = 2;
		else if (val < 134)
			lcd2[8*(position-2) + 4] = '|';
		else if (val >= 134)
			lcd2[8*(position-2) + 4] = 5;

		// 5th digit
		if (val >= 173)
			lcd2[8*(position-2) + 5] = 0xFF;
		else if (val >= 160)
			lcd2[8*(position-2) + 5] = 3;
		else if (val >= 147)
			lcd2[8*(position-2) + 5] = 1;
		else if (val < 147)
			lcd2[8*(position-2) + 5] = ' ';

		// 6th digit
		if (val >= 199)
			lcd2[8*(position-2) + 6] = 0xFF;
		else if (val >= 186)
			lcd2[8*(position-2) + 6] = 2;
		else if (val >= 173)
			lcd2[8*(position-2) + 6] = 0;
		else if (val < 173)
			lcd2[8*(position-2) + 6] = ' ';

		// 7th digit
		if (val >= 238)
			lcd2[8*(position-2) + 7] = 0xFF;
		else if (val >= 225)
			lcd2[8*(position-2) + 7] = 3;
		else if (val >= 212)
			lcd2[8*(position-2) + 7] = 1;
		else if (val < 212)
			lcd2[8*(position-2) + 7] = ' ';
	}
	
}


void power_off()
{		
	uint8_t addr_temp;
	
	// Save trim
	addr_temp = 15*cur_model + 1;
	eeprom_update_byte((uint8_t*)addr_temp,       trim[AILERON]);
	eeprom_update_byte((uint8_t*)(addr_temp + 1), trim[ELEVATOR]);
	eeprom_update_byte((uint8_t*)(addr_temp + 2), trim[RUDDER]);
	eeprom_update_byte((uint8_t*)(addr_temp + 3), trim[CH5]);
	_delay_ms(20);
	
	
	// LCD	
	LCD_home();
	LCD_string("      BYE!      ", 16);
	LCD_NWL();
	LCD_string("                ", 16);
	
	
	// Power off sound effect
	buzzer(900);
	_delay_ms(300);
	buzzer(600);
	_delay_ms(200);
	buzzer(0);
	_delay_ms(300);
	
	// Turn Power regulator off
	while (1)
	{
		PORTD &= ~PWRTOGL;
		_delay_ms(1000);
	}
	
}