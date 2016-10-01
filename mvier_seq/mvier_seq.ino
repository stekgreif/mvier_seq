/** ****************************************************************************
	@date	2016-09-30
	@author	dan@stekgreif.com
	@brief	- Arduino Micro
			- Atmega32U4
			- 64 RGBW LEDS (Neopixel)
			- 64 buttons via shift register (74HC165)

	MIDI Messages:
	90	Note On			Buttons, LEDs
	B0	Control Change	Analog Inputs
	F0	Channel Mode	System

	RGBW Commands:
	Ch	Cmd
	0	R
	1	G
	2	B
	3	W
	4	Color Table (val: 0..127)
	5	Update
	6	All RGBWs
	7	All off
	*NOTE:	Commands via Ch 0..4 need the update command to be shown
			Ch 7 is updated automaticly
*******************************************************************************/
#include <SPI.h>
#include "MIDIUSB.h"
#include "Adafruit_NeoPixel.h"
#ifdef __AVR__
#include <avr/power.h>
#endif

#define SPI_SCK 13  //grau 165_P2
#define SPI_SAP 10  //lila 165_P1
#define SPI_SDI 12  //blau 165_P9

#define BTN_MASK 0b00000001
#define BTN_DEAD_TIME  100
#define NUM_REGS 8
#define NUM_BTNS (NUM_REGS * 8)

#define PIN            7
#define NUMPIXELS      64
#define _RED 1
#define _GRN 2
#define _BLU 3
#define _WHT 0


uint8_t col_table[6][4] = {
	{ 50,  0,  0,  0},	// 0 red	relsub0
	{  0, 50,  0,  0},	// 1 green	relsub1
	{  0,  0, 50,  0},	// 2 blue	relsub2
	{ 40,  0, 40,  0},	// 3 purple	relsub3
	{  0,  0,  0, 50},	// 4 white	stepled
	{  0,  0,  0,  0},	// 5 off
};

uint32_t currentMillis  = 0;
uint32_t previousMillis = 0;
uint8_t  sched_cnt = 0;
uint8_t  btn_input[8] = {};
uint8_t  btn_debounce_array[64] = {};
uint8_t  usb_midi_msg_cnt = 0;

uint8_t color = 0;
uint8_t id = 0;
uint8_t brightness = 0;
uint8_t col_tab_pos = 0;
uint8_t prevColor[4] = {};
int16_t potVal = 0;
int16_t lastPotVal = 0;

uint8_t rgbw_dirty_flag = 0;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);




/** ****************************************************************************
	inits spi bus for buttons and clears leds
*******************************************************************************/
void setup()
{
	Serial.begin(115200);
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);

	pinMode(SPI_SAP, OUTPUT);
	digitalWrite(SPI_SAP, HIGH);

	pixels.begin();
	pixels.show();
}



/** ****************************************************************************

*******************************************************************************/
void buttons_read(void)
{
	digitalWrite(SPI_SAP, LOW);
	digitalWrite(SPI_SAP, HIGH);

	btn_input[0] = SPI.transfer(0);
	btn_input[1] = SPI.transfer(1);
	btn_input[2] = SPI.transfer(2);
	btn_input[3] = SPI.transfer(3);
	btn_input[4] = SPI.transfer(4);
	btn_input[5] = SPI.transfer(5);
	btn_input[6] = SPI.transfer(6);
	btn_input[7] = SPI.transfer(7);
}



/** ****************************************************************************
	simple debouncing mechanism; only button pressing, not releasing
	btn 01234567
*******************************************************************************/
void buttons_check(void)
{
	for (uint8_t reg = 0; reg < NUM_REGS; reg++)
	{
		for (uint8_t btn = 0; btn < 8; btn++)
		{
			if ( (btn_input[reg] & (BTN_MASK << btn ) ) == 0)
			{
				uint8_t button_id = 8 * reg + (7 - btn);	//0..63
				if ( btn_debounce_array[button_id] == 0 )
				{
					// write to usb midi buffer
					midiEventPacket_t event = {0x09, 0x90, button_id, 1};
					MidiUSB.sendMIDI(event);
					usb_midi_msg_cnt++;
					btn_debounce_array[button_id] = BTN_DEAD_TIME;
				}
			}
		}
	}
}


/** ****************************************************************************

*******************************************************************************/
void buttons_debounce(void)
{
	for (uint8_t btn = 0; btn < NUM_BTNS; btn++ )
	{
		if( btn_debounce_array[btn] > 0)
			btn_debounce_array[btn]--;
	}
}



/** ****************************************************************************

*******************************************************************************/
void flush_usb_midi(void)
{
	if ( usb_midi_msg_cnt > 0 )
	{
		MidiUSB.flush();
		usb_midi_msg_cnt = 0;
	}
}



/** ****************************************************************************
	@return	false:	there was no midi data in the buffer
			true:	there was midi data in the buffer
*******************************************************************************/
bool rgbw_usb_midi_process()
{
	midiEventPacket_t rx;
	rx = MidiUSB.read();

	if (rx.header == 0x9) // NOTE ON
	{
		color = rx.byte1 & 0b00001111;
		id = rx.byte2;
		brightness = rx.byte3 << 1;
    col_tab_pos = rx.byte3 & 0b01111111;

		uint32_t prevCol = pixels.getPixelColor(id);
		prevColor[_BLU] = (uint8_t)  prevCol;
		prevColor[_GRN] = (uint8_t) (prevCol >> 8);
		prevColor[_RED] = (uint8_t) (prevCol >> 16);
		prevColor[_WHT] = (uint8_t) (prevCol >> 24);


		// set pixel color
		switch (color)
		{
			case 0: // red
			{
				pixels.setPixelColor(id, pixels.Color(brightness, prevColor[_GRN], prevColor[_BLU], prevColor[_WHT]));
				break;
			}
			case 1: // green
			{
				pixels.setPixelColor(id, pixels.Color(prevColor[_RED], brightness, prevColor[_BLU], prevColor[_WHT]));
				break;
			}
			case 2: // blue
			{
				pixels.setPixelColor(id, pixels.Color(prevColor[_RED], prevColor[_GRN], brightness, prevColor[_WHT]));
				break;
			}
			case 3: // white
			{
				pixels.setPixelColor(id, pixels.Color(prevColor[_RED], prevColor[_GRN], prevColor[_BLU], brightness));
				break;
			}
			case 4: // color table
			{
				if( rx.byte3 < 6 )
				{
					pixels.setPixelColor( id, pixels.Color( col_table[col_tab_pos][0], col_table[col_tab_pos][1], col_table[col_tab_pos][2], col_table[col_tab_pos][3] ));
				}
				break;
			}
			case 5: // update
			{
				pixels.show();
				break;
			}
			case 6: // all RGBWs
			{
				break;
			}
			case 7: // ALL OFF
			{
				for( uint8_t i = 0; i < 64; i++ )
				{
					pixels.setPixelColor(i, pixels.Color(0, 0, 0, 0));
				}
				pixels.show();
				break;
			}
		}
		return true;
	}

	else
		return false;
}




/** ****************************************************************************

*******************************************************************************/
void loop()
{
	currentMillis = millis();

	while(rgbw_usb_midi_process());

	if ( currentMillis != previousMillis )
	{
		sched_cnt++;

		switch (sched_cnt)
		{
			case 1:
			buttons_read();
			break;

			case 2:
			buttons_check();
			break;

			case 3:
			flush_usb_midi();
			break;

			case 4:
			buttons_debounce();
			sched_cnt = 0;
			break;
		}

		previousMillis = currentMillis;
	}
}



