#include <SPI.h>
#include "MIDIUSB.h"

#define SPI_SCK 13  //grau 165_P2
#define SPI_SAP 10  //lila 165_P1
#define SPI_SDI 12  //blau 165_P9

#define BTN_MASK 0b00000001
#define BTN_DEAD_TIME  100
#define NUM_REGS 8
#define NUM_BTNS (NUM_REGS * 8)

uint32_t currentMillis  = 0;
uint32_t previousMillis = 0;
uint8_t  sched_cnt = 0;
uint8_t  btn_input[8] = {};
uint8_t  btn_debounce_array[64] = {};
uint8_t  usb_midi_msg_cnt = 0;


void setup()
{
	Serial.begin(115200);
	SPI.begin();
	SPI.setDataMode(SPI_MODE3);
	
	pinMode(SPI_SAP, OUTPUT);
	digitalWrite(SPI_SAP, HIGH);
	
}



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
	
	Serial.println("test"); 
}


void buttons_check(void)
{
	// btn 01234567
	for(uint8_t reg = 0; reg < NUM_REGS; reg++)
	{
		for(uint8_t btn = 0; btn < 8; btn++)
		{
			if( (btn_input[reg] & (BTN_MASK << btn ) ) == 0)
			{
				uint8_t button_id = 8*reg + (7-btn);	//0..63
				if( btn_debounce_array[button_id] == 0 )
				{	
					Serial.println( button_id ); 
					// write to usb midi buffer
					//midiEventPacket_t event = {0x09, 0x90, button_id, 1};
					//MidiUSB.sendMIDI(event);
					//usb_midi_msg_cnt++;
					btn_debounce_array[button_id] = BTN_DEAD_TIME;
				}
			}
		}
	}
}


void buttons_debounce(void)
{
	for( uint8_t btn = 0; btn < NUM_BTNS; btn++ )
	{
		if(	btn_debounce_array[btn] > 0)
			btn_debounce_array[btn]--;
	}
}


void flush_usb_midi(void)
{
	if( usb_midi_msg_cnt > 0 )
	{
		//flush usb midi
	}
}


void loop()
{
	currentMillis = millis();

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



