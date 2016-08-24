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

uint32_t currentMillis  = 0;
uint32_t previousMillis = 0;
uint8_t  sched_cnt = 0;
uint8_t  btn_input[8] = {};
uint8_t  btn_debounce_array[64] = {};
uint8_t  usb_midi_msg_cnt = 0;

uint8_t color = 0;
uint8_t id = 0;
uint8_t brightness = 0;
uint8_t prevColor[4] = {};
int16_t potVal = 0;
int16_t lastPotVal = 0;

uint8_t rgbw_dirty_flag = 0;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);





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


void buttons_check(void)
{
  // btn 01234567
  for (uint8_t reg = 0; reg < NUM_REGS; reg++)
  {
    for (uint8_t btn = 0; btn < 8; btn++)
    {
      if ( (btn_input[reg] & (BTN_MASK << btn ) ) == 0)
      {
        uint8_t button_id = 8 * reg + (7 - btn);	//0..63
        if ( btn_debounce_array[button_id] == 0 )
        {
          Serial.println( button_id );
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


void buttons_debounce(void)
{
  for ( uint8_t btn = 0; btn < NUM_BTNS; btn++ )
  {
    if (	btn_debounce_array[btn] > 0)
      btn_debounce_array[btn]--;
  }
}


void flush_usb_midi(void)
{
  if ( usb_midi_msg_cnt > 0 )
  {
    MidiUSB.flush();
    usb_midi_msg_cnt = 0;
  }
}



void usb_midi_read()
{
  midiEventPacket_t rx;


  rx = MidiUSB.read();

  if (rx.header == 0xB) // MIDI_CC
  {
    color = rx.byte1 & 0b00001111;
    id = rx.byte2;
    brightness = rx.byte3 << 1;

    uint32_t prevCol = pixels.getPixelColor(id);
    prevColor[_BLU] = (uint8_t)  prevCol;
    prevColor[_GRN] = (uint8_t) (prevCol >> 8);
    prevColor[_RED] = (uint8_t) (prevCol >> 16);
    prevColor[_WHT] = (uint8_t) (prevCol >> 24);

    //Serial.print("LED: color: ");

    // set pixel color
    switch (color)
    {
      case 0: // RED
        {
          pixels.setPixelColor(id, pixels.Color(brightness, prevColor[_GRN], prevColor[_BLU], prevColor[_WHT]));
          //Serial.print("red");
          rgbw_dirty_flag = 1;
          break;
        }
      case 1: // GREEN
        {
          pixels.setPixelColor(id, pixels.Color(prevColor[_RED], brightness, prevColor[_BLU], prevColor[_WHT]));
          //Serial.print("green");
          rgbw_dirty_flag = 1;
          break;
        }
      case 2: // BLUE
        {
          pixels.setPixelColor(id, pixels.Color(prevColor[_RED], prevColor[_GRN], brightness, prevColor[_WHT]));
          //Serial.print("blue");
          rgbw_dirty_flag = 1;
          break;
        }
      case 3: // WHITE
        {
          pixels.setPixelColor(id, pixels.Color(prevColor[_RED], prevColor[_GRN], prevColor[_BLU], brightness));
          //Serial.print("white");
          rgbw_dirty_flag = 1;
          break;
        }
      case 4: // ALL COLORS
        {
          pixels.setPixelColor(id, pixels.Color(brightness, brightness, brightness, brightness));
          //Serial.print("all colors");
          rgbw_dirty_flag = 1;
          break;
        }
      case 5: // ALL OFF
        {
          for( uint8_t i = 0; i < 64; i++ )
          {
            pixels.setPixelColor(i, pixels.Color(0, 0, 0, 0));
          }
          rgbw_dirty_flag = 1;
          break;
        }
    }
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
        usb_midi_read();
        break;

      case 5:
        if ( rgbw_dirty_flag )
        {
          pixels.show();
          rgbw_dirty_flag = 0;
        }
        break;

      case 6:
        buttons_debounce();
        sched_cnt = 0;
        break;
    }

    previousMillis = currentMillis;
  }
}



