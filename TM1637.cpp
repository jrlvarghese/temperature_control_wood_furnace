
//  Author: avishorp@gmail.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include "TM1637.h"
#include <Arduino.h>
// #include <String.h>

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t char_arr[26][1] = {
    // XGFEDCBA
    {0b01110111},    // A
    {0b01111100},    // b
    {0b00111001},    // C
    {0b01011110},    // d
    {0b01111001},    // E
    {0b01110001},    // F
    {0b00001000},     // G (blank)
    {0b01110110},     // H
    {0b00110000},     // I
    {0b00011110},     // J
    {0b00001000},     // K (blank)
    {0b00111000},     // L
    {0b00001000},     // M (blank)
    {0b01010100},     // n
    {0b00111111},     // O
    {0b01110011},     // P
    {0b00000000},     // Q (blank)
    {0b00001000},     // R (blank)
    {0b01101101},     // S
    {0b01111000},     // t
    {0b00111110},     // U
    {0b00001000},     // V (blank)
    {0b00001000},     // W (blank)
    {0b00001000},     // X (blank)
    {0b01101110},     // y
    {0b00001000}      // Z (blank)
};
const uint8_t t_letter[] = {0b01111000};
const uint8_t h_letter[] = {0b01110110};
const uint8_t c_letter[] = {0b00111001};

const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };

static const uint8_t minusSegments = 0b01000000;

TM1637::TM1637(uint8_t pinClk, uint8_t pinDIO, unsigned int bitDelay)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;
	m_bitDelay = bitDelay;

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
    pinMode(m_pinClk, INPUT);
    pinMode(m_pinDIO,INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}

void TM1637::setBrightness(uint8_t brightness, bool on)
{
	m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
}

void TM1637::setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();

	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k=0; k < length; k++)
	  writeByte(segments[k]);

	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}

void TM1637::clear()
{
    uint8_t data[] = { 0, 0, 0, 0 };
	setSegments(data);
}

void TM1637::showNumberDec(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
  showNumberDecEx(num, 0, leading_zero, length, pos);
}

void TM1637::showNumberDecEx(int num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  showNumberBaseEx(num < 0? -10 : 10, num < 0? -num : num, dots, leading_zero, length, pos);
}

void TM1637::showNumberHexEx(uint16_t num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
  showNumberBaseEx(16, num, dots, leading_zero, length, pos);
}

void TM1637::showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, bool leading_zero,
                                    uint8_t length, uint8_t pos)
{
    bool negative = false;
	if (base < 0) {
	    base = -base;
		negative = true;
	}


    uint8_t digits[4];

	if (num == 0 && !leading_zero) {
		// Singular case - take care separately
		for(uint8_t i = 0; i < (length-1); i++)
			digits[i] = 0;
		digits[length-1] = encodeDigit(0);
	}
	else {
		//uint8_t i = length-1;
		//if (negative) {
		//	// Negative number, show the minus sign
		//    digits[i] = minusSegments;
		//	i--;
		//}
		
		for(int i = length-1; i >= 0; --i)
		{
		    uint8_t digit = num % base;
			
			if (digit == 0 && num == 0 && leading_zero == false)
			    // Leading zero is blank
				digits[i] = 0;
			else
			    digits[i] = encodeDigit(digit);
				
			if (digit == 0 && num == 0 && negative) {
			    digits[i] = minusSegments;
				negative = false;
			}

			num /= base;
		}
    }
	
	if(dots != 0)
	{
		showDots(dots, digits);
	}
    
    setSegments(digits, length, pos);
}

void TM1637::bitDelay()
{
	delayMicroseconds(m_bitDelay);
}

void TM1637::start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}

void TM1637::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}

bool TM1637::writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();

	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);

    bitDelay();

	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }

  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();

  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint8_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);


  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();

  return ack;
}

void TM1637::showDots(uint8_t dots, uint8_t* digits)
{
    for(int i = 0; i < 4; ++i)
    {
        digits[i] |= (dots & 0x80);
        dots <<= 1;
    }
}

uint8_t TM1637::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}


void TM1637::showReadingWithUnit(int temp, char unit)
{
    clear();
    if(temp/100 == 0){
        showNumberDec(temp, false, 2, 0);
    }else{
        showNumberDec(temp, false, 3, 0);
    }
    setSegments(char_arr[(int)unit-65], 1, 3);
    
}

// show menu selection 
void TM1637::show_menu_option(int menu_item)
{
  clear(); // clear the display
  setSegments(char_arr[(int)'P'-65], 1, 0); // load letter p
  showNumberDec(menu_item+1, false, 1, 1); // load the menu number
}

// uint8_t *TM1637::encodeLetter(char c, uint8_t *arr_add)
// {
//   return arr_add[(int)c-65];
// }



// void TM1637::showString(String word)
// {
//     clear();
//     for(int i=0; i<word.length(); i++){
//         setSegments(char_arr[(int)word[i]-65], 1, i);

//     }

// }