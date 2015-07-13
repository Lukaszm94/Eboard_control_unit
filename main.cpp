/*
 * main.cpp
 *
 *  Created on: 7 lip 2015
 *      Author: Lukasz
 */

#include <avr/io.h>
#include <stdio.h>
#include "uart.h"
#include "lcd.h"

#define TEMP1_ADC_CHANNEL 0
#define TEMP2_ADC_CHANNEL 1

void initADC()
{
	ADMUX |= (1<<REFS1) | (1<<REFS0); //1.1V ref
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable ADC, 128div factor
}

uint16_t readADC(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF0) | channel; //set appropiate channel
	ADCSRA |= (1<<ADSC); //start conversion
	while(ADCSRA & (1<<ADSC)); //wait till the end of conversion

	return ADC;
}

float convertRawADCToTemp(uint16_t raw)
{
	float result = (1.1 * (float)raw)/10.24;
	return result;
}

void printTemperatures(float t1, float t2)
{
	LCD_clear();
	LCD_str("T1: ");
	LCD_int(t1);
	LCD_goto(0,1); //goto second line
	LCD_str("T2: ");
	LCD_int(t2);
}

int main()
{
	uint16_t t1RawADC = 0;
	uint16_t t2RawADC = 0;
	float temp1 = 0;
	float temp2 = 0;

	initADC();
	lcd_init();
	while(1)
	{
		t1RawADC = readADC(TEMP1_ADC_CHANNEL);
		t2RawADC = readADC(TEMP2_ADC_CHANNEL);

		temp1 = convertRawADCToTemp(t1RawADC);
		temp2 = convertRawADCToTemp(t2RawADC);

		printTemperatures(temp1, temp2);
//		LCD_clear();
//		LCD_int(temp1);

		_delay_ms(100);
	}

}


