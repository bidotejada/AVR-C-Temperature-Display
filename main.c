/*
 * FinalProject.c
 *
 * Created: 4/19/2021 9:21:50 AM
 * Author : Willy Bido
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sfr_defs.h> //loop_until_bit_is_clear ADC
#include <stdlib.h>       //for dtostrf() function
#include <stdbool.h>      //for booleans

#include "lcd.h"

void setup()
{

    DDRB |= (0 << PORTB0);  //set PORTB0 as input
    PORTB |= (1 << PORTB0); //enable pull-up resistor PORTB0

    lcd_init();
    lcd_clear_display();
    lcd_return_home();

    //configure ADC
    DIDR0 |= (1 << ADC0D);                                          //disable digital input A0
    ADMUX |= (0 << REFS1) | (1 << REFS0);                           //AVcc 5 volt ref Vcc
    ADMUX |= (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0); //ADC0 (A0) being used
    ADMUX |= (0 << ADLAR);                                          //right shift 10-bit resolution
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);           //prescalar 128
    ADCSRA |= (1 << ADEN);                                          //enable ADC
}

int main(void)
{
    setup();

    float val;
    float mv;
    float cel;
    float farh;

    char tempc[6];
    char tempf[6];

    bool bttn_state = false;

    while (1)
    {

        ADCSRA |= (1 << ADSC); //start conversion
        _delay_ms(20);
        loop_until_bit_is_clear(ADCSRA, ADSC);

        val = ADC;
        mv = val * (5000.0 / 1023.0);
        cel = mv / 10.0;
        farh = (cel * 9.0) / 5.0 + 32.0;

        dtostrf(cel, 5, 2, tempc); //convert value to string, save in "temp"
        dtostrf(farh, 5, 2, tempf);

        if (!(PINB & (1 << PINB0)))
        {
            _delay_ms(100); //debounce

            if (!(PINB & (1 << PINB0)))
            {
                //switch between far and cel
                if (bttn_state == false)
                {

                    lcd_clear_display();
                    lcd_go_to_xy(2, 0);
                    lcd_puts("Temperature:");
                    lcd_go_to_xy(0, 1);
                    lcd_go_to_xy(4, 1);
                    lcd_puts(tempc);
                    lcd_puts("\"C");

                    bttn_state = true;
                }
                else
                {

                    lcd_clear_display();
                    lcd_go_to_xy(2, 0);
                    lcd_puts("Temperature:");
                    lcd_go_to_xy(0, 1);
                    lcd_go_to_xy(4, 1);
                    lcd_puts(tempf);
                    lcd_puts("\"F");

                    bttn_state = false;
                }
            }
        }
    }
}
