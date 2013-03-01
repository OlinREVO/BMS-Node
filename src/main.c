#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MAXV ((uint16_t) (4.0/ 5.0 * 0x3FF))

int main (void)
{
    //Enable ADC, set prescalar to 128 (slow down ADC clock)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);
    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    //ADC0 is pin 11
    uint8_t ch = 0;
    //the low 4 bits of ADMUX select the ADC channel
    ADMUX |= ch;

    // Set PB1 to output
    DDRB |= 0xFF;
    PORTB |= _BV(PB7);


    //Loop Begins
    for (;;) {
        /* toggle PORTB.2 pins */
        //PORTB ^= 0xFF;

        //Read pin 11 (set by ch above)
        ADCSRA |=  _BV(ADSC);
        while(bit_is_set(ADCSRA, ADSC));

        //ADC is a macro to combine ADCL and ADCH
        if (ADC >= MAXV)
          PORTB |= _BV(PB1); //All on
        else
          PORTB &= ~(_BV(PB1)); //All off

        _delay_ms(5);

    }

    return 1;


}
