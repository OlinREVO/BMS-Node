#define F_IO 1000000
#define F_CPU 1000000

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <math.h>
#include <util/delay.h>
//#include <stdint.h>
//#include <inttypes.h>
#define MAXV ((int) (3.5/ 5.0 * 0x3FF))
#define OVER_VOLTAGE_LED (_BV(PB6) | _BV(PB7)) //Gets turned on, stays on


void chip_init () {
    //Enable ADC, set prescalar to 128 (slow down ADC clock)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);
    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    // Set all of PORTB to output low
    DDRB |= 0xFF;
    PORTB = 0x00;
}

int main ()
{

    uint8_t ch = 0; //Selects ADC Channel and PORTB write output
    int counter = 0;
    chip_init();


    //Loop Begins
    _delay_ms(10);
    /*for (;;) {
        ch++;
        PORTB &= 0x00;
        PORTB |=(uint8_t) (ch);
        _delay_ms(100);
    }*/

   for (;;) {
        //Check each of the 4 cells
        for (ch = 0; ch < 4; ch++) {

            //the low 4 bits of ADMUX select the ADC channel
            ADMUX &= 0xF8; //Reset low 4 bits
            ADMUX |= ch;
            //Wait for ADC reading
            ADCSRA |=  _BV(ADSC);
            while(bit_is_set(ADCSRA, ADSC));

            //ADC is a macro to combine ADCL and ADCH
            //ch selects bits of PORTB (0 - 3)
            if (ADC >= MAXV) {
                PORTB |= (1 << ch);
                PORTB |= OVER_VOLTAGE_LED;
            }
            else {
                PORTB &= ~(1 << ch);
            }

            counter++;
            if (counter == 10000) { //Every ~20 seconds?
              counter = 0;
              PORTB &= ~OVER_VOLTAGE_LED;

            }
        }

    }

    return 1;
}

