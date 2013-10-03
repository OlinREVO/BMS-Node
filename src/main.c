#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MAXV ((int) (3.7/ 5.0 * 0x3FF))
#define MINV ((int) (2.7/ 5.0 * 0x3FF))

#define OVER_VOLTAGE_LED _BV(PB7) //Gets turned on, stays on
int main (void)
{
    //Enable ADC, set prescalar to 128 (slow down ADC clock)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);
    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    // Set all of PORTB to output low
    DDRB |= 0xFF;
    PORTB &= ~0x00;
    PORTB |= _BV(PB6);

    uint8_t ch; //Selects ADC Channel and PORTB write output
    //Loop Begins
    for (;;) {
        //Check each of the 4 cells
        for (ch = 0; ch < 4; ch++) {

            //the low 4 bits of ADMUX select the ADC channel
            ADMUX |= ch;
            //Wait for ADC reading
            ADCSRA |=  _BV(ADSC);
            while(bit_is_set(ADCSRA, ADSC));

            //ADC is a macro to combine ADCL and ADCH
            //ch selects bits of PORTB (0 - 3)
            if (ADC >= MAXV) {
                PORTB |= (1 << ch);
                PORTB |= OVER_VOLTAGE_LED;
                sendCANmsg(0,4,'0',1);
            }
            else if (ADC <= MINV){
                //Sending low voltage signal over CAN
                //int sendCANmsg(int destID, int msgID, char msg[], int msgLen);
                sendCANmsg(0,3,'0',1);
            }
            else {
                PORTB &= ~(1 << ch);
            }
            _delay_ms(1);
        }

    }

    return 1;
}

int sendCANmsg(int destID, int msgID, char msg[], int msgLen) {

}

