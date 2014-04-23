#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define MAXV ((int) (3.7/ 5.0 * 0x3FF))
#define MINV ((int) (2.7/ 5.0 * 0x3FF))

int sendCANmsg(int destID, int msgID, char msg[], int msgLen);

// inputs determines which ADC to use to read cell voltage
uint8_t inputs[] = { 0x06, 0x05, 0x03, 0x04, 0x01, 0x02};
// outputs determines which pin in PORTB to use to shunt the cell
uint8_t outputs[] = { _BV(PD1), _BV(PC0), _BV(PD0), _BV(PB3), _BV(PB4), _BV(PC7)};

uint8_t ports[] = {PORTD, PORTC, PORTD, PORTB, PORTB, PORTC}

int main (void) {
    //Enable ADC, set prescalar to 128 (slow down ADC clock)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);
    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    // Set all of PORTB (except PB7 - reading from cell) to output low
    DDRB = 0x7F;

    PORTB &= 0x80;

    DDRD = 0x00;

    //setting inputs 
    DDRD &=~_BV(PD5); //input 1
    DDRD &=~_BV(PD6);
    DDRB &=~_BV(PB2);
    DDRB &=~_BV(PB7);
    DDRB &=~_BV(PB6);
    DDRB &=~_BV(PB5); 

    //setting outputs 
    DDRD |=_BV(PD1); 
    DDRC |=_BV(PC0); 
    DDRD |=_BV(PD0);
    DDRB |=_BV(PB3);
    DDRB |=_BV(PB4);
    DDRC |=_BV(PC7);



    uint8_t ch; //Selects ADC Channel and PORTB write output
    //Loop Begins
    for (;;) {
        //Check each of the 6 cells
        for (ch = 0; ch < 6; ch++) {

            //Reset the ADMUX channel select bits (lowest 5)
            ADMUX &= ~(0x1F);
            //the low 4 bits of ADMUX select the ADC channel
            ADMUX |= inputs[ch];
            //Wait for ADC reading
            ADCSRA |=  _BV(ADSC);
            while(bit_is_set(ADCSRA, ADSC));

            //ADC is a macro to combine ADCL and ADCH
            uint16_t voltage = ADC;

            if (voltage >= MAXV){
                ports[ch] |= outputs[ch];
                sendCANmsg(NODE_watchdog,MSG_shunting,'0',1);
            }
            else if (voltage <= MINV) {
                ports[ch] &= ~outputs[ch];
                //Sending low voltage signal over CAN
                sendCANmsg(NODE_watchdog,MSG_voltagelow,'0',1);
            }
            else {
                ports[ch] &= ~outputs[ch];
            }
            _delay_ms(1);
        }
    }
    return 1;
}

