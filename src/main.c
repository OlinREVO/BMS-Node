#define F_CPU (1000000L)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "api.h"


#define MAXV ((uint16_t) (3.5/ 5.0 * 0x3FF))
#define MINV ((uint16_t) (2.7/ 5.0 * 0x3FF))

// inputs determines which ADC to use to read cell voltage
uint8_t inputs[] = { 0x06, 0x05, 0x03, 0x04, 0x01, 0x02};
// outputs determines which pin in PORTB to use to shunt the cell
uint8_t outputs[] = { _BV(PD1), _BV(PC0), _BV(PD0), _BV(PB3), _BV(PB4), _BV(PC7)};

volatile uint32_t millis = 0; //Accurate to within ~5% for RC oscillator
volatile uint8_t timer1_counter = 0;

void init_io_pins() {
    // Set all of PORTB (except PB7 - reading from cell) to output low
    DDRB = 0x7F;
    PORTB &= 0x80;

    //setting inputs
    DDRD = 0x00;
    DDRD &= ~(_BV(PD5)); //input 1
    DDRD &= ~(_BV(PD6));
    DDRB &= ~(_BV(PB2));
    DDRB &= ~(_BV(PB7));
    DDRB &= ~(_BV(PB6));
    DDRB &= ~(_BV(PB5));

    //setting outputs
    DDRD |= _BV(PD1);
    DDRC |= _BV(PC0);
    DDRD |= _BV(PD0);
    DDRB |= _BV(PB3);
    DDRB |= _BV(PB4);
    DDRC |= _BV(PC7);
}

void initADC() {
    //Enable ADC, set prescalar to 128 (slow down ADC clock)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    //Enable internal reference voltage
    ADCSRB &= _BV(AREFEN);
    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);
}
//
uint16_t read_channel(uint8_t channel) {
  //Reset the ADMUX channel select bits (lowest 5)
  ADMUX &= ~(0x1F);
  //the low 4 bits of ADMUX select the ADC channel
  ADMUX |= inputs[channel];
  //Wait for ADC reading
  ADCSRA |=  _BV(ADSC);
  while(bit_is_set(ADCSRA, ADSC));

  //ADC is a macro to combine ADCL and ADCH
  return ADC;
}

void init_millis() {
    TCCR1B |= (_BV(WGM12) | _BV(CS10));
    TIMSK1 |= (1<<OCIE1A);
    OCR1A = 248;
}

ISR(TIMER1_COMPA_vect){
    timer1_counter++;
    if (timer1_counter >= 4) {
      millis++;
      /*PORTB ^= _BV(PB1);*/
      timer1_counter = 0;
    }
}

int main (void) {
    init_io_pins();
    init_millis();
    initADC();
    initCAN(NODE_bms);

    uint8_t ch; //Selects ADC Channel and PORTB write output
    uint8_t msg[1];
    uint32_t old_millis = 0;

    //Loop Begins
    for (;;) {
        if ((uint32_t)(millis - old_millis) > (uint32_t) 499) {
            PORTB ^= _BV(PB1);
            old_millis = millis;
        }
        //Check each of the 6 cells
        for (ch = 0; ch < 6; ch++) {

            uint16_t voltage = read_channel(ch);

            if (voltage >= MAXV){
                if (ch == 0 || ch == 2) { PORTD |= outputs[ch];}
                if (ch == 1 || ch == 5) { PORTC |= outputs[ch];}
                if (ch == 3 || ch == 4) { PORTB |= outputs[ch];}
                msg[0] = 0;
                sendCANmsg(NODE_watchdog,MSG_shunting,msg,1);
            }
            else if (voltage <= MINV) {
                if (ch == 0 || ch == 2) { PORTD &= ~outputs[ch];}
                if (ch == 1 || ch == 5) { PORTC &= ~outputs[ch];}
                if (ch == 3 || ch == 4) { PORTB &= ~outputs[ch];}
                //Sending low voltage signal over CAN
                msg[0] = 0;
                sendCANmsg(NODE_watchdog,MSG_voltagelow,msg,1);
            }
            else {
                if (ch == 0 || ch == 2) { PORTD &= ~outputs[ch];}
                if (ch == 1 || ch == 5) { PORTC &= ~outputs[ch];}
                if (ch == 3 || ch == 4) { PORTB &= ~outputs[ch];}
            }
            _delay_ms(1);
        }
    }
    return 1;
}

void handleCANmsg(uint8_t destID, uint8_t msgID, uint8_t* msg, uint8_t msgLen) {
    PORTB ^= _BV(PB0);
}

