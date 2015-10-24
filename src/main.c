#define F_CPU (1000000L)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "api.h"


#define MAXV ((uint16_t) (3.5/ 5.0 * 0x3FF))
#define MINV ((uint16_t) (2.7/ 5.0 * 0x3FF))

// inputs determines which ADC to use to read cell voltage
uint8_t inputs[] = { 0x02, 0x03, 0x05, 0x04, 0x07, 0x06};
// outputs determines which pin to use to shunt the cell
uint8_t outputs[] = { _BV(PD1), _BV(PC0), _BV(PD0), _BV(PB3), _BV(PB4), _BV(PC7)};

uint8_t shunt[] = { 0, 0, 0, 0, 0, 0 };

volatile uint32_t millis = 0; //Accurate to within ~10% for RC oscillator
volatile uint8_t timer1_counter = 0;

#define CAPTURE_STEP 1000
uint32_t capture_time = 0;
#define LED_LONG_STEP 500
#define LED_SHORT_STEP 150
uint32_t led_time = 0;
uint32_t led_step = 0;
uint8_t led_index = 0;
uint8_t led_progress = 0;

void init_io_pins() {
    //setting inputs
    DDRD &= ~(_BV(PD5)); //input 1
    DDRD &= ~(_BV(PD6));
    DDRB &= ~(_BV(PB2));
    DDRB &= ~(_BV(PB7));
    DDRB &= ~(_BV(PB6));
    DDRB &= ~(_BV(PB5));

    //setting outputs
    DDRD |= _BV(PD0);
    DDRD |= _BV(PD1);
    DDRC |= _BV(PC0);
    DDRB |= _BV(PB3);
    DDRB |= _BV(PB4);
    DDRC |= _BV(PC7);

    // Don't shunt initially
    PORTD &= ~(_BV(PD0) | _BV(PD1));
    PORTC &= ~(_BV(PC0) | _BV(PC7));
    PORTB &= ~(_BV(PB3) | _BV(PB4));

    //LEDs
    DDRB |= _BV(PB0);
    DDRB |= _BV(PB1);
    DDRD |= _BV(PD7);
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
      timer1_counter = 0;
    }
}

void blink_on_init() {
    PORTB |= _BV(PB0);
    _delay_ms(500);
    PORTB &= ~_BV(PB0);
    PORTB |= _BV(PB1);
    _delay_ms(500);
    PORTB &= ~_BV(PB1);
    PORTD |= _BV(PD7);
    _delay_ms(500);
    PORTD &= ~_BV(PD7);
}

int main (void) {
    init_io_pins();
    init_millis();
    initADC();
    initCAN(NODE_bms);

    uint8_t ch; //Selects ADC Channel and PORTB write output
    uint8_t msg[3];

    blink_on_init();

    //Loop Begins
    for (;;) {
        if ((millis - capture_time) > CAPTURE_STEP) {
            capture_time += CAPTURE_STEP;
            // Let cells settle
            for (ch = 0; ch < 6; ch++) {
                if (ch == 0 || ch == 2) { PORTD &= ~outputs[ch];}
                if (ch == 1 || ch == 5) { PORTC &= ~outputs[ch];}
                if (ch == 3 || ch == 4) { PORTB &= ~outputs[ch];}
            }
            _delay_ms(10);

            //Check each of the 6 cells
            for (ch = 0; ch < 6; ch++) {

                uint16_t voltage = read_channel(ch);

                if (voltage >= MAXV){
                    shunt[ch] = 1;
                } else {
                    shunt[ch] = 0;
                }
            }
        }

        // Shunt if needed
        for (ch = 0; ch < 6; ch++) {
            if (shunt[ch] == 1) {
                if (ch == 0 || ch == 2) { PORTD |= outputs[ch];}
                if (ch == 1 || ch == 5) { PORTC |= outputs[ch];}
                if (ch == 3 || ch == 4) { PORTB |= outputs[ch];}
            } else {
                if (ch == 0 || ch == 2) { PORTD &= ~outputs[ch];}
                if (ch == 1 || ch == 5) { PORTC &= ~outputs[ch];}
                if (ch == 3 || ch == 4) { PORTB &= ~outputs[ch];}
            }
        }

        if ((millis - led_time) > led_step) {
            led_time = millis;
            if ((led_progress > led_index) | !shunt[led_index]) {
                PORTB &= ~_BV(PB0);
                led_index = (led_index + 1) % 6;
                led_progress = 0;
                led_step = LED_LONG_STEP;
            } else {
                if (!(PORTB & _BV(PB0))) {
                    PORTB |= _BV(PB0);
                } else {
                    PORTB &= ~_BV(PB0);
                    led_progress += 1;
                }
                led_step = LED_SHORT_STEP;
            }
        }
    }
    return 1;
}

void handleCANmsg(uint8_t destID, uint8_t msgID, uint8_t* msg, uint8_t msgLen) {
    PORTB ^= _BV(PB0);
}

