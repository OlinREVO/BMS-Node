#include <avr/io.h>

#define LED_LONG_STEP 500
#define LED_SHORT_STEP 150
uint32_t led_time = 0;
uint32_t led_step = 0;
uint8_t led_index = 0;
uint8_t led_progress = 0;

void ledUpdate(uint32_t millis, uint8_t *shunt) {
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
