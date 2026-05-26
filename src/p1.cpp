#include<avr/io.h>
#include<stdint.h>
#include<avr/interrupt.h>
#include<util/delay.h>
/*_________*/

const static uint8_t SWITCH0=(1<<PD1);
const static uint8_t SWITCH1=(1<<PD2);
const static uint8_t SWITCH2=(1<<PD3);

const static uint8_t LED0=(1<<PD5);
const static uint8_t LED1=(1<<PD6);
const static uint8_t LED2=(1<<PD7);

const static uint8_t LED3=(1<<PB0);
const static uint8_t LED4=(1<<PB1);
const static uint8_t LED5=(1<<PB2);
const static uint8_t LED6=(1<<PB3);
const static uint8_t LED7=(1<<PB4);

static uint8_t ledArr[8] = {LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7};
static int delay = 500;

int switchOn(int led) {
    if (led < 3) {
        PORTD ^= ledArr[led];
    } else {
        PORTB ^= ledArr[led];
    }
}

int switchOff(uint8_t led) {
    if (led < 3) {
        PORTD &= ~ledArr[led];
    } else {
        PORTB &= ~ledArr[led];
    }
}

int onClick() {
    for (int i = 0; i < 3; i++) {
        for (int i = 0; i < 5; i++){
            switchOn(i);
            switchOn(i+1);
            switchOn(i+2);
            _delay_ms(delay);
            switchOff(i);
            switchOff(i+1);
            switchOff(i+2);
        }
        for (int i = 4; i >= 0; i--){
            switchOn(i);
            switchOn(i+1);
            switchOn(i+2);
            _delay_ms(delay);
            switchOff(i);
            switchOff(i+1);
            switchOff(i + 2);
        }
        _delay_ms(delay);
    }
}

int main(void) {
/* interrupt setup */
EIMSK |= (1<<INT1); // enable interrupt 
EICRA |= (1<<ISC11); // falling edge 
sei();

/* led / switch setup */
PORTD|=(SWITCH0|SWITCH1|SWITCH2);
PORTB=0;
DDRD|=(LED0|LED1|LED2);
DDRB|=(LED3|LED4|LED5|LED6|LED7);

while(1) {
    if (SWITCH0 == 0) {
        onClick();
    }
    if (SWITCH1 == 0) {
        delay = 200;
    }
}
}

/* interrupt logic */
ISR(INT1_vect) {
    _delay_ms(5);
    if (SWITCH2 == 0) {
        switchOff(0);
        switchOff(7);
        ledArr[0] = 00000000; // 0000 0000 xor 1010 1010 == 1010 1010
        ledArr[7] = 00000000;
    }
}
