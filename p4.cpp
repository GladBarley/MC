#include <avr/io.h>         // Enthält die Hardware-Adressen (PORTB, DDRD etc.)
#include <stdio.h>       
#include <stdint.h>         // Gibt uns saubere Datentypen wie uint8_t (exakt 8 Bit groß)
#include <avr/eeprom.h>
#include <avr/interrupt.h>  // Brauchen wir für ISR() und sei()
#include <util/delay.h>     // Für das _delay_ms() Makro

// USART Initialisierung (Folie 40)
void InitUSART(uint32_t baud_rate) {
    uint16_t ubrr_val = (F_CPU - 8 * baud_rate) / (16 * baud_rate); 
    UBRR0 = ubrr_val;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);   
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

// Einzelnes Byte senden
void TransmitByte(uint8_t data) {
    do {} while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// C-String senden
void TransmitString(const char* my_string) {
    uint16_t i = 0;
    while (my_string[i] != '\0') {
        TransmitByte(my_string[i]);
        i++;
    }
}

int16_t lastSpeed1 = 1;
int16_t lastSpeed2 = 1;

bool dirChange(int16_t speed1, int16_t speed2) {
  if (speed1 > 0 && lastSpeed1 < 0){
    return 1;
  }
  if (speed1 < 0 && lastSpeed1 > 0){
    return 1;
  }

  if (speed2 > 0 && lastSpeed2 < 0){
    return 1;
  }
  if (speed2 < 0 && lastSpeed2 > 0){
    return 1;
  }

  return 0;
}

void setBothMotors(int16_t speed1, int16_t speed2) {
  //dir flip
  if(1) {
    speed1 = -speed1;
    speed2 = -speed2;
  }

  if (speed1 > 255) speed1 = 255; if (speed1 < -255) speed1 = -255;
  if (speed2 > 255) speed2 = 255; if (speed2 < -255) speed2 = -255;
  

  // char lul[60];             
  // sprintf(lul, "speed1: %d, ", speed1);
  // TransmitString(lul);
  if (speed1 >= 0) { PORTB &= ~(1 << PORTB0); OCR1A = speed1; } else { PORTB |= (1 << PORTB0); OCR1A = -speed1; }
  
  // sprintf(lul, "speed2: %d, ", speed2);
  // TransmitString(lul);
  if (speed2 >= 0) { PORTB &= ~(1 << PORTB3); OCR1B = speed2; } else { PORTB |= (1 << PORTB3); OCR1B = -speed2; }
}

void init() {
  // PWM Konfig (Timer 1)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
  DDRB |= (1 << DDB1) | (1 << DDB0) | (1 << DDB2) | (1 << DDB3);

  // Taster (PD2) und Jumper (PD4) als Eingang
  DDRD &= ~((1 << DDD4) | (1 << DDD2));
  PORTD |= (1 << PORTD4) | (1 << PORTD2); 
}

void InitADC(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
    do {} while (ADCSRA & (1 << ADSC));
    uint16_t garbage = ADC; 
}

uint16_t ReadADCSingleConversion(uint8_t channel) {
    const uint8_t kMuxMask = ((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
    ADMUX &= ~kMuxMask; 
    ADMUX |= channel;   
    ADCSRA |= (1 << ADSC);
    do {} while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint16_t EEMEM eeprom_threshold = 300; 

int main() {
  InitUSART(9600);
  InitADC();
  init();
  char ret[60];
  uint16_t lightVal = eeprom_read_word(&eeprom_threshold);
  float kp = 0.0045;

  while(1) {
      // Bang Bang
      // TransmitString("BangBang\n");
      if (!(PIND & (1 << PIND2))) {
          lightVal = (ReadADCSingleConversion(0) + ReadADCSingleConversion(1)) / 2 + 150;
          eeprom_write_word(&eeprom_threshold, lightVal);
      }

      uint16_t adc1 = ReadADCSingleConversion(0);
      uint16_t adc2 = ReadADCSingleConversion(1);

    // slow speed at high motor stettings
    OCR1A = 0;
    OCR1B = 0;

    _delay_ms(25); // pause time

      uint16_t Poti = ReadADCSingleConversion(7);

      int m1val = (35 + (Poti / 1023.0) * 40);
      int m2val = (50 + (Poti / 1023.0) * 60);

    if (!(PIND & 1 << PD4)) {
      if (adc1 >= lightVal || adc2 >= lightVal) {
        int16_t error = adc1 - adc2; 
        int16_t steering = error * kp;
        setBothMotors(m1val+steering, m2val-steering);
      } else {
        // white white case
        setBothMotors(-m1val/1.25, -m2val/1.25);
      }
    } else {
      if (adc1 >= lightVal && adc2 >= lightVal) {
        // black black case
        setBothMotors(m1val, m2val);
      } else if (adc1 <= lightVal && adc2 >= lightVal) {
        // white black case
        setBothMotors(m1val, 0);
      } else if(adc1 >= lightVal && adc2 <= lightVal){
        // black white case
        setBothMotors(0, m2val);
      } else {
        // white white case
        setBothMotors(-m1val/1.25, -m2val/1.25);
      }

    }

    _delay_ms(10); // movement time

    // sprintf(ret, "adc1: %d, adc2: %d\n", adc1, adc2);
    sprintf(ret, "Poti: %d, m1: %d, m2 %d\n", Poti, m1val, m2val);
    TransmitString(ret);
  }
}
