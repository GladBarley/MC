#define F_CPU 16000000UL 
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>

void InitUSART(uint32_t baud_rate) {
    uint16_t ubrr_val = (F_CPU - 8 * baud_rate) / (16 * baud_rate); 
    UBRR0 = ubrr_val;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void TransmitByte(uint8_t data) {
    do {} while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void TransmitString(const char* my_string) {
    uint16_t i = 0;
    while (my_string[i] != '\0') { TransmitByte(my_string[i]); i++; }
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

int16_t last_m1 = 0;
int16_t last_m2 = 0;

void setMotor(uint8_t motor, int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (motor == 1) {
        if ((last_m1 > 0 && speed < 0) || (last_m1 < 0 && speed > 0)) { OCR1A = 0; _delay_ms(50); }
        
        // --- SOFTWARE-INVERTIERUNG MOTOR 1 ---
        // Falls er falsch rum dreht, vertausche die Zeilen bei "speed >= 0" und "else"
        if (speed >= 0) { PORTB &= ~(1 << PORTB0); OCR1A = speed; } 
        else { PORTB |= (1 << PORTB0); OCR1A = -speed; }
        
        last_m1 = speed;
    } else {
        if ((last_m2 > 0 && speed < 0) || (last_m2 < 0 && speed > 0)) { OCR1B = 0; _delay_ms(50); }
        
        // --- SOFTWARE-INVERTIERUNG MOTOR 2 ---
        // Falls er falsch rum dreht, vertausche die Zeilen bei "speed >= 0" und "else"
        if (speed >= 0) { PORTB &= ~(1 << PORTB3); OCR1B = speed; }
        else { PORTB |= (1 << PORTB3); OCR1B = -speed; }
        
        last_m2 = speed;
    }
}


uint16_t EEMEM eeprom_threshold = 300; 

int main(void) {
    InitUSART(9600);
    InitADC();
    
    // PWM Konfig (Timer 1)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    DDRB |= (1 << DDB1) | (1 << DDB0) | (1 << DDB2) | (1 << DDB3);

    // Taster (PD2) und Jumper (PD4) als Eingang
    DDRD &= ~((1 << DDD4) | (1 << DDD2));
    PORTD |= (1 << PORTD4) | (1 << PORTD2); 

    uint16_t white_val = eeprom_read_word(&eeprom_threshold);

    while(1) {
        // A) KALIBRIERUNG
        if (!(PIND & (1 << PIND2))) {
            white_val = (ReadADCSingleConversion(0) + ReadADCSingleConversion(1)) / 2 + 150;
            eeprom_write_word(&eeprom_threshold, white_val);
            TransmitString("Kalibriert!\r\n");
            _delay_ms(500);
        }

        // B) SENSOREN & LOGIK
        uint16_t s1 = ReadADCSingleConversion(0);
        uint16_t s2 = ReadADCSingleConversion(1);
        int16_t base = ReadADCSingleConversion(7) / 4;

        // C) REGELUNG
        if (!(PIND & (1 << PIND4))) { // Jumper auf GND -> P-Regelung
            int16_t err = (int16_t)s1 - (int16_t)s2;
            setMotor(1, base - (err / 10)); 
            setMotor(2, base + (err / 10));
        } else { // Kein Jumper -> Bang-Bang
            // HINWEIS: Wenn er die Linie verliert, ändere hier die Vergleichsoperatoren < oder >
            // Schwarz = niedriger Wert, Weiß = hoher Wert (oder umgekehrt, je nach Sensor-Modul)
            if (s1 < white_val && s2 < white_val) { setMotor(1, base); setMotor(2, base); }
            else if (s1 < white_val) { setMotor(1, 0); setMotor(2, base); }
            else if (s2 < white_val) { setMotor(1, base); setMotor(2, 0); }
            else { setMotor(1, -base); setMotor(2, -base); }
        }
        _delay_ms(10);
    }
}