#include <Arduino.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>

void InitADC(void);
uint16_t ReadADCSingleConversion(uint8_t channel);
void InitMotors(void);
void SetMotor(uint8_t motor, int16_t speed);
uint8_t breakPoint;
uint8_t speed;
uint8_t currSpeedM1;
uint8_t currSpeedM2;

void initInterrupt();
ISR(INT0_vect);
int readPoti();

void InitMotors(void) {

    // Pins D8 (PB0), D9 (PB1), D10 (PB2) und D11 (PB3) als Ausgänge setzen
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);

    // Timer1 für 8-Bit Fast PWM konfigurieren (Zählt von 0 bis 255)
    // COM1A1 / COM1B1: Setzt OC1A/OC1B beim Compare Match auf LOW (nicht invertierend)
    // WGM10: Teil 1 des 8-Bit Fast PWM Modus
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);

    // WGM12: Teil 2 des 8-Bit Fast PWM Modus
    // CS11 | CS10: Prescaler auf 64 setzen -> Frequenz: 16MHz / (64 * 256) = 976 Hz
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    // Motoren zum Start ausschalten (Tastverhältnis 0) 
    OCR1A = 0; 
    OCR1B = 0; 
}

void SetMotor(uint8_t motor, int16_t speed) {
    // Wertebereich limitieren
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    uint8_t pwm_val = 0;
    uint8_t dir_forward = 0;

    if (speed > 0) {
        pwm_val = speed;
        dir_forward = 1;
    } else if (speed < 0) {
        pwm_val = -speed;
        dir_forward = 0;
    } else {
        pwm_val = 0;
    }

    // Motor 1 (Physisch falsch herum verbaut)
    if (motor == 1) {
        // Bei Drehrichtungswechsel kurz warten
        if((currSpeedM1 < 0 && speed > 0) || (currSpeedM1 > 0 && speed < 0)) {
            SetMotor(1, 0);
            _delay_ms(100);
        }
        if (!dir_forward) {    
            PORTB |= (1 << PB0);  // PB0 sicher auf HIGH setzen
        } else {
            PORTB &= ~(1 << PB0); // PB0 sicher auf LOW setzen
        }
        OCR1A = pwm_val;
        currSpeedM1 = pwm_val;
        
    }
    // Motor 2 (M2_REV an PB3, M2_EN an OCR1B)
    else if (motor == 2) {
        // Bei Drehrichtungswechsel kurz warten
        if((currSpeedM2 < 0 && speed > 0) || (currSpeedM2 > 0 && speed < 0)) {
            SetMotor(2, 0);
            _delay_ms(100);
        }
        if (dir_forward) {
            PORTB |= (1 << PB3);  // Vorwärts
        } else {
            PORTB &= ~(1 << PB3); // Rückwärts
        }
        OCR1B = pwm_val;
        currSpeedM2 = pwm_val;

    }
}

uint16_t ReadADCSingleConversion(uint8_t channel) {
    // MUX-Bits löschen und gewünschten Kanal (0-7) wählen
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // ADSC: Messung starten
    ADCSRA |= (1 << ADSC);
    // Warten, bis das ADSC-Bit wieder 0 ist (Messung fertig)
    while (ADCSRA & (1 << ADSC));
    // Das 10-Bit Ergebnis (ADCL + ADCH) zurückgeben
    return ADC;
}

void initInterrupt() {
    // Korrekte Bitmaske für Eingänge
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    // Pull-Ups aktivieren
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);
    
    EICRA |= (1 << ISC01); 
    EIMSK |= (1 << INT0);  
    sei();
}

ISR(INT0_vect) {
    uint8_t left = ReadADCSingleConversion(0);
    uint8_t right = ReadADCSingleConversion(1);
    breakPoint = ((right+left)/2 -400);
    // Save breakPoint in eeprom
    eeprom_update_byte((uint8_t*)0, breakPoint);
}

int readPoti() {
    return ReadADCSingleConversion(7)/4;
}

int main(void) {
    
    InitADC();
    InitMotors();
    initInterrupt();
    bool lineFollower;
    // setup PD4 Pin for Input and Pullup
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);


    
    // Check if PD4 is HIGH
    if (PIND & (1 << PD4)) {
        lineFollower = false;
    } else {
        lineFollower = true;
    }


    breakPoint = eeprom_read_byte((uint8_t*)0);
    speed = readPoti();

    while(lineFollower) {
        uint8_t left = ReadADCSingleConversion(0);
        uint8_t right = ReadADCSingleConversion(1);
        if (left < breakPoint && right < breakPoint) {
            SetMotor(1, speed);
            SetMotor(0, speed);
        } else if (left < breakPoint && right > breakPoint) {
            SetMotor(1, speed/10);
            SetMotor(0, speed);
        } else {
            SetMotor(1, speed);
            SetMotor(0, speed/10);
        }
    }
    uint8_t error;
    uint8_t steering;

    #define MIN_OUTPUT 0
    #define MAX_OUTPUT 255
    #define PD 1.0

    while(!lineFollower){
        uint8_t left = ReadADCSingleConversion(0);
        error = left - breakPoint;
        steering = error * PD;
        

        if (steering < MIN_OUTPUT) {
        steering = MIN_OUTPUT;
        } else if (steering > MAX_OUTPUT) {
        steering = MAX_OUTPUT;
        }

        SetMotor(1, speed - steering);
        SetMotor(0, speed + steering);
        


    }
}
