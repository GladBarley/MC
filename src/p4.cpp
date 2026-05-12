// p4.cpp
#include "p4.h" // Deinen eigenen Header einbinden
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>

// Globale Variablen (static, damit sie nur in dieser Datei gelten)
static uint8_t breakPoint;
static uint8_t speed;
static uint8_t currSpeedM1;
static uint8_t currSpeedM2;

// Funktionsprototypen
uint16_t ReadADCSingleConversion(uint8_t channel);
void InitMotors(void);
void SetMotor(uint8_t motor, int16_t speed);
void initInterrupt();
int readPoti();
extern void (*current_interrupt_action)(); 




void InitMotors(void) {
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3);
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 0; 
    OCR1B = 0; 
}

void SetMotor(uint8_t motor, int16_t set_speed) {
    if (set_speed > 255) set_speed = 255;
    if (set_speed < -255) set_speed = -255;

    uint8_t pwm_val = 0;
    uint8_t dir_forward = 0;

    if (set_speed > 0) {
        pwm_val = set_speed;
        dir_forward = 1;
    } else if (set_speed < 0) {
        pwm_val = -set_speed;
        dir_forward = 0;
    } else {
        pwm_val = 0;
    }

    if (motor == 1) {
        if((currSpeedM1 < 0 && set_speed > 0) || (currSpeedM1 > 0 && set_speed < 0)) {
            SetMotor(1, 0);
            _delay_ms(100);
        }
        if (!dir_forward) {    
            PORTB |= (1 << PB0); 
        } else {
            PORTB &= ~(1 << PB0);
        }
        OCR1A = pwm_val;
        currSpeedM1 = pwm_val;
    }
    else if (motor == 2) { // (Eigentlich Motor 0 in deinem main-Aufruf, aber hier 2 genannt)
        if((currSpeedM2 < 0 && set_speed > 0) || (currSpeedM2 > 0 && set_speed < 0)) {
            SetMotor(2, 0);
            _delay_ms(100);
        }
        if (dir_forward) {
            PORTB |= (1 << PB3); 
        } else {
            PORTB &= ~(1 << PB3);
        }
        OCR1B = pwm_val;
        currSpeedM2 = pwm_val;
    }
}

uint16_t ReadADCSingleConversion(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void initInterrupt() {
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);
    EICRA |= (1 << ISC01); 
    
    current_interrupt_action = p4_interrupt_logic;
    
    EIMSK |= (1 << INT0);  
    sei();
}

void p4_interrupt_logic() {
    uint8_t left = ReadADCSingleConversion(0);
    uint8_t right = ReadADCSingleConversion(1);
    breakPoint = ((right+left)/2 - 400);
    eeprom_update_byte((uint8_t*)0, breakPoint);
}


int readPoti() {
    return ReadADCSingleConversion(7)/4;
}

// AUS int main(void) WIRD JETZT runProgram4()
void runProgram4() {
    // InitADC(); // Auskommentiert, da die Funktion in deiner Vorlage nicht definiert ist
    InitMotors();
    initInterrupt();
    bool lineFollower;
    
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);

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