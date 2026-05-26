#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdbool.h> // Hinzugefügt für Datentyp bool

// Globale Variablen (Typen korrigiert für 10-Bit ADC und Vorzeichen)
static uint16_t breakPoint; 
static uint8_t speed;
static int16_t currSpeedM1; 
static int16_t currSpeedM2;

#define MIN_OUTPUT 0
#define MAX_OUTPUT 255
#define PD 0.05

// Funktionsprototypen
uint16_t ReadADCSingleConversion(uint8_t channel);
void InitMotors(void);
void SetMotor(uint8_t motor, int16_t speed);
void initInterrupt(void);
int readPoti(void);

void InitMotors(void) {
    // PB0-PB3 als Ausgang für PWM und Drehrichtung
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); 
    
    // Timer 1: 8-Bit Fast PWM, Non-inverting (Pin geht auf LOW bei Match)
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    // Prescaler auf 64
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    
    // Initiale Pulsweite auf 0 (Motor aus)
    OCR1A = 0; 
    OCR1B = 0; 
}

void SetMotor(uint8_t motor, int16_t set_speed) {
    // Geschwindigkeit auf zulässigen 8-Bit PWM-Bereich limitieren
    if (set_speed > 255) set_speed = 255;
    if (set_speed < -255) set_speed = -255;

    uint8_t pwm_val = 0;
    uint8_t dir_forward = 0;

    // Betrag (PWM) und Richtungszustand extrahieren
    if (set_speed > 0) {
        pwm_val = set_speed;
        dir_forward = 1;
    } else if (set_speed < 0) {
        pwm_val = -set_speed;
        dir_forward = 0;
    }

    if (motor == 1) {
        // H-Brücke schonen: Totzeit bei abruptem Richtungswechsel
        if((currSpeedM1 < 0 && set_speed > 0) || (currSpeedM1 > 0 && set_speed < 0)) {
            SetMotor(1, 0);
            _delay_ms(100);
        }
        
        // Drehrichtung über Pin PB0 setzen
        if (!dir_forward) {    
            PORTB |= (1 << PB0); 
        } else {
            PORTB &= ~(1 << PB0);
        }
        
        // PWM-Duty-Cycle aktualisieren
        OCR1A = pwm_val;
        currSpeedM1 = set_speed; // Korrigiert: Speichere den echten Wert inkl. Vorzeichen
    }
    else if (motor == 0) { // Korrigiert: In main() wird Motor 0 aufgerufen, nicht 2
        // H-Brücke schonen: Totzeit
        if((currSpeedM2 < 0 && set_speed > 0) || (currSpeedM2 > 0 && set_speed < 0)) {
            SetMotor(0, 0);
            _delay_ms(100);
        }
        
        // Drehrichtung über Pin PB3 setzen
        if (dir_forward) {
            PORTB |= (1 << PB3);
        } else {
            PORTB &= ~(1 << PB3);
        }
        
        OCR1B = pwm_val;
        currSpeedM2 = set_speed; // Korrigiert: Speichere den echten Wert inkl. Vorzeichen
    }
}

uint16_t ReadADCSingleConversion(uint8_t channel) {
    // Gewünschten ADC-Kanal wählen, Referenzspannungs-Konfiguration beibehalten
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); 
    
    // Messung starten
    ADCSRA |= (1 << ADSC);
    
    // Warten, bis das Hardware-Bit gelöscht wird (Messung fertig)
    while (ADCSRA & (1 << ADSC));
    
    // 10-Bit ADC-Wert zurückgeben
    return ADC;
}

void initInterrupt() {
    // PD2 (INT0), PD4 und PD5 als Eingänge setzen und Pull-Ups aktivieren
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);
    
    // INT0 auf fallende Flanke konfigurieren
    EICRA |= (1 << ISC01); 
    EIMSK |= (1 << INT0);  
    
    // Globale Interrupts freigeben
    sei();
}

// Interrupt-Service-Routine für den externen Interrupt 0 (Kalibrierung)
ISR(INT0_vect) {
    uint16_t left = ReadADCSingleConversion(0);
    uint16_t right = ReadADCSingleConversion(1);
    
    // Mittelwert abzüglich Offset als Schwellenwert berechnen
    breakPoint = ((right + left) / 2) - 400;
    
    // Als 16-Bit-Wert (Word) netzausfallsicher im EEPROM speichern
    eeprom_update_word((uint16_t*)0, breakPoint);
}

int readPoti() {
    // ADC-Wert (0-1023) durch 4 teilen, um auf 8-Bit (0-255) für PWM zu skalieren
    return ReadADCSingleConversion(7) / 4;
}

int main(void) {
    InitMotors();
    initInterrupt();
    bool lineFollower;
    
    // Betriebsmodus über Pin PD4 einlesen (LOW-aktiv durch Pull-Up)
    if (PIND & (1 << PD4)) {
        lineFollower = false;
    } else {
        lineFollower = true;
    }

    // Gespeicherten Schwellenwert auslösen und Grundgeschwindigkeit setzen
    breakPoint = eeprom_read_word((uint16_t*)0);
    speed = readPoti();

    // Modus 1: Bang-Bang-Regelung (Zweipunktregler)
    while(lineFollower) {
        // Korrigiert: Datentyp uint16_t für 10-Bit ADC
        uint16_t left = ReadADCSingleConversion(0);
        uint16_t right = ReadADCSingleConversion(1);
        
        if (left < breakPoint && right < breakPoint) {
            // Auf der Linie: Beide Motoren Vollgas
            SetMotor(1, speed);
            SetMotor(0, speed);
        } else if (left < breakPoint && right > breakPoint) {
            // Linie rechts verlassen: Links drosseln
            SetMotor(1, speed/10);
            SetMotor(0, speed);
        } else {
            // Linie links verlassen: Rechts drosseln
            SetMotor(1, speed);
            SetMotor(0, speed/10);
        }
    }
    
    // Variablen für den P-Regler (signed, da Fehler/Lenkung negativ sein können)
    int16_t error;
    int16_t steering;

    // Modus 2: Proportional-Regelung
    while(!lineFollower){
        uint16_t left = ReadADCSingleConversion(0);
        
        // Regelabweichung (Error) berechnen
        error = left - breakPoint;
        
        // Stellgröße berechnen (P-Anteil)
        steering = error * PD; 
        // alternativ steering = (error * 5) / 100 für bessere performance
        
        // Lenkeinschlag begrenzen
        if (steering < MIN_OUTPUT) {
            steering = MIN_OUTPUT;
        } else if (steering > MAX_OUTPUT) {
            steering = MAX_OUTPUT;
        }

        // Lenkmanöver auf Motoren verteilen (Differenzsteuerung)
        SetMotor(1, speed - steering);
        SetMotor(0, speed + steering);
    }
    return 0;
}