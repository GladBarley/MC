// p4.cpp
#include "p4.h" // Deinen eigenen Header einbinden
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/eeprom.h>

// Globale Variablen
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
    // PB0-PB3 als Ausgang konfigurieren (zuständig für PWM-Signale und Drehrichtung)
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3); 
    
    // Timer 1 (16-Bit) für Fast PWM (8-Bit) konfigurieren
    // COM1A1/COM1B1: Non-inverting Mode (schaltet den Pin beim Erreichen des Vergleichswerts ab)
    // WGM10: Teil der Fast PWM (8-Bit) Konfiguration
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    
    // WGM12: Vervollständigt die Fast PWM Konfiguration
    // CS11 & CS10: Prescaler (Taktteiler) auf 64 setzen
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    
    // Output Compare Registers (Pulsweite/Duty-Cycle) initial auf 0 setzen
    OCR1A = 0; 
    OCR1B = 0; 
}

void SetMotor(uint8_t motor, int16_t set_speed) {
    // Geschwindigkeit auf erlaubten 8-Bit PWM-Bereich begrenzen
    if (set_speed > 255) set_speed = 255;
    if (set_speed < -255) set_speed = -255;

    uint8_t pwm_val = 0;
    uint8_t dir_forward = 0;

    // Betrag und Richtung ermitteln
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
        // Kurze Pause bei Richtungswechsel, um die H-Brücke/Motoren zu schonen
        if((currSpeedM1 < 0 && set_speed > 0) || (currSpeedM1 > 0 && set_speed < 0)) {
            SetMotor(1, 0);
            _delay_ms(100);
        }
        
        // Drehrichtung über Port B steuern
        if (!dir_forward) {    
            PORTB |= (1 << PB0);  // Pin High
        } else {
            PORTB &= ~(1 << PB0); // Pin Low
        }
        
        // PWM-Wert für Motor 1 (Timer 1, Kanal A) aktualisieren
        OCR1A = pwm_val;
        currSpeedM1 = pwm_val;
    }
    else if (motor == 2) { // (Eigentlich Motor 0 in deinem main-Aufruf, aber hier 2 genannt)
        if((currSpeedM2 < 0 && set_speed > 0) || (currSpeedM2 > 0 && set_speed < 0)) {
            SetMotor(2, 0);
            _delay_ms(100);
        }
        
        // Drehrichtung über Port B steuern
        if (dir_forward) {
            PORTB |= (1 << PB3);  // Pin High
        } else {
            PORTB &= ~(1 << PB3); // Pin Low
        }
        
        // PWM-Wert für Motor 2 (Timer 1, Kanal B) aktualisieren
        OCR1B = pwm_val;
        currSpeedM2 = pwm_val;
    }
}

uint16_t ReadADCSingleConversion(uint8_t channel) {
    // ADMUX: Obere 4 Bits (Referenzspannung) beibehalten, untere 4 Bits (Kanal) neu setzen
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); 
    
    // ADCSRA: ADC Start Conversion (ADSC) Bit setzen, um die Messung zu starten
    ADCSRA |= (1 << ADSC);
    
    // Warten, bis die Hardware das ADSC-Bit wieder löscht (Messung abgeschlossen)
    while (ADCSRA & (1 << ADSC));
    
    // Gibt das fertige 10-Bit Ergebnis aus den ADC-Datenregistern (ADCL + ADCH) zurück
    return ADC;
}

void initInterrupt() {
    // PD2, PD4 und PD5 als Eingänge konfigurieren (Data Direction Register)
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    // Interne Pull-Up-Widerstände für diese Pins aktivieren
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);
    
    // EICRA (External Interrupt Control Register A): INT0 so einstellen, dass 
    // er bei einer fallenden Flanke (High zu Low) auslöst (ISC01=1, ISC00=0)
    EICRA |= (1 << ISC01); 
    
    // Funktionszeiger für den Interrupt zuweisen (spezifisch für deine Architektur)
    current_interrupt_action = p4_interrupt_logic;
    
    // EIMSK (External Interrupt Mask Register): Hardware-Interrupt INT0 freigeben
    EIMSK |= (1 << INT0);  
    
    // Globale Interrupts aktivieren (Set Enable Interrupts)
    sei();
}

void p4_interrupt_logic() {
    // ADC für linke und rechte Sensoren auslesen
    uint8_t left = ReadADCSingleConversion(0);
    uint8_t right = ReadADCSingleConversion(1);
    
    // Schwellenwert berechnen
    breakPoint = ((right+left)/2 - 400);
    
    // Berechneten Schwellenwert in den EEPROM schreiben (Adresse 0), um ihn spannungsausfallsicher zu speichern
    eeprom_update_byte((uint8_t*)0, breakPoint);
}


int readPoti() {
    // Kanal 7 messen und auf 8-Bit skalieren (10-Bit ADC / 4 = 8-Bit)
    return ReadADCSingleConversion(7)/4;
}

// AUS int main(void) WIRD JETZT runProgram4()
void runProgram4() {
    // InitADC(); // Auskommentiert, da die Funktion in deiner Vorlage nicht definiert ist
    InitMotors();
    initInterrupt();
    bool lineFollower;
    
    // Taster/Schalter-Pins als Eingang und Pull-Ups aktivieren (Redundant zu initInterrupt, aber sicher)
    DDRD &= ~((1 << PD2) | (1 << PD4) | (1 << PD5));
    PORTD |= (1 << PD2) | (1 << PD4) | (1 << PD5);

    // PIND liest den aktuellen Status der Eingangspins ein (hier Pin PD4)
    if (PIND & (1 << PD4)) {
        lineFollower = false;
    } else {
        lineFollower = true;
    }

    // Gespeicherten Schwellenwert aus dem EEPROM laden
    breakPoint = eeprom_read_byte((uint8_t*)0);
    speed = readPoti();

    while(lineFollower) {
        uint8_t left = ReadADCSingleConversion(0);
        uint8_t right = ReadADCSingleConversion(1);
        
        // Einfache Bang-Bang Steuerung für die Linie
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

    // PD-Regler Logik (aktuell reiner P-Regler, da D-Anteil fehlt)
    while(!lineFollower){
        uint8_t left = ReadADCSingleConversion(0);
        
        error = left - breakPoint;
        steering = error * PD;
        
        // Ausgang begrenzen
        if (steering < MIN_OUTPUT) {
            steering = MIN_OUTPUT;
        } else if (steering > MAX_OUTPUT) {
            steering = MAX_OUTPUT;
        }

        // Lenken durch Anpassen der Motorgeschwindigkeiten
        SetMotor(1, speed - steering);
        SetMotor(0, speed + steering);
    }
}