#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

// --- PIN-DEFINITIONEN (Exakt nach deiner Hardware) ---
#define LED_BLINK_PIN   PB5   // Onboard-LED an Pin D13 (Port B, Pin 5) -> Blinkt ab 2,5V
#define MOTOR_PIN       PB1   // Motor an Pin D9 (Port B, Pin 1) -> Hardware-PWM Kanal OC1A
#define P1_PWM_PIN      PD3   // LED P1 an Pin D3 (Port D, Pin 3) -> Hardware-PWM Kanal OC2B

// --- FUNKTIONSPROTOTYPEN ---
void InitUSART(uint32_t baud_rate);
void TransmitByte(uint8_t data);
void TransmitString(char* str);
void InitADC(void);
uint16_t ReadADCSingleConversion(uint8_t channel);
void InitTimer0_Blink(void);
void InitTimer1_Motor(void);
void InitTimer2_P1Fading(void);

// --- GLOBALE VARIABLEN ---
volatile uint8_t timer0_overflow_count = 0; // Zähler für die Überläufe von Timer0 (für das 500ms-Blinken)

int main(void) {
    // 1. Hardware-Komponenten initialisieren
    InitUSART(9600);       // Serielle Schnittstelle mit 9600 Baud starten
    InitADC();             // Analog-Digital-Wandler für das Potentiometer aktivieren
    InitTimer0_Blink();    // Timer0 für das Blinken der Onboard-LED (D13) vorbereiten
    InitTimer1_Motor();    // Timer1 für das exakte Signal des Motors (D9) vorbereiten
    InitTimer2_P1Fading(); // Timer2 für das kontinuierliche LED-Dimmen von P1 (D3) vorbereiten
    
    // 2. Globale Interrupts aktivieren (wichtig für die Timer0-ISR)
    sei();

    char buffer[128];      // Speicherplatz für die Textausgabe am PC
    uint16_t adc_val;      // Variable für den digitalen Messwert (0-1023)
    uint8_t angle_m1;      // Variable für den berechneten Winkel (nur für die Anzeige)
    
    // Variablen für das kontinuierliche Fading von LED P1 (Pin D3)
    uint8_t brightness = 0;     // Start-Helligkeit (0 = aus, 255 = voll an)
    int8_t fade_direction = 1;  // +1 bedeutet heller werden, -1 bedeutet dunkler werden
    
    // Software-Zähler zur zeitlichen Steuerung ohne delay_ms()-CPU-Blockierung
    uint16_t fade_counter = 0;
    uint16_t serial_print_counter = 0;

    while (1) {
        // SCHRITT A: Potentiometer an Pin A0 auslesen (Kanal 0)
        adc_val = ReadADCSingleConversion(0); 

        // SCHRITT B: Winkel für M1 per Dreisatz berechnen (nur für die serielle Textausgabe)
        angle_m1 = ((uint32_t)adc_val * 180) / 1023;

        // SCHRITT C: Motor-Ansteuerung an Pin D9 (OC1A) perfekt auf echte 180° kalibriert
        // Ein sicherer Startwert von 850 blockiert das wilde 360°-Drehen bei 0° komplett.
        // Der erhöhte Hub von 4000 Einheiten (850 + 4000 = 4850) drückt den Servo auf echte 180°.
        OCR1A = 850 + ((uint32_t)adc_val * 4000) / 1023;

        // SCHRITT D: Überwachung der 2,5V-Schwelle für die Onboard-LED (511 entspricht exakt 2,5V)
        if (adc_val > 511) {
            // Spannung > 2,5 V -> Timer0 einschalten mit Vorteiler/Prescaler 1024
            TCCR0B |= (1 << CS02) | (1 << CS00);
        } else {
            // Spannung <= 2,5 V -> Timer0 ausschalten (Taktbits löschen)
            TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
            PORTB &= ~(1 << LED_BLINK_PIN); // LED explizit ausschalten, falls sie an war
        }

        // SCHRITT E: Kontinuierliches Fading für LED P1 an Pin D3 (OC2B) aktualisieren
        fade_counter++;
        if (fade_counter >= 1500) {
            OCR2B = brightness; // PWM-Wert korrekt in das Register für Pin D3 schreiben
            brightness += fade_direction;
            
            // Richtung umkehren bei voll (255) oder aus (0)
            if (brightness == 255 || brightness == 0) {
                fade_direction = -fade_direction;
            }
            fade_counter = 0; // Zähler zurücksetzen
        }

        // SCHRITT F: Alle Werte alle ca. 500 ms an den Seriellen Monitor senden
        serial_print_counter++;
        if (serial_print_counter >= 8000) {
            // Zeigt Poti, den berechneten Winkel, den Motor-Registerwert UND die Helligkeit von P1 an
            sprintf(buffer, "Poti A0: %4u | Winkel M1: %3d Grad | Motor Reg: %4u | LED P1 PWM: %3u\r\n", 
                    adc_val, angle_m1, OCR1A, OCR2B);
            TransmitString(buffer);
            serial_print_counter = 0;
        }
    }
    return (0);
}

// =========================================================================
// --- TIMER-KONFIGURATIONEN & INTERRUPTS ---
// =========================================================================

// CONFIG TIMER0: Zuständig für das 1-Sekunden-Blinken der Onboard-LED an D13 (PB5)
void InitTimer0_Blink(void) {
    DDRB |= (1 << LED_BLINK_PIN);   // Pin D13 als Ausgang definieren
    TCCR0A = 0x00;                  // Normal Mode aktivieren
    TCCR0B = 0x00;                  // Bleibt beim Start gestoppt
    TIMSK0 |= (1 << TOIE0);         // Interrupt bei Register-Überlauf aktivieren
}

// INTERRUPT SERVICE ROUTINE (ISR) für den Timer0 Überlauf
ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
    if (timer0_overflow_count >= 31) { 
        PORTB ^= (1 << LED_BLINK_PIN);  // LED an/aus toggeln
        timer0_overflow_count = 0;      // Zähler zurücksetzen
    }
}

// CONFIG TIMER1: Erzeugt ein stabiles 50Hz Signal für den Motor an Pin D9 (PB1 / OC1A)
void InitTimer1_Motor(void) {
    DDRB |= (1 << MOTOR_PIN);       // Pin D9 als Ausgang definieren
    
    // Modus 14 aktivieren: Fast PWM mit dem Register ICR1 als Maximalwert (TOP)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // CS11 setzt den Prescaler auf 8
    
    // TOP-Wert auf 40000 setzen:
    // 16.000.000 Hz / Prescaler 8 / 50 Hz Ziel-Frequenz = Genau 40.000 Ticks pro Periode (20 ms)
    ICR1 = 40000; 
    OCR1A = 1500;                   // Startwert (Mittelstellung)
}

// CONFIG TIMER2: Hardware-PWM für das flüssige Fading von LED P1 an Pin D3 (PD3 / OC2B)
void InitTimer2_P1Fading(void) {
    DDRD |= (1 << P1_PWM_PIN);      // Pin D3 als Ausgang definieren
    
    // Fast-PWM Modus (8-Bit) einstellen (WGM21 und WGM20)
    // COM2B1 setzt den Ausgang OC2B (Pin D3) in den nicht-invertierenden Modus
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS21);           // Prescaler = 8 für eine flimmerfreie PWM-Frequenz der LED
    OCR2B = 0;                      // Start-Helligkeit auf 0 setzen (aus)
}


// =========================================================================
// --- BASIS-FUNKTIONEN (Aus den alten Modulen) ---
// =========================================================================

// FUNKTION: Initialisiert den Analog-Digital-Wandler (ADC)
void InitADC(void) {
    // ADMUX: ADC Multiplexer Selection Register
    // (1 << REFS0): Setzt die Referenzspannung des ADC auf AVCC (die 5V Versorgungsspannung des Arduino).
    ADMUX = (1 << REFS0); 
    
    // ADCSRA: ADC Control and Status Register A
    // (1 << ADEN): ADC Enable – Schaltet das komplette ADC-Modul der Hardware ein.
    // (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0): Setzt den Frequenzteiler (Prescaler) auf 128.
    // Rechnung: 16.000.000 Hz (CPU-Takt) / 128 = 125.000 Hz (Optimaler Wandlertakt für präzise Ergebnisse).
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
}

// FUNKTION: Startet eine einzelne Analog-Digital-Wandlung und gibt das 10-Bit Ergebnis zurück
uint16_t ReadADCSingleConversion(uint8_t channel) {
    // ADMUX & 0xF0: Löscht die alten Kanalbits (untere 4 Bits), behält aber die Referenzspannung (obere Bits).
    // channel & 0x0F: Setzt die Bits für den gewünschten Eingangskanal (hier Kanal 0 für Pin A0).
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); 
    
    // ADCSRA |= (1 << ADSC): ADC Start Conversion – Zündet die Hardware an, um JETZT die Messung zu starten.
    ADCSRA |= (1 << ADSC);                     
    
    // while-Schleife: Wartet, solange das Bit ADSC noch auf 1 steht.
    // Sobald der Mikrocontroller die Spannung fertig gemessen hat, setzt er das ADSC-Bit automatisch auf 0 zurück.
    while (ADCSRA & (1 << ADSC));              
    
    // ADC: Das 16-Bit-Ergebnisregister (bestehend aus ADCL und ADCH). 
    // Es liefert einen digitalen Ganzzahlwert zwischen 0 (bei 0V) und 1023 (bei 5V) zurück.
    return ADC;                                
}

// FUNKTION: Initialisiert die serielle USART-Kommunikation zum Computer
void InitUSART(uint32_t baud_rate) {
    // ubrr_val: Berechnet den Wert für das Baudraten-Register aus der CPU-Frequenz und der Wunsch-Baudrate.
    // Formel laut Datenblatt für den asynchronen Normalmodus.
    uint16_t ubrr_val = (F_CPU - 8 * baud_rate) / (16 * baud_rate);
    
    // UBRR0: USART Baud Rate Register
    // Schreibt den berechneten Teilerwert in das Hardware-Register, um exakt 9600 Baud zu treffen.
    UBRR0 = ubrr_val;
    
    // UCSR0B: USART Control and Status Register 0 B
    // (1 << TXEN0): Transmitter Enable – Schaltet den Hardware-Sende-Pin (TX) des Arduino frei.
    UCSR0B = (1 << TXEN0);                  
    
    // UCSR0C: USART Control and Status Register 0 C
    // (1 << UCSZ01) | (1 << UCSZ00): Setzt die Zeichengröße (Character Size) auf 8 Datenbits.
    // Standardmäßig ist damit das klassische Format 8N1 (8 Datenbits, keine Parität, 1 Stoppbit) aktiv.
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

// FUNKTION: Sendet ein einzelnes Textzeichen (Byte) über die Leitung zum PC
void TransmitByte(uint8_t data) {
    // UCSR0A: USART Control and Status Register 0 A
    // UDRE0: USART Data Register Empty – Dieses Bit wird von der Hardware 1, wenn der interne Sendepuffer leer ist.
    // Die Schleife blockiert solange, bis das alte Zeichen fertig gesendet wurde und Platz für das neue Zeichen ist.
    while (!(UCSR0A & (1 << UDRE0)));       
    
    // UDR0: USART I/O Data Register 0
    // Schreibt das zu sendende Byte direkt in den Hardware-Speicher. Der Chip schickt es danach sofort los.
    UDR0 = data;                            
}

// FUNKTION: Sendet eine komplette Zeichenkette (Text/String) Buchstabe für Buchstabe
void TransmitString(char* str) {
    // Die Schleife läuft so lange, bis der Zeiger auf das String-Endezeichen '\0' (Null-Terminator) stößt.
    while (*str) {
        // *str++: Sendet das aktuelle Zeichen per TransmitByte ab und springt danach sofort zum nächsten Buchstaben im Text.
        TransmitByte(*str++);               
    }
}
