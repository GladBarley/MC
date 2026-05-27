#include <avr/io.h>      // Hardware-Register für ATmega328P
#include <stdio.h>       // Für sprintf() (Text-Formatierung)
#include <Arduino.h>     // Für millis() und init()


// USART Initialisierung (Folie 40)
void InitUSART(uint32_t baud_rate) {
    uint16_t ubrr_val = (F_CPU - 8 * baud_rate) / (16 * baud_rate); 
    UBRR0 = ubrr_val;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Sender und Empfänger aktivieren
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 Bit Daten, 1 Stoppbit
}

// Einzelnes Byte senden (Folie 43)
void TransmitByte(uint8_t data) {
    // Warten, bis der Sendepuffer leer ist (Polling)
    do {} while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// C-String senden (Folie 45)
void TransmitString(const char* my_string) {
    uint16_t i = 0;
    while (my_string[i] != '\0') {
        TransmitByte(my_string[i]);
        i++;
    }
}

// ADC Initialisierung (Folie 89)
void InitADC(void) {
    ADMUX = (1 << REFS0); // Referenzspannung 5V (Vcc)
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128 (125 kHz)
    ADCSRA |= (1 << ADEN); // ADC aktivieren
    // Dummy-Messung zum Aufwärmen
    ADCSRA |= (1 << ADSC); 
    do {} while (ADCSRA & (1 << ADSC)); 
    uint16_t garbage = ADC; 
}

// ADC Messwert auslesen (Folie 90)
uint16_t ReadADCSingleConversion(uint8_t channel) {
    const uint8_t kMuxMask = ((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3));
    ADMUX &= ~kMuxMask; 
    ADMUX |= channel;   
    ADCSRA |= (1 << ADSC); 
    do {} while (ADCSRA & (1 << ADSC)); // Warten bis fertig
    return ADC; 
}

// ======================================================================================
// 2. HAUPTPROGRAMM (Deine Multitasking-Logik)
// ======================================================================================

int main(void) {
    init(); // Startet Timer-Service
    InitUSART(9600);
    InitADC(); 

    // --- PINS KONFIGURIEREN ---
    // Pin 8 (PB0) als Ausgang für Rechtecksignal
    DDRB |= (1 << DDB0);
    PORTB &= ~(1 << PORTB0); 

    // Pins 2-6 als Eingänge (Taster) mit internen Pull-Ups
    DDRD &= ~((1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6));
    PORTD |= (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5) | (1 << PORTD6);

    unsigned long letzter_tastendruck = 0; 
    unsigned long letzter_wechsel = 0;     
    unsigned long letzte_ausgabe = 0;      
    
    int modus = 5;               // 5 = Alles aus / Entladen
    unsigned long intervall = 0; // ms für eine Halbwelle
    char puffer[60];             

    while (1) { 
        unsigned long aktuelle_zeit = millis(); 


        if (aktuelle_zeit - letzter_tastendruck > 200) {
            int neuer_modus = 0;
            if ((PIND & (1 << PIND2)) == 0) neuer_modus = 1;
            if ((PIND & (1 << PIND3)) == 0) neuer_modus = 2;
            if ((PIND & (1 << PIND4)) == 0) neuer_modus = 3;
            if ((PIND & (1 << PIND5)) == 0) neuer_modus = 4;
            if ((PIND & (1 << PIND6)) == 0) neuer_modus = 5;

            if (neuer_modus != 0 && neuer_modus != modus) {
                modus = neuer_modus; 
                letzter_tastendruck = aktuelle_zeit; 
                letzter_wechsel = aktuelle_zeit; 
                
                // Intervalle für R=5k / C=220uF (Tau=1.1s)
                if (modus == 1) PORTB |= (1 << PORTB0);
                if (modus == 2) { intervall = 3450; PORTB |= (1 << PORTB0); }
                if (modus == 3) { intervall = 34500; PORTB |= (1 << PORTB0); }
                if (modus == 4) { intervall = 345; PORTB |= (1 << PORTB0); }
                if (modus == 5) PORTB &= ~(1 << PORTB0);
            }
        }

        if (modus == 2 || modus == 3 || modus == 4) {
            if (aktuelle_zeit - letzter_wechsel >= intervall) {
                // Pin toggeln
                PORTB ^= (1 << PORTB0); 
                letzter_wechsel = aktuelle_zeit; 
            }
        } 

        if (aktuelle_zeit - letzte_ausgabe >= 100) {
            // Prof-Funktion für ADC nutzen
            uint16_t adc_wert = ReadADCSingleConversion(0); 
            long spannung = ((long)adc_wert * 5000) / 1023; 
            
            // Text formatieren und via Prof-Funktion senden
            sprintf(puffer, "Zeit: %lu ms | Modus: %d | V: %ld mV\r\n", aktuelle_zeit, modus, spannung);
            TransmitString(puffer);
            
            letzte_ausgabe = aktuelle_zeit; 
        }
    } 
    return 0; 
}
