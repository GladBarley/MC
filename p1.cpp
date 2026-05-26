#include <avr/io.h>         // Enthält die Hardware-Adressen (PORTB, DDRD etc.)
#include <stdint.h>         // Gibt uns saubere Datentypen wie uint8_t (exakt 8 Bit groß)
#include <avr/interrupt.h>  // Brauchen wir für ISR() und sei()
#include <util/delay.h>     // Für das _delay_ms() Makro

// ======================================================================================
// KONFIGURATION & VARIABLEN
// ======================================================================================

// volatile ist absolut Pflicht! Das sagt dem Compiler: "Achtung, diese Variable kann 
// sich jederzeit durch äußere Einflüsse (wie Taster oder Interrupts) ändern. 
// Speicher sie nicht weg, sondern lies sie jedes Mal frisch aus dem RAM!"
static volatile int delay_ms_val = 500; 

// --- BITMASKEN FÜR DIE TASTER ---
// (1 << PD1) schiebt eine '1' genau an die Stelle von Pin 1. 
// So basteln wir uns eine Maske, um später genau diesen einen Pin abzufragen.
const static uint8_t SWITCH0 = (1 << PD1); // Normaler Taster (Polling)
const static uint8_t SWITCH1 = (1 << PD2); // Taster für Geschwindigkeit
const static uint8_t SWITCH2 = (1 << PD3); // ACHTUNG: PD3 ist physikalisch der INT1-Pin! (Wichtig für den Interrupt)

// --- BITMASKEN FÜR DIE LEDs ---
// Die LEDs sind fies auf zwei verschiedene Hardware-Ports (D und B) aufgeteilt.
const static uint8_t LED0 = (1 << PD5);
const static uint8_t LED1 = (1 << PD6);
const static uint8_t LED2 = (1 << PD7);
const static uint8_t LED3 = (1 << PB0);
const static uint8_t LED4 = (1 << PB1);
const static uint8_t LED5 = (1 << PB2);
const static uint8_t LED6 = (1 << PB3);
const static uint8_t LED7 = (1 << PB4);

// Dieses Array ist dein "Übersetzer". 
// Du sagst später nur "Mach LED 4 an", und das Array weiß, dass LED 4 die Maske (1<<PB1) hat.
// ACHTUNG: Das ist absichtlich NICHT 'const', weil wir es im Interrupt verändern wollen!
static uint8_t ledArr[8] = {LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7};


// ======================================================================================
// HILFS-FUNKTIONEN
// ======================================================================================

// _delay_ms() schluckt nur feste Zahlen, keine Variablen. 
// Deshalb bauen wir uns eine for-Schleife, die einfach 1ms so oft wiederholt, 
// wie wir es in 'ms' fordern. Problem gelöst!
void mein_delay(int ms) {
    for(int i = 0; i < ms; i++) {
        _delay_ms(1);
    }
}

// Schaltet eine logische LED (0 bis 7) physikalisch AN
void switchOn(int led) {
    // Da LED 0, 1 und 2 am PORTD hängen und der Rest am PORTB, müssen wir hier unterscheiden.
    if (led < 3) PORTD |= ledArr[led];  // |= (OR) setzt das Bit auf 1, der Rest bleibt unverändert!
    else         PORTB |= ledArr[led];
}

// Schaltet eine logische LED (0 bis 7) physikalisch AUS
void switchOff(int led) {
    if (led < 3) PORTD &= ~ledArr[led]; // &= ~ (AND NOT) zwingt exakt dieses eine Bit auf 0!
    else         PORTB &= ~ledArr[led];
}


// ======================================================================================
// DIE LAUFLICHT-ANIMATION
// ======================================================================================
void onClick() {
    // Die äußere Schleife wiederholt die ganze Animation 3 Mal
    for (int i = 0; i < 3; i++) {
        
        // --- VORWÄRTS-LAUF ---
        // Das Fenster (j, j+1, j+2) schiebt sich von links nach rechts (0 bis 5)
        for (int j = 0; j < 6; j++) { 
            switchOn(j); switchOn(j+1); switchOn(j+2); // 3 LEDs anmachen
            mein_delay(delay_ms_val);                  // Warten (Geschwindigkeit!)
            switchOff(j); switchOff(j+1); switchOff(j+2);// Die 3 LEDs wieder ausmachen
        }
        
        // --- RÜCKWÄRTS-LAUF ---
        // Das Fenster schiebt sich wieder zurück (5 bis 0)
        for (int j = 5; j >= 0; j--) {
            switchOn(j); switchOn(j+1); switchOn(j+2);
            mein_delay(delay_ms_val);
            switchOff(j); switchOff(j+1); switchOff(j+2);
        }
        mein_delay(delay_ms_val); // Kurze Pause, bevor der nächste der 3 Durchläufe startet
    }
}


// ======================================================================================
// HAUPTPROGRAMM (MAIN)
// ======================================================================================
int main(void) {
    // --- 1. INTERRUPT HARDWARE EINRICHTEN ---
    // EIMSK (External Interrupt Mask Register): Schaltet den INT1 Kanal scharf.
    EIMSK |= (1 << INT1); 
    // EICRA (External Interrupt Control Register A): ISC11 auf 1 bedeutet "Falling Edge".
    // Der Interrupt löst genau in dem Moment aus, wo die Spannung von 5V auf 0V fällt (Taster wird gedrückt).
    EICRA |= (1 << ISC11); 
    // sei() (Set Enable Interrupts): Gibt dem Chip generell die Erlaubnis, Interrupts auszuführen.
    sei();

    // --- 2. PINS EINRICHTEN ---
    // Pull-Up-Widerstände für die Taster aktivieren. Die Pins ziehen sich intern auf 5V (HIGH).
    PORTD |= (SWITCH0 | SWITCH1 | SWITCH2);
    
    // DDR (Data Direction Register) auf 1 (Ausgang) setzen, damit die LEDs Strom bekommen.
    DDRD |= (LED0 | LED1 | LED2);
    DDRB |= (LED3 | LED4 | LED5 | LED6 | LED7);

    // --- 3. DIE ENDLOSSCHLEIFE ---
    while(1) {
        // POLLING: Wir fragen aktiv im Kreis ab, ob Pin PD1 auf Masse (0) gezogen wurde.
        if (!(PIND & SWITCH0)) {
            onClick(); // Wenn ja -> Starte die lange Lauflicht-Animation!
        }
        
        // Wenn Taster 2 gedrückt wird, wird die Variable auf 200ms geändert.
        // Das Lauflicht wird dadurch ab dem nächsten Schritt viel schneller!
        if (!(PIND & SWITCH1)) {
            delay_ms_val = 200;
        }
    }
}


// ======================================================================================
// DIE INTERRUPT SERVICE ROUTINE (Hardware-Notbremse)
// ======================================================================================
// Dieser Code reißt die CPU aus allem heraus, was sie gerade tut (sogar mitten aus onClick!),
// wenn der Taster an PD3 (INT1) gedrückt wird.
ISR(INT1_vect) {
    // Kurzes Warten, damit das mechanische Metall-Wackeln des Tasters (Prellen) aufhört.
    _delay_ms(5); 
    
    // Noch mal prüfen: Ist der Taster nach den 5ms wirklich noch gedrückt?
    if (!(PIND & SWITCH2)) {
        
        // 1. Die äußeren beiden LEDs physikalisch hart ausschalten
        switchOff(0);
        switchOff(7);
        
        // 2. DER GENIALE SOFTWARE-HACK!
        // Wir löschen die Bitmasken für LED0 und LED7 im Array und ersetzen sie durch 0.
        // Wenn das Programm später in onClick() versucht, switchOn(0) aufzurufen, 
        // rechnet der Chip: "PORTD |= 0". Das ändert rein gar nichts am PORTD!
        // Die LEDs sind damit für den Rest der Laufzeit "software-technisch abgeklemmt".
        ledArr[0] = 00000000;
        ledArr[7] = 00000000;
    }
}