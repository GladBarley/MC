#ifndef F_CPU
#define F_CPU 16000000UL 
#endif

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

// --- Pin- und Modus-Definitionen ---
#define MODE_PIN  4      
#define CALIB_PIN 5      

const int8_t P = 0;
const int8_t BangBang = 1;

// --- Enums und globale Zustände ---
enum Motor { M1 = 0, M2 = 1 };
int8_t current_dir[2] = {0, 0}; // Speichert die aktuelle Richtung der Motoren (0=Stop, 1=Vorwärts, 2=Rückwärts)

const int16_t PWM_FREQ = 15999;
const int16_t kBaud = 9600;
const int8_t sensor_left = PC0;    
const int8_t sensor_right = PC1;
const int8_t POT1 = 7;
volatile float Kp = 0.0038;       // *10^-2
uint16_t threshold = 600;
int8_t rMode = 0;

uint16_t EEMEM eeprom_threshold;
float EEMEM eeprom_kp;

// --- Funktionsprototypen (Vorwärtsdeklarationen) ---
void initTimerPWM(void);
void stop_motors(void);
void InitADC(uint8_t channel);
uint16_t ReadADCSingleConversion(uint8_t channel);
uint8_t getSensor(int16_t value, int16_t threshold, int16_t accuracy);
float pMath(int16_t sensor1, int16_t sensor2, float Kp);
void pController(uint16_t base_speed);
void calibrate_safe_threshold_kp(void); // KORRIGIERT: Namenskonflikt behoben
void flash_led(uint8_t times);          // KORRIGIERT: Fehlende Deklaration ergänzt
void save_kp(float value);
float load_kp(void);
void save_threshold(uint16_t value);
uint16_t load_threshold(void);
void calibrate_safe_threshold(void);
uint16_t calibrate_threshold(void);
void bangBang(uint16_t speed);
void set_Motor(enum Motor i_Motor, uint16_t i_Motor_Speed);
void forward(enum Motor i_Motor);
void backward(enum Motor i_Motor);
void run_forward(uint16_t speed);
void run_backward(uint16_t speed);
void left(uint16_t speed);
void right(uint16_t speed);
void TimerPWM(enum Motor i_Motor, int dutyCycle);

// --- Implementierungen ---

int8_t initMain(void){
  initTimerPWM();
  stop_motors();
  InitADC(sensor_left);
  InitADC(sensor_right);

  // PD4 als Eingang
  DDRD &= ~(1 << MODE_PIN);
  // internen Pull-Up aktivieren
  PORTD |= (1 << MODE_PIN);
  _delay_ms(10);  // Warten bis Pin stabil ist
  if(PIND & (1 << MODE_PIN)){
    rMode = BangBang;       // Bang Bang Steuerung
  }
  else{
    rMode = P;       // P Regelung wenn LOW
  }

  DDRD &= ~(1 << CALIB_PIN);
  PORTD |= (1 << CALIB_PIN);

  return 1;
}

void InitADC(uint8_t channel){
    ADMUX=(1<<REFS0); // internal Vcc (+5 V) as REF voltage
    const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    ADMUX &= ~kMuxMask; // clear MUX
    ADMUX |= channel; // set input channel
    // prescale 128, ADC clock speed: 125 kHz, sampling freq: 9,6 kHz
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRA |= (1 << ADEN); // ADC aktivieren,
    ADCSRA |= (1 << ADSC); // Auslesen starten
}

uint16_t ReadADCSingleConversion(uint8_t channel){
    const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    ADMUX &= ~kMuxMask; // clear MUX
    ADMUX |= channel; // input channel setzen
    ADCSRA |= (1<<ADSC); // Auslesen starten
    do {} while( ADCSRA & (1<<ADSC) ); // Warten bis Auslesen beendet
    return ADC;
}

//-----------------------------------------------------
// P - REGELUNG
//-----------------------------------------------------

uint8_t getSensor(int16_t value, int16_t threshold, int16_t accuracy){
    uint8_t result  = (value >  (threshold - (threshold/100*accuracy)));
    return result;
}

float pMath(int16_t sensor1, int16_t sensor2, float Kp){
    int16_t fehler;          
    float stellgroesse;
    fehler = sensor1 - sensor2;  // Differenz Sensoren
    stellgroesse = (Kp * static_cast<float>(fehler)) ; // Stellgröße mit kp Faktor errechnen
    return stellgroesse;
}

void pController(uint16_t base_speed){
    if(base_speed > 1000) base_speed = 1000;
    
    // Statische Variable speichert die letzte bekannte Position der Linie
    // 1 = Linie war zuletzt links, -1 = Linie war zuletzt rechts
    static int8_t last_line_pos = 0; 

    int16_t sensor_left  = ReadADCSingleConversion(sensor_left);
    int16_t sensor_right = ReadADCSingleConversion(sensor_right);

    uint8_t left_black  = getSensor(sensor_left, threshold, 1);
    uint8_t right_black = getSensor(sensor_right, threshold, 1);

    if(left_black || right_black){
        if(sensor_left > sensor_right) {
            last_line_pos = 1;  // Links ist schwärzer
        } else {
            last_line_pos = -1; // Rechts ist schwärzer
        }

        
        float correction = pMath(sensor_left, sensor_right, Kp); // P-Regler aufrufen

        // Geschwindigkeiten berechnen
        float left_speed_float  = base_speed - (correction * base_speed); 
        float right_speed_float = base_speed + (correction * base_speed);

        int16_t left_speed  = (int16_t)left_speed_float;   
        int16_t right_speed = (int16_t)right_speed_float;  

        if(left_speed > 1000) left_speed = 1000;
        if(left_speed < -1000) left_speed = -1000;
        if(right_speed > 1000) right_speed = 1000;
        if(right_speed < -1000) right_speed = -1000;

        // Motoransteuerung
        if(left_speed >= 0) {
            forward(M1);
            set_Motor(M1, (uint16_t)left_speed/2);
        } else {
            backward(M1);
            set_Motor(M1, (uint16_t)abs(left_speed/2)); 
        }

        if(right_speed >= 0) {
            forward(M2);
            set_Motor(M2, (uint16_t)right_speed/2);
        } else {
            backward(M2);
            set_Motor(M2, (uint16_t)abs(right_speed/2));
        }
    }
    else{
        // FALLBACK: Beide Sensoren sehen Weiß (Linie verloren)
        uint16_t search_speed = base_speed; 

        if(last_line_pos == 1) {
            // Linie war zuletzt links
            left(search_speed);
        } 
        else if (last_line_pos == -1) {
            // Linie war zuletzt rechts
            right(search_speed);
        }
        else {
            // Edge Case beim Start
            run_forward(search_speed);
        }
    }
}

// KORRIGIERT: Eindeutiger Funktionsname, korrekte Funktionsaufrufe
void calibrate_safe_threshold_kp(void){
    uint16_t whitethreshold;
    uint16_t diff_thershold;

    threshold = calibrate_threshold(); 
    _delay_ms(1000);
    do{} while ((PIND & (1 << CALIB_PIN)));
    flash_led(2);
    whitethreshold = calibrate_threshold();

    save_threshold(threshold);

    diff_thershold = abs(threshold - whitethreshold);

    if(diff_thershold == 0){
        Kp = 0.005;   // Fallback
    }
    else{
        Kp = (1.0f / (float)diff_thershold);
    }

    save_kp(Kp);
    _delay_ms(5000);
}

// KORRIGIERT: Implementierung für flash_led hinzugefügt
void flash_led(uint8_t times){
    while (times--){
        PORTB |= (1 << PORTB5);   // LED an
        _delay_ms(500);
        PORTB &= ~(1 << PORTB5);  // LED aus
        _delay_ms(200);
    }
}

void save_kp(float value) { 
    eeprom_write_float(&eeprom_kp, value); 
} 

float load_kp(void) { 
    float val = eeprom_read_float(&eeprom_kp); 
    
    // Prüfe ob EEPROM beschreiben wurde
    if(isnan(val)){
        return 0.005; 
    }
    return val; 
} 

//-----------------------------------------------------
// Bang - Bang - Steuerung
//-----------------------------------------------------

/* ----------------------------- EEPROM ----------------------------- */ 
void save_threshold(uint16_t value) { 
    eeprom_write_word(&eeprom_threshold, value); 
} 

uint16_t load_threshold(void) { 
    uint16_t val = eeprom_read_word(&eeprom_threshold); 
    
    if(val == 0xFFFF){
        return 500; 
    }
    return val; 
} 
    
/* ----------------------------- Kalibrierung ----------------------------- */ 

void calibrate_safe_threshold(void) { 
    save_threshold(calibrate_threshold()); 
    _delay_ms(500); 
}

uint16_t calibrate_threshold(void) { 
    uint32_t sum = 0;
    uint16_t lthreshold = 0;
    
    for(uint8_t i = 0; i < 20; i++) { 
        sum += ReadADCSingleConversion(sensor_left);
        sum += ReadADCSingleConversion(sensor_right); 
        _delay_ms(20); 
    } 
    
    lthreshold = sum / 40; 
    return lthreshold;
}

void bangBang(uint16_t speed){
    uint16_t left  = ReadADCSingleConversion(sensor_left);
    uint16_t right = ReadADCSingleConversion(sensor_right);

    uint8_t left_black  = (left >  (threshold - (threshold/100*5)));
    uint8_t right_black = (right > (threshold - (threshold/100*5)));

    if(left_black && right_black){
        run_forward(speed/1.75);
    }
    else if(left_black && !right_black){
        left(speed/1.75);
    }
    else if(!left_black && right_black){
        right(speed/1.75);
    }
    else{
        run_backward(speed/1.75);
    }
}

void set_Motor(enum Motor i_Motor, uint16_t i_Motor_Speed){
    TimerPWM(i_Motor,((uint32_t)PWM_FREQ*i_Motor_Speed*90)/100000);  
}

void stop_motors(void){
    set_Motor(M1,0);
    set_Motor(M2,0);
}

void forward(enum Motor i_Motor){
    int mPort = 0;  // select Motor Port
    switch (i_Motor){
    case M1:    mPort = PB0;
                break;
    case M2:    mPort = PB3;
                break;
    };

    if(current_dir[i_Motor] != 1)
    {
        stop_motors();
        _delay_ms(50);
        
        PORTB |= (1 << mPort);
        current_dir[i_Motor] = 1;
    }
}

void backward(enum Motor i_Motor){
    int mPort = 0;  // select Motor Port
    switch (i_Motor){
    case M1:    mPort = PB0;
                break;
    case M2:    mPort = PB3;
                break;
    };

    if(current_dir[i_Motor] != 2)
    {
        stop_motors();
        _delay_ms(50);
        
        PORTB &= ~ (1 << mPort);
        current_dir[i_Motor] = 2;
    }
}

void run_forward(uint16_t speed){
    forward(M1);
    forward(M2);

    set_Motor(M1, speed);
    set_Motor(M2, speed);
}

void run_backward(uint16_t speed){
    backward(M1);
    backward(M2);

    set_Motor(M1, speed);
    set_Motor(M2, speed);
}

void left(uint16_t speed){      
    forward(M1);
    forward(M2);

    set_Motor(M1, speed/8);
    set_Motor(M2, speed);
}

void right(uint16_t speed){
    forward(M1);
    forward(M2);

    set_Motor(M1, speed);
    set_Motor(M2, speed/8);
}

void initTimerPWM(){
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler = 8
    ICR1 = PWM_FREQ; // PWM frequency = (16000000 Hz)/8/(TOP+1) = (1000 Hz)/8 = 125 Hz
    OCR1A = 0; // duty cycle for pin OC1A (PB1) = 3999/15999 = 25%, assuming ICR1 = 15999
    OCR1B = 0; // duty cycle for pin OC1B (PB2) = 11999/15999 = 75%, assuming ICR1 = 1599
    DDRB |= (1 << PB0) | (1<<PB1) | (1<<PB2) | (1<< PB3);
}

void TimerPWM(enum Motor i_Motor, int dutyCycle){
    switch (i_Motor){
    case M1:    OCR1A = dutyCycle;
                break;
    case M2:    OCR1B = dutyCycle;
                break;
    };
}


int main(void) {
  do{} while (!initMain());

    while(1)
    {
        // Geschwindigkeit über POTI
        uint16_t speed = ReadADCSingleConversion(POT1);
        switch (rMode){
        case P: pController(speed); break;
        case BangBang: bangBang(speed); break;
        };
        
        // Kalibrierung
        if(!(PIND & (1 << CALIB_PIN)))
        {
            switch (rMode){
            case P: calibrate_safe_threshold_kp(); break; // KORRIGIERT: Aufruf angepasst
            case BangBang: calibrate_safe_threshold(); break;
            };
        }

        _delay_ms(1);
    }
  }