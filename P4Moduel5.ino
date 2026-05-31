#ifndef F_CPU
#define F_CPU 16000000UL // Fallback, falls in platformio.ini nicht definiert
#endif

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>  // Für _delay_ms()
#include <stdlib.h>      // Für itoa(), abs(), dtostrf()
#include <math.h>        // Für isnan()

// --- Pin- und Modus-Definitionen ---
#define MODE_PIN  4      // Beispiel: PD4 (Bitte ggf. anpassen)
#define CALIB_PIN 5      // Beispiel: PD5 (Bitte ggf. anpassen)

const int8_t P = 0;
const int8_t BangBang = 1;

// --- Enums und globale Zustände ---
enum Motor { M1 = 0, M2 = 1 };
int8_t current_dir[2] = {0, 0}; // Speichert die aktuelle Richtung der Motoren (0=Stop, 1=Vorwärts, 2=Rückwärts)

const int16_t PWM_FREQ = 15999;
const int16_t kBaud = 9600;
const int8_t PU1 = PC0;    
const int8_t PU2 = PC1;
const int8_t POT1 = 7;
volatile float Kp = 0.00166;       // *10^-1
uint16_t threshold = 600;
int8_t rMode = 0;

uint16_t EEMEM eeprom_threshold;
float EEMEM eeprom_kp;

// --- Funktionsprototypen (Vorwärtsdeklarationen) ---
void InitUSART(uint32_t baud_rate);
void initTimer1(void);
void stop_motors(void);
void InitADC(uint8_t channel);
uint16_t ReadADCSingleConversion(uint8_t channel);
void TransmitString(const char* my_string);
void TransmitByte(uint8_t data);
uint8_t ReceiveByte(void);
void TransmitUint16(uint16_t value);
void TransmitInt16(int16_t value);
void TransmitFloat(float value);
uint8_t eval_Sensor_Val(int16_t value, int16_t threshold, int16_t accuracy);
float pRegler(int16_t istwertPU1, int16_t istwertPU2, float Kp);
void p_control_mode(uint16_t base_speed);
void calibrate_safe_threshold_kp(void);
void flash_led(uint8_t times);
void save_kp(float value);
float load_kp(void);
void save_threshold(uint16_t value);
uint16_t load_threshold(void);
void calibrate_safe_threshold(void);
uint16_t calibrate_threshold(void);
void bang_bang_mode(uint16_t speed);
void ctrlMotor(enum Motor i_Motor, uint8_t i_Motor_Speed);
void set_forward(enum Motor i_Motor);
void set_backward(enum Motor i_Motor);
void drive_forward(uint8_t speed);
void drive_backward(uint8_t speed);
void turn_left(uint8_t speed);
void turn_right(uint8_t speed);
void setTimerDC(enum Motor i_Motor, int dutyCycle);

// --- Implementierungen ---

int8_t initc(void){
  InitUSART(kBaud);
  initTimer1();
  stop_motors();
  InitADC(PU1);
  InitADC(PU2);

  // PD4 als Eingang
  DDRD &= ~(1 << MODE_PIN);
  // internen Pull-Up aktivieren
  PORTD |= (1 << MODE_PIN);
  _delay_ms(10);  // Warten bis Pin stabil
  if(PIND & (1 << MODE_PIN)){ //umgekehrte Logik -> Pull up
    rMode = BangBang;       // Bang Bang Steuerung
    TransmitString("Mode: BangBang\n");
  }
  else{
    rMode = P;       // P Regelung
    TransmitString("Mode: P\n");
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
    // prescale divison factor 128, ADC clock speed: 125 kHz, sampling freq: 9,6 kHz
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRA |= (1 << ADEN); // enable ADC,
    ADCSRA |= (1 << ADSC); // start converting
}

uint16_t ReadADCSingleConversion(uint8_t channel){
    const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    // clear MUX
    ADMUX &= ~kMuxMask;
    // set input channel
    ADMUX |= channel;
    // start first single conversion
    ADCSRA |= (1<<ADSC);
    // wait until it is finished
    do {} while( ADCSRA & (1<<ADSC) );
    // return result
    return ADC;
}

void InitUSART(uint32_t baud_rate){
    uint16_t ubrr_val = (F_CPU - 8 * baud_rate)/(16*baud_rate); // clever rounding
    UBRR0 = ubrr_val;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) ; // Enable receiver and transmitter
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void TransmitByte(uint8_t data){
    do{} while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

uint8_t ReceiveByte(void){
    do{} while (!(UCSR0A & (1<<RXC0)));
    return UDR0;
}

void TransmitString(const char* my_string) {
    uint16_t i = 0;
    while ( my_string[i] != '\0' ) {
        TransmitByte(my_string[i]);
        i++;
    }
}

void TransmitUint16(uint16_t value){
    char buffer[6]; 
    itoa(value, buffer, 10); 
    TransmitString(buffer);
    TransmitByte('\n');
}

void TransmitInt16(int16_t value){
    char buffer[7]; 
    itoa(value, buffer, 10); 
    TransmitString(buffer);
    TransmitByte('\n');
}

void TransmitFloat(float value){
    char buffer[16];
    dtostrf(value, 0, 3, buffer);
    TransmitString(buffer);
    TransmitByte('\n');
}

//-----------------------------------------------------
// P - REGELUNG
//-----------------------------------------------------

uint8_t eval_Sensor_Val(int16_t value, int16_t threshold, int16_t accuracy){
    uint8_t result  = (value >  (threshold - (threshold/100*accuracy)));
    return result;
}

float pRegler(int16_t istwertPU1, int16_t istwertPU2, float Kp){
    int16_t fehler;           // Regeldifferenz
    float stellgroesse;

    // Berechung der Regeldifferenz
    fehler = istwertPU1 - istwertPU2;   
    
    // Berechung des Ausgangswertes der Regeung
    stellgroesse = (Kp * static_cast<float>(fehler)) ;
    return stellgroesse;
}

void p_control_mode(uint16_t base_speed){
    // Clamp base_speed to 0-1024 for safety
    if(base_speed > 1024) base_speed = 1024;
    
    int16_t sensor_left;
    int16_t sensor_right;

    float correction;

    float left_speed_float;
    float right_speed_float;
    int16_t left_speed;      // Keep as int16_t
    int16_t right_speed;     // Keep as int16_t

    // Sensorwerte einlesen
    sensor_left  = ReadADCSingleConversion(PU1);
    sensor_right = ReadADCSingleConversion(PU2);

    uint8_t left_black  = eval_Sensor_Val(sensor_left,threshold,1);
    uint8_t right_black = eval_Sensor_Val(sensor_right,threshold,1);

    if(left_black || right_black)
    {
        // P-Regler aufrufen
        correction = pRegler(sensor_left, sensor_right, Kp);
        if(correction > 1.0f) correction = 1.0f;   // Begrenzung der Korrektur auf max 100%
        if(correction < -1.0f) correction = -1.0f;

        // Motorgeschwindigkeiten berechnen as floats with correction
        left_speed_float  = (1.0 - correction) * base_speed;
        right_speed_float = (1.0 + correction) * base_speed;

        // Round floats and convert to int16_t with bounds checking
        left_speed  = (int16_t)(left_speed_float + 0.5);   
        right_speed = (int16_t)(right_speed_float + 0.5);  

        // Begrenzung Motoren auf 0-1024 (after rounding)
        if(left_speed < 0) left_speed = 0;
        if(left_speed > 1024) left_speed = 1024;

        if(right_speed < 0) right_speed = 0;
        if(right_speed > 1024) right_speed = 1024;

        // beide Vorwärts
        set_forward(M1);
        set_forward(M2);

        ctrlMotor(M1, (uint8_t)left_speed);   
        ctrlMotor(M2, (uint8_t)right_speed);  
    }
    else
    {
        // beide Motoren rückwärts
        set_backward(M1);
        set_backward(M2);

        ctrlMotor(M1, base_speed);
        ctrlMotor(M2, base_speed);
    }
}

void calibrate_safe_threshold_kp(void){
    uint16_t whitethreshold;
    uint16_t diff_thershold;

    TransmitString("Calib Kp and Th\n");

    flash_led(1);
    threshold = calibrate_threshold();
    _delay_ms(1000);
    do{} while ((PIND & (1 << CALIB_PIN))); // Wait until button pressed
    flash_led(2);
    whitethreshold = calibrate_threshold();

    save_threshold(threshold);

    diff_thershold = abs(threshold - whitethreshold);

    if(diff_thershold == 0)
    {
        Kp = 0.005;   // Fallback
    }
    else
    {
        Kp = (1.0f / (float)diff_thershold);
    }

    save_kp(Kp);

    TransmitString("Kp\n");
    TransmitFloat(Kp);
    TransmitString("TH1:");
    TransmitUint16(threshold);
    TransmitString("TH2:");
    TransmitUint16(whitethreshold);
    _delay_ms(5000);
}

void flash_led(uint8_t times){
    while (times--)
    {
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
    
    for(uint8_t i = 0; i < 20; i++) 
    { 
        sum += ReadADCSingleConversion(PU1);
        sum += ReadADCSingleConversion(PU2); 

        if(i==0){
            TransmitUint16(sum/(2));
        }
        else{
            TransmitUint16(sum/(i*2));
        }
        _delay_ms(20); 
    } 
    
    lthreshold = sum / 40; 

    TransmitString("Calib\n");
    TransmitUint16(lthreshold);
    return lthreshold;
}

void bang_bang_mode(uint16_t speed){
    uint16_t left  = ReadADCSingleConversion(PU1);
    uint16_t right = ReadADCSingleConversion(PU2);

    uint8_t left_black  = (left >  (threshold - (threshold/100*5))); // 5 prozent schwelle on top
    uint8_t right_black = (right > (threshold - (threshold/100*5)));

    TransmitString("left:");
    TransmitUint16(left);
    TransmitString("\n");
    TransmitUint16(left_black);
    TransmitString("\n");
    TransmitString("right:");
    TransmitUint16(right);
    TransmitString("\n");
    TransmitUint16(right_black);
    TransmitString("\n");

    if(left_black && right_black)
    {
        drive_forward(speed);
    }
    else if(left_black && !right_black)
    {
        turn_left(speed);
    }
    else if(!left_black && right_black)
    {
        turn_right(speed);
    }
    else
    {
        drive_backward(speed);
    }
}

void ctrlMotor(enum Motor i_Motor, uint8_t i_Motor_Speed){
    setTimerDC(i_Motor,((uint32_t)PWM_FREQ*i_Motor_Speed*90)/100000);  // set duty cycle for motor speed control, max 90% to avoid overcurrent
}

void stop_motors(void){
    ctrlMotor(M1,0);
    ctrlMotor(M2,0);
}

void set_forward(enum Motor i_Motor){
    int mPort = 0;  // select Motor Port
    switch (i_Motor){
    case M1:    mPort = PB0;
                break;
    case M2:    mPort = PB3;
                break;
    default: TransmitString("No Motor selected\n");
    };

    if(current_dir[i_Motor] != 1)
    {
        stop_motors();
        _delay_ms(50);
        
        PORTB |= (1 << mPort);// forward
        current_dir[i_Motor] = 1;
    }
}

void set_backward(enum Motor i_Motor){
    int mPort = 0;  // select Motor Port
    switch (i_Motor){
    case M1:    mPort = PB0;
                break;
    case M2:    mPort = PB3;
                break;
    default: TransmitString("No Motor selected\n");
    };

    if(current_dir[i_Motor] != 2)
    {
        stop_motors();
        _delay_ms(50);
        
        PORTB &= ~ (1 << mPort); // backward  
        current_dir[i_Motor] = 2;
    }
}

void drive_forward(uint8_t speed){
    set_forward(M1);
    set_forward(M2);

    ctrlMotor(M1, speed);
    ctrlMotor(M2, speed);
}

void drive_backward(uint8_t speed){
    set_backward(M1);
    set_backward(M2);

    ctrlMotor(M1, speed);
    ctrlMotor(M2, speed);
}

void turn_left(uint8_t speed){      
    set_forward(M2);     
    set_backward(M1);   

    ctrlMotor(M1,speed);;
    ctrlMotor(M2, speed);
}

void turn_right(uint8_t speed){
    set_forward(M1);
    set_backward(M2);

    ctrlMotor(M1, speed);
    ctrlMotor(M2,speed);
}

void initTimer1(){
    TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler = 8
    ICR1 = PWM_FREQ; // PWM frequency = (16000000 Hz)/8/(TOP+1) = (1000 Hz)/8 = 125 Hz
    OCR1A = 0; // duty cycle for pin OC1A (PB1) = 3999/15999 = 25%, assuming ICR1 = 15999
    OCR1B = 0; // duty cycle for pin OC1B (PB2) = 11999/15999 = 75%, assuming ICR1 = 1599
    DDRB |= (1 << PB0) | (1<<PB1) | (1<<PB2) | (1<< PB3); // output pins, globally set above already
}

void setTimerDC(enum Motor i_Motor, int dutyCycle){
    switch (i_Motor){
    case M1:    OCR1A = dutyCycle; // duty cycle for pin OC1A (PB1) 
                break;
    case M2:    OCR1B = dutyCycle; // duty cycle for pin OC1B (PB2) 
                break;
    default: TransmitString("No Motor selected\n");
    };
}


int main(void) {
  do{} while (!initc());

    while(1)
    {
        // Geschwindigkeit über POTI
        uint16_t speed = ReadADCSingleConversion(POT1);

        switch (rMode){
        case P: p_control_mode(speed); break;
        case BangBang: bang_bang_mode(speed); break;
        default: TransmitString("No Mode Selected\n"); break;
        };

        // Kalibrierung
        if(!(PIND & (1 << CALIB_PIN)))
        {
            switch (rMode){
            case P: calibrate_safe_threshold_kp(); break;
            case BangBang: calibrate_safe_threshold(); break;
            default: TransmitString("No Mode Selected\n"); break;
            };
        }

        _delay_ms(1);
    }
  }