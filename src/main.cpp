// main.cpp
#include <Arduino.h>

// #include "p1.h"
// #include "p2.h"
// #include "p3.h"
#include "p4.h"


void (*current_interrupt_action)() = nullptr;

// Die EINZIGE ISR(INT0_vect) im gesamten Projekt!
ISR(INT0_vect) {
    // Wenn ein Programm eine Funktion angemeldet hat, führe sie aus
    if (current_interrupt_action != nullptr) {
        current_interrupt_action(); 
    }
}

int main(void) {
    init(); 

    
    // runProgram1();
    // runProgram2();
    // runProgram3();
    runProgram4(); 

    while(1) {
    }
    
    return 0;
}