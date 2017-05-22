#include <Arduino.h>

// varsion 0.6

// eclipse don't allows to set breakpoint in .ino files :(
extern void setup_c();

extern void loop_c();



void setup() {
    setup_c();
}


void loop() {  
    loop_c();
}
