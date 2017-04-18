#ifndef __HX712__H__
#define __HX712__H__

#include <Arduino.h>


class Hx711 {
public:
    Hx711(uint8_t sck, uint8_t dt)
    : _sck(sck)
    , _dt(dt)
    , Weight_zero(0)
    {}
    
    void init();
    float get_weight(float k);

private:
    void get_zero();
    unsigned long read(void);

    uint8_t _sck;
    uint8_t _dt;

    float Weight_zero;
};

#endif
