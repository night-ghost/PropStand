#include "HX711.h"

//****************************************************
//инициализация HX711
//****************************************************
void Hx711::init() // 4.89 - 4.846
{

    //pinMode(_sck, OUTPUT);	
    //pinMode(_dt,  INPUT);
    gpio_set_mode(PIN_MAP[_sck].gpio_device, PIN_MAP[_sck].gpio_bit, GPIO_OUTPUT_PP);
    gpio_set_mode(PIN_MAP[_dt].gpio_device, PIN_MAP[_dt].gpio_bit, GPIO_INPUT_PU);
    
    get_zero();
}


//****************************************************
//
//****************************************************

#define SUM_ZERO 32
void Hx711::get_zero()
{
    float sum = 0;
    
    for(int i=0; i<SUM_ZERO; i++) {
	sum +=  read() /100.0;
    }
    
    Weight_zero = sum / SUM_ZERO;
} 

//****************************************************

#define SUM_WEIGHT 4
float Hx711::get_weight(float k)
{

    float sum = 0;
    
    for(int i=0; i<SUM_WEIGHT; i++){
	sum +=  read() /100.0;
    }

    float w = sum/SUM_WEIGHT - Weight_zero;
	
    return w/k;
}

//****************************************************
//чтение данных с HX711
//****************************************************
unsigned long Hx711::read(void)	//усиление 128
{
	uint32_t inp=0; 
	unsigned char m;
	volatile int err=0;

//	digitalWrite(_dt, HIGH);
//	delayMicroseconds(1);

	digitalWrite(_sck, LOW);
	delayMicroseconds(1);

        uint32_t t = millis();
  	while(digitalRead(_dt)){ // wait for data ready
  	    if(millis() - t > 1000) { // no data for a second
  	        return -1;
  	    }
  	}
  	
  	for(m=0;m<24;m++) { 
  	    inp<<=1;   	    
    	    digitalWrite(_sck, HIGH); 
	    delayMicroseconds(1);
	    volatile uint8_t v1 = digitalRead(_dt);
	    digitalWrite(_sck, LOW); 
	    volatile uint8_t v2 = digitalRead(_dt);
	    delayMicroseconds(1);
	    volatile uint8_t v3 = digitalRead(_dt);
	    
	    if(v1 != v2 || v1!=v3) {
	        err++;
	    }
	    if(v1) inp|=1; 
	} 
 	digitalWrite(_sck, HIGH); // 25 pulse  - ch_A, 128
	delayMicroseconds(1);
	digitalWrite(_sck, LOW); 
//	delayMicroseconds(1);

	inp ^= 0x800000;                // 
	
	return inp;
}
