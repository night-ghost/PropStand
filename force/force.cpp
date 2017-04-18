#include <Arduino.h>

#include "config.h"

#include <SPI.h>

#include "HX711.h"
#include <Servo.h> 
#include <libmaple/timer.h>

//#include <stm32f4xx.h>
//#include <hal.h>
#include "stopwatch.h"


//#define MCU_STM32F103R8
#include <EEPROM.h>

#define MIN_PWM 1000

#define LED_PIN PC13
#define LED_ON 0
#define LED_OFF 1

#define USB_DP PA12
#define USB_DM PA11


void writeRegister(uint8_t registerAddress,uint8_t value);
void readRegister(char registerAddress, int numBytes,unsigned char * values);


volatile long time_last = 0;
volatile float rpm; // calculated in interrupt
float last_rpm=0;

uint16_t pwm_val = MIN_PWM ;

unsigned long previousMillis = 0; // храним время
unsigned long currentMillis = 0;

const unsigned long STOP_TIME = 1000;  // 1s stop time
byte start_rotation=0;

int16_t current0; // начальный ток модуля

Servo servo1; //объявляем 

Hx711 tenzo1(HX711_A_SCK,HX711_A_DT);

Hx711 tenzo2(HX711_B_SCK,HX711_B_DT);

char serial_buf[80];
uint8_t in_count;

enum Modes {
    WAIT_COMMAND=0,
    CALC_FORCE,
    BALANCE,
    FORCE_DEBUG,
    BALANCE_DEBUG,
    MOTOR_DEBUG
};

enum Modes current_mode = WAIT_COMMAND;  

SPIClass SPI_2(2);

void fan_interrupt();
void fan_interrupt2();
void writeRegister(uint8_t registerAddress,uint8_t value);
void get_samples();
void calibrate_accel();
void timer1_overflow();
void reset_rpm();
void set_pwm(uint16_t val);


// [ vibro 

// stm32 has 10times more memory than arduino
int16_t sample[MAX_SAMPLES];
byte zero[MAX_SAMPLES];

int16_t acc_x0;
int16_t acc_y0;
int16_t acc_z0;


// ]


#define HIST_SIZE 512

uint32_t timer_hist[HIST_SIZE];
uint16_t hist_ptr=0;

uint32_t timer_last_hist[HIST_SIZE];
uint16_t hist_last_ptr=0;


uint32_t regs[16] = { // регистры настройки режимов
    0,// r0 - координата, возвращаемая при измерении вибрации: 0 - x, 1 - y, 2 - z, 3 - max
    (uint32_t)(2710 * 1000L / 12.56),// r1 - коэффмциент коррекции напряжения * 1000
    1000L,// r2 - коэффмциент коррекции тока * 1000
    155   * 1000L /20,// r3 - коэффициент тензодатчика силы, *1000
    415   * 1000L /20,// r4 - коэффициент тензодатчика момента, *1000
};

uint32 timer_tick;


/* 
 * Provides a micro-second granular delay using the CPU cycle counter.
 */

/* cycles per microsecond */
uint32_t us_ticks;


void stopwatch_init(void)
{
//        RCC_ClocksTypeDef       clocks;
        /* compute the number of system clocks per microsecond */
//        RCC_GetClocksFreq(&clocks);
//        us_ticks = clocks.SYSCLK_Frequency / 1000000;
        us_ticks = CYCLES_PER_MICROSECOND; // F_CPU / 1000000;

        /* turn on access to the DWT registers */
        DEMCR |= DEMCR_TRCENA;
        /* enable the CPU cycle counter */
        DWT_CTRL |= CYCCNTENA;

        stopwatch_reset();
}


void stopwatch_delay_us(uint32_t us){
//      stopwatch_reset(); we can't do that because any delay() in interrupt will reset main counter. It should be free running
    uint32_t ts = stopwatch_getticks(); // start time in ticks
    uint32_t dly = us * us_ticks;       // delay in ticks
    while(1) {
        uint32_t dt;
        uint32_t now = stopwatch_getticks(); // current time in ticks

        dt = now - ts;
        if (dt >= dly)
                break;
    }
}





void  eeprom_read_len(byte *p, uint16_t e, uint16_t l){
    uint16_t *wp = (uint16_t  *)p;
    l/=2;
    
    for(;l!=0; l--) {
        *wp++ = (byte)EEPROM.read( e++ );
    }
}

void eeprom_write_len(byte *p, uint16_t e, uint16_t l){
    uint16_t *wp = (uint16_t  *)p;
    l/=2;

    for(;  l!=0; l--) {
        EEPROM.write( e++, *wp++);
    }

}


void setup_c() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_ON);

//    Serial.begin(115200);
    Serial.end();
    
    stopwatch_init();
    
    // search for BOARD_USB_DISC_BIT
    
//    USB_OTG_dev.regs.GREGS->GCCFG = 0;
//    usb_suspend();

    pinMode(USB_DP, OUTPUT);
    pinMode(USB_DM, OUTPUT);
    digitalWrite(USB_DP, 0);
    digitalWrite(USB_DM, 0);
    delay(12);
    pinMode(USB_DP, INPUT_PULLUP);
    pinMode(USB_DM, INPUT);

    delay(10);
    
//    usb_resume_init();
    Serial.begin(115200);


    EEPROM.init();
    
    uint16_t sts = 0, cnt=0;

    sts=EEPROM.count(&cnt);
    
    if(sts==EEPROM_OK && cnt>0) {
        eeprom_read_len((byte *)&regs,0, sizeof(regs));
    }
    
    

/*
 // Setup SPI 1
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(SPI1_NSS_PIN, OUTPUT);


  // Setup SPI 2
  SPI_2.begin(); //Initialize the SPI_2 port.
  SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
  pinMode(SPI2_NSS_PIN, OUTPUT);
*/  


 
    servo1.attach(PWM_PIN); //выход сервосигнала
    servo1.writeMicroseconds(MIN_PWM);
  

// [ vibro

    //Initiate an SPI communication instance.
    SPI_2.begin();
    //Configure the SPI connection for the ADXL345.
    SPI_2.setDataMode(SPI_MODE3);
    SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
    SPI_2.setClockDivider(SPI_CLOCK_DIV16);

    //Set up the Chip Select pin to be an output from the Arduino.
    pinMode(ADXL345_CS, OUTPUT);
    //Before communication starts, the Chip Select pin needs to be set high.
    digitalWrite(ADXL345_CS, HIGH);
    pinMode(ADXL345_READY, INPUT_PULLUP);
    pinMode(ADXL345_INP, INPUT_PULLUP);

    attachInterrupt(ADXL345_INP, fan_interrupt2, FALLING);
/* нога A9 подключена к TIM1 CH2, так что лучше использовать аппаратный таймер для захвата времени
   вот только подпаяно к A15


#define MAX_RELOAD ((1 << 16) - 1)
    
    uint32 microseconds = 500000; //timer's period 0.5s - min rotate speed 120rpm
    
    uint32 period_cyc = microseconds * CYCLES_PER_MICROSECOND;
    uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
    timer_set_prescaler(TIMER1, prescaler);
    
    timer_tick = CYCLES_PER_MICROSECOND * prescaler; // one tick
    
    gpio_set_mode(PIN_MAP[ADXL345_INP].gpio_device, PIN_MAP[ADXL345_INP].gpio_bit, GPIO_AF_OUTPUT_PP);
    
//    timer_set_capture_mode(TIMER1, 2, TIMER_IC_MODE_FILTER_0 | TIMER_IC_MODE_PRESCALER_8 | TIMER_IC_CAPTURE_RISING, 0);
    timer_set_mode(TIMER1, 2, TIMER_INPUT_CAPTURE);
    
//    timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, TIMER_PWM : TIMER_DISABLED);


    timer_attach_interrupt(TIMER1, TIMER_CC2_INTERRUPT, fan_interrupt);
    

//    timer_set_mode(TIMER1, 4, TIMER_OUTPUT_COMPARE);
//    timer_set_compare(TIMER1, 4, );
    
    timer_attach_interrupt(TIMER1, TIMER_UPDATE_INTERRUPT, timer1_overflow);
    timer_resume(TIMER1);
*/    
    


    //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
    writeRegister(POWER_CTL, 0x08);  //Measurement mode  
    //Put the ADXL345 into +/- 8G range by writing the value 0x01 to the DATA_FORMAT register.
    writeRegister(DATA_FORMAT, 0x02);
    
    writeRegister(0x2C, 0x0F); // 3200 HZ
    writeRegister(0x2E, 0x80); // set dataready to interrupt pin INT1 and active high
/* 
    writeRegister(0x1E, 0x00); // X-offset to 0
    writeRegister(0x1F, 0x00); // Y-offset to 0
    writeRegister(0x20, 0x00); // Z-offset to 0
*/

// ]

//    delay(1000); // time to ESC
    calibrate_accel();

    pinMode(CURRENT_PIN,INPUT_ANALOG);
    pinMode(VOLTAGE_PIN,INPUT_ANALOG);


    tenzo1.init();	 //инициализация датчика веса 9s per 100 inputs

    tenzo2.init();	 //инициализация датчика момента  

    current0 = analogRead(CURRENT_PIN);

    digitalWrite(LED_PIN, LED_OFF);


    Serial.println("Propeller stand");
    Serial.println(" press 'h' to get help");  
}

void set_pwm(uint16_t val){
    uint16_t last = pwm_val;
    last_rpm=0;
    pwm_val = val;
    servo1.writeMicroseconds(val); 
    delay(200); //  время на стабилизацию оборотов

    
    if(last == MIN_PWM || abs(pwm_val-last)>100){
        reset_rpm();
        uint32_t t=millis();
        while(rpm==0) { // wait for RPM measure
            if(millis()-t > 100) break;
        }
    }

    if(val != MIN_PWM) {
        previousMillis = millis();
    }

}

void loop_c() {  
    bool got_string=false;

    while(Serial.available()) {
        char c = Serial.read();
        switch(c) {
        case '\r':
            break;
            
        case '\n':
            if(in_count) got_string=true;
            serial_buf[in_count]=0; // closed string
            in_count=0;
            break;

        case '.': //  1st symbol of line - ends mode
            if(in_count == 0) {
                c = Serial.read(); //  get LF

                current_mode = WAIT_COMMAND;
                serial_buf[0]=0; // no command
                in_count=0;
                serial_buf[in_count]=0; // closed string
                set_pwm(MIN_PWM );
                break;
            }
            // no break!
        default:
            serial_buf[in_count++] = c;
            serial_buf[in_count]=0; // closed string
        }
    }

    switch(current_mode) {
    case CALC_FORCE: {
        float force  = tenzo1.get_weight(regs[3]/1000.0);
        float moment = tenzo2.get_weight(regs[4]/1000.0);
  
        uint16_t Volt    = analogRead(VOLTAGE_PIN); //чтение напряжения
        int16_t current  = analogRead(CURRENT_PIN);
    
        Serial.print(abs(force)); // 
        Serial.print(' ');
        Serial.print(abs(moment)); // момент
        Serial.print(' ');   
        Serial.print(Volt * regs[1] / 1000.0);	//напряжение
        Serial.print(' ');
        Serial.print((current-current0) * regs[2] / 1000.0, DEC); //ток
        Serial.print(' ');
        Serial.print(pwm_val);	//сервосигнал
        Serial.print(' ');  
        Serial.println(rpm); //обороты

        if(got_string) {
            set_pwm(atoi(serial_buf)); 
        }

        currentMillis = millis();

        if( currentMillis - previousMillis > STOP_TIME) { //  остановка при потере связи
            set_pwm(MIN_PWM );
        }
    } break;

    case FORCE_DEBUG: { // without motor
        float force  = tenzo1.get_weight(regs[3]/1000.0);
        float moment = tenzo2.get_weight(regs[4]/1000.0);
  
        uint16_t Volt   = analogRead(VOLTAGE_PIN); //чтение напряжения
        int16_t current = analogRead(CURRENT_PIN);
        if(current<current0) current0=current;
    
        Serial.print(force); // 
        Serial.print(' ');
        Serial.print(moment); // момент
        Serial.print(' ');   
        Serial.print(Volt / ( regs[1] / 1000.0));	//напряжение
        Serial.print(' ');
        Serial.print((current-current0) * regs[2] / 1000.0, DEC); //ток
        Serial.print(' ');
        Serial.print(pwm_val);	//сервосигнал
        Serial.print(' ');  
        Serial.println(rpm); //обороты

    } break;
        
    case BALANCE: {
        if(got_string) {
            set_pwm(atoi(serial_buf)); 
        }

        currentMillis = millis();

        if( currentMillis - previousMillis > STOP_TIME) { //  остановка при потере связи
            set_pwm(MIN_PWM );
        }
        
        get_samples(); // read ADXL and send samples
        
        } break;

    case BALANCE_DEBUG:  // without motor
        
        get_samples(); // read ADXL and send samples
        
        break;

    case MOTOR_DEBUG:
        if(got_string) {
            set_pwm(atoi(serial_buf)); 
        }
        if(last_rpm != rpm) {
            Serial.print(pwm_val);	//сервосигнал
            Serial.print(' ');  
            Serial.println(rpm); //обороты
            last_rpm = rpm;
        }

        if(start_rotation) { // each rotation
            start_rotation=0;
            uint16_t nv=hist_last_ptr;

            Serial.print(nv);
            Serial.print("> ");
            for(int i=0;i<nv; i++){
                Serial.print(timer_last_hist[i]);
                Serial.print(' ');
            }                
            Serial.println();
        }

        break;
 

    case WAIT_COMMAND: {
        if(!got_string) break;

        if(serial_buf[0] && serial_buf[1]==0 ){ // commands
            switch(serial_buf[0]){
            case 'f': // force
                current_mode = CALC_FORCE;
                Serial.println("Force mode");
                reset_rpm();
                break;

            case 'g': // force
                current_mode = FORCE_DEBUG;
                Serial.println("Force debug mode");
                break;

            case 'b':
                current_mode = BALANCE;
                Serial.println("Balance mode");
                reset_rpm();
                break;

            case 'c':
                current_mode = BALANCE_DEBUG;
                Serial.println("Balance debug mode");
                break;

            case 'm':
                current_mode = MOTOR_DEBUG;
                Serial.println("Motor debug mode");
                reset_rpm();
                break;


            case 'h':
                Serial.println("Commands:\n"
                " f - measure force\n"
                " b - balance\n"
                " g - force debug\n"
                " c - balance debug\n"
                " m - motor debug\n"
                " r - recalibrate\n"
                " w - write registers to EEPROM\n"
                " Rn=val - set value to register\n\n"
                "Registers:\n"
                " R0 - vibration coordinate\n"
                " R1 - voltage factor * 1000\n"
                " R2 - current factor * 1000\n"
                " R3 - force tenzo factor * 1000\n"
                " R4 - torque tenzo factor * 1000\n");
                
                break;
                
            case 'r':
                Serial.print("Calibration");
                tenzo1.init();	 //инициализация датчика веса
                tenzo2.init();	 //инициализация датчика момента  
            
                calibrate_accel();
            
                Serial.println(" done");

                break;

            case 's':
                Serial.print("R0="); Serial.println(regs[0]);
                Serial.print("R1="); Serial.println(regs[1]);
                Serial.print("R2="); Serial.println(regs[2]);
                Serial.print("R3="); Serial.println(regs[3]);
                Serial.print("R4="); Serial.println(regs[4]);
                break;
            
            case 'w':
                eeprom_write_len((byte *)&regs, 0, sizeof(regs));
                break;
            
            default:
                Serial.print("Bad command - ");
                Serial.println(serial_buf);
            }
        
        } else { // variables: Rn=value or n?
            // assume 1st char is R
            char *bp=serial_buf+1;
                            
            byte n=atol(bp); 
            if(n > sizeof(regs)/sizeof(uint32_t)) {
                Serial.print("Bad register - ");
                Serial.println(n);
                
                    break;
            }
                            
            bool ok=false;
            while(*bp) {
                if(*bp == '?' ){
                    Serial.printf("R%d=%ld\n",n, regs[n]);
                    ok=true;
                    break;
                }
                if(*bp++ == '=') {
                    regs[n] = atol(bp); // если не пустая строка то преобразовать и занести в численный параметр                
                    ok=true;
                    break;
                }                
            }
            
            if(!ok){
                Serial.print("Bad command - ");
                Serial.println(serial_buf);
            }
        
        }
    
    }   break;
    
    default:
        break;
    }
}


uint8_t sendSPI(uint8_t b)
{
  digitalWrite(SPI1_NSS_PIN, LOW); // manually take CSN low for SPI_1 transmission
  b = SPI.transfer(b); //Send data over SPI-1 port and return the received byte
  digitalWrite(SPI1_NSS_PIN, HIGH); // manually take CSN high between spi transmissions
  return b;
}


uint8_t sendSPI2(uint8_t b)
{
  digitalWrite(SPI2_NSS_PIN, LOW); // manually take CSN low for SPI_2 transmission
  b = SPI_2.transfer(b); //Send data over SPI-2 port and return the received byte
  digitalWrite(SPI2_NSS_PIN, HIGH); // manually take CSN high between spi transmissions
  return b;
}



volatile uint32_t last_tim=0;


uint32_t t_max=0;
uint8_t rpm_mode = 0;
volatile uint32_t int_cnt=0;
//volatile uint16_t ovf_cnt=0;

//volatile uint32_t int2_cnt=0;
uint32_t res_time=0;

uint32_t last_intcnt=0;
uint16_t sides=0;


/*
void timer1_overflow(){

    rpm_mode=1; // turn to measure mode
    ovf_cnt++;
}
*/

void reset_rpm(){
    t_max=0;
    rpm_mode = 0; // накопление
    int_cnt = 0;
    rpm=0;
    last_rpm=0;
    res_time=millis();
    last_intcnt=0;
}

static uint32 _micros(void) {
    uint32 ms;
    uint32 cycle_cnt;


    noInterrupts();
    ms = millis();
    cycle_cnt = systick_get_count();
    if(systick_check_underflow()) ms++;
    interrupts();

#define US_PER_MS               1000
    /* SYSTICK_RELOAD_VAL is 1 less than the number of cycles it
     * actually takes to complete a SysTick reload */
    return ((ms * US_PER_MS) +
            (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND);
#undef US_PER_MS
}


void calc_revo(){
    start_rotation=1; // as label

    //uint32_t t = _micros(); // запомним его время
    uint32_t t = stopwatch_getticks();    
    uint32_t dt = t - time_last;
    time_last = t;

    //Update The RPM
    if(dt > 0) {
        float r = 60000000.0 * us_ticks / dt / sides; // in RPM

        if(rpm==0) rpm = r;
        else       rpm += (r - rpm) / 8;        
    }        

    memmove(timer_last_hist, timer_hist, sizeof(timer_hist) );
    hist_last_ptr=hist_ptr;


    hist_ptr=0;
}

#define PULSE_GAP 30L //in percent

void fan_interrupt(){
/*
    uint32_t capt =  timer_get_capture(TIMER1, 2) + (ovf_cnt<<16);
    uint32_t dt = capt - last_tim;
    last_tim = capt;
*/    

    togglePin(LED_PIN);

    uint32_t capt = stopwatch_getticks();
    uint32_t dt = (capt - last_tim) / us_ticks;
    last_tim = capt;

    int_cnt++; // number of interrupts between large times

    if(dt<30) return;

    if(hist_ptr<HIST_SIZE) {
        timer_hist[hist_ptr++] = dt;
    }


    if((int32_t)dt < 0) { // strange things happens
        return;
    }

    if(dt > 500000) return; // not count if less than 2/s
    
    switch(rpm_mode) {
    case 0: // поиск максимальной длины
        if(dt > (t_max - (t_max*PULSE_GAP/100)) ) { // это может быть самый длинный
            time_last = _micros(); // запомним его время
        }    
    
        if(dt > t_max) {
            t_max=dt;
            
            last_intcnt=int_cnt;
            int_cnt=0;
        }
        if(millis() - res_time > 500)  rpm_mode=1; // turn to measure mode after 0.5s

        break;
        
    case 1: // вычисление RPM в предположении что максимальная длительность как минимум вдвое больше остальных
        if(dt > (t_max - (t_max*PULSE_GAP/100)) ) { // это как раз самый длинный, меряем период - 10% разницы на оборот 
            t_max = dt; // сохраним для компенсации плавного изменения скорости


            if(int_cnt == last_intcnt || (int_cnt + last_intcnt) / abs(int_cnt - last_intcnt) > 20) {
                sides=1;
                calc_revo();
            } else { // отличаются
                sides=2;            
                if(int_cnt > last_intcnt) {
                    calc_revo();
                }
            }

            last_intcnt=int_cnt;
            int_cnt=0;
                
        } else { // check time from last interrupt
//            if(t_max > 
        }
    }
}

void fan_interrupt2(){
    
    fan_interrupt();
}


struct ADXL345_data {
    int16_t x;
    int16_t y;
    int16_t z;
};


inline uint16_t abs_max(uint16_t n1, uint16_t n2){
    if(abs(n1)>abs(n2)) return n1;
    return n2;
}


void calibrate_accel(){
    struct ADXL345_data d;
    float x=0,y=0,z=0;

    for (uint16_t sample_index=0; sample_index<MAX_SAMPLES; sample_index++) {
        while (digitalRead(ADXL345_READY) == LOW) ;
                 

      //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
      //The results of the read operation will get stored to the values[] buffer.

        readRegister(DATAX0, 6, (uint8_t *)&d);

        x+=d.x;
        y+=d.y;
        z+=d.z;
    }
    
    acc_x0 = x / MAX_SAMPLES;
    acc_y0 = y / MAX_SAMPLES;
    acc_z0 = z / MAX_SAMPLES;
    
}

void get_samples(){

    struct ADXL345_data d;

    for (uint16_t sample_index=0; sample_index<MAX_SAMPLES; sample_index++) {
        while (digitalRead(ADXL345_READY) == LOW) ;
                 

      //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
      //The results of the read operation will get stored to the values[] buffer.

        readRegister(DATAX0, 6, (uint8_t *)&d);

        zero[sample_index] = start_rotation; // Photo label processed
        start_rotation = 0;

        //The ADXL345 gives 10-bit acceleration values

        switch(regs[0]) {
        case 0:
            sample[sample_index]=d.x-acc_x0;
            break;
        
        case 1:
            sample[sample_index]=d.y-acc_y0;
            break;

        case 2:
            sample[sample_index]=d.z-acc_z0;
            break;
        
        case 3:
            sample[sample_index]=abs_max(abs_max(d.x-acc_x0,d.y-acc_y0), d.z-acc_z0);
            break;        

        case 4:
            sample[sample_index]= d.x + d.y + d.z - (acc_x0+acc_y0+acc_z0);
            break;        
        }
    }


    
    for (uint16_t t=0; t < MAX_SAMPLES; t++) {
        //Print the results to the terminal.
        Serial.print(sample[t], DEC);
        Serial.print(' '); 
        Serial.println(zero[t]);
    }
    
    Serial.println(".");
  
} 


//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(uint8_t registerAddress,uint8_t value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(ADXL345_CS, LOW);
  //Transfer the register address over SPI_2.
  SPI_2.transfer(registerAddress);
  //Transfer the desired register value over SPI_2.
  SPI_2.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(ADXL345_CS, HIGH);
}
  
//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes,unsigned char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1) address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(ADXL345_CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI_2.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI_2.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(ADXL345_CS, HIGH);
}
