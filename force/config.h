
#define HX711_A_SCK PB5 
#define HX711_A_DT PB6  

#define HX711_B_SCK PB7 
#define HX711_B_DT PB8  

/*
    Using the first SPI port (SPI_1)
    SS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
    SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
    MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
    MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN

    Using the second SPI port (SPI_2)
    SS    <-->  PB12 <-->  BOARD_SPI2_NSS_PIN
    SCK   <-->  PB13 <-->  BOARD_SPI2_SCK_PIN
    MISO  <-->  PB14 <-->  BOARD_SPI2_MISO_PIN
    MOSI  <-->  PB15 <-->  BOARD_SPI2_MOSI_PIN
*/

#define SPI1_NSS_PIN PA4    //SPI_1 Chip Select pin is PA4. You can change it to the STM32 pin you want.
#define SPI2_NSS_PIN PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

#define CURRENT_PIN PA7
#define VOLTAGE_PIN PA6

#define PWM_PIN PB0


// vibro


#define MAX_SAMPLES 3200 // 1s

// on SPI2
#define ADXL345_CS PB12
#define ADXL345_READY PA8
//#define PHOTO_IN_PIN PA9 // фотодиод - TIM1 CH2

#define PHOTO_IN_PIN PA15 // фотодиод - TIM1 CH2



//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;     //X-Axis Data 0
char DATAX1 = 0x33;     //X-Axis Data 1
char DATAY0 = 0x34;     //Y-Axis Data 0
char DATAY1 = 0x35;     //Y-Axis Data 1
char DATAZ0 = 0x36;     //Z-Axis Data 0
char DATAZ1 = 0x37;     //Z-Axis Data 1

