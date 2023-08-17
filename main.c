//  STM32F103-CMSIS-I2C-LED-Scanner
//
//  Simple I2C demonstration that uses an IC2 LCD driver module and my STM32F103-CMSIS-I2C-lib.c
//  library.
//
//  Instead of driving an LCD display, the module will drive 8 LEDs based on I2C input.
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 06/2023
//
//  ==========================================================================
//  This I2C demonstration is based on the excellent writeup and video made by
//  Controllers Tech. I highly recommend viewing his content!
//
//    https://controllerstech.com/stm32-i2c-configuration-using-registers/
//  ==========================================================================
//
// 
//  REQUIRED HARDWARE
//  =================
//  STM32F103 Blue Pill
//  ST-LINK V2 programmer or other device for programming the Blue Pill
//  I2C LCD Driver Module
//  8 LEDs
//  7 LED dropper resistors ( 330 to 570 ohms should be fine for 3.3 V)
//  1 LED dropper resistor of approx 4.7k ohms for P3.
//  Breadboard and jumper wires

//  HARDWARE SETUP
//  ==============
//
//     Blue Pill  I2C LCD Driver    LED     Resistor   3.3V / GND
//     =========  ==============  =======   ========   ==========
//        GND ------- GND (4P) ------------------------ GND
//         5V ------- VCC (4P) ------------------------ 5V
//         
//         B7 ------- SDA (4P)
//         B6 ------- SCL (4P)
//
//                     4 --------- [+LED-] -- [220] --- GND
//                     5 --------- [+LED-] -- [220] --- GND
//                     6 --------- [+LED-] -- [220] --- GND
//                    16 --------- [-LED+] -- [22K] --- 5V 
//                    11 --------- [+LED-] -- [220] --- GND
//                    12 --------- [+LED-] -- [220] --- GND
//                    13 --------- [+LED-] -- [220] --- GND
//                    14 --------- [+LED-] -- [220] --- GND
//
//
//  Alternative Hardware Setup
//  ==========================
//  This might not work for everyone, but at least for some LCD
//  modules, the pullup resistors may not be required, except
//  for the one between 5V and the LCD RS line. Also, instead of
//  a bulky potentiometer for the contrast control to V0, a 2K
//  resistor between V0 and 5V may be good enough to maintain
//  good contrast.
//
// ** Pot may be replaced by a 2k resistor between V0 and GND.
//         VO -- [ 2k ] --- GND (LCD contrast control)
//        
//  * These pullup resistors to 5V might not be required.
//    Note that the RS line seems to require the pullup resistor.
//        B14 ------ RS
//         A8 ------ D4
//         A9 ------ D5
//        A10 ------ D6
//        A11 ------ D7



#include "stm32f103x8.h"  // Primary CMSIS header file
#include "STM32F103-CMSIS-I2C-lib.c"
#include "STM32F103-pause-lib.c"
int
main()
{
  uint8_t  light=0;
  I2C_init( I2C2 );


  while( 1 )
  {
    I2C_writeByte( I2C2, 1<<light, 0x3f );

    delay_us( 50e3 );
    if( ++light>7 )
      light = 0;
  }  
  return 1;
}  
