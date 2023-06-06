//  STM32F103-CMSIS-I2C-LED-Scanner
//
//  Simple I2C demonstration that uses an IC2 LCD driver module based on the PCF8574 IC.
//  Instead of driving an LCD display, the module will drive 8 LEDs based on I2C input.
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//  Mike Shegedin, 05/2023
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


#include <stdlib.h>
#include "stm32f103x8.h"  // Primary CMSIS header file


//  I2C1_init
//    Initialize theGPIO B6 and B7 as alternate function pins.
//    Set the I2C timing/clock registers.
//    Enable the I2C peripheral.
void
I2C1_init( void )
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | 
                  RCC_APB2ENR_AFIOEN ;  // Enable GPIO Port B and Alt. Func. Clocks
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // Enable I2C Clock

  // Set CNF and MODE bits for B6/B7 to 0b11 for Alternate Function / High Speed
  GPIOB->CRL |= ( 0b11 << GPIO_CRL_CNF6_Pos  |
                  0b11 << GPIO_CRL_CNF7_Pos  |
                  0b11 << GPIO_CRL_MODE6_Pos |
                  0b11 << GPIO_CRL_MODE7_Pos );

  I2C1->TRISE |= 0x02;                      // Set the TRISE time
  I2C1->CR1   |=  I2C_CR1_SWRST;            // Set I2C reset bit
  I2C1->CR1   &= ~I2C_CR1_SWRST;            // Clear I2C reset bit
  I2C1->CR2   |= 0x08 << I2C_CR2_FREQ_Pos;  // Set to APB1 Peripheral Clock freq. in MHz (8 MHz = 8)
  I2C1->TRISE  = 0x09;                      // Set to APB1 Peripheral Clock Freq. in MHz + 1 (8+1 = 9)
  I2C1->CCR    = 0x28;                      // CCR = 5 us * APB1 Peripheral clock speed (8E6) = 40 (0x28)
                                            // F/S and DUTY bits are 0 as not using "fast mode" i2c.
  I2C1->OAR1   = 0x4000;                    // Probably not needed when you are the master
  I2C1->CR1   |= I2C_CR1_PE;                // Turn on I2C peripheral
}


//  delay_ms
//  Input: uint16_t d
//  Causes a delay for approx d ms
//  ** Only works at clock speed of 8 MHz!
void
delay_ms( uint16_t d )
{
  for( uint16_t x=0; x<d ; x++ )
  {
    for( uint16_t y=0; y<800; y++) ; 
  }
}


void
I2C_start( void )
{
  I2C1->CR1 |= I2C_CR1_START;             // Set START bit in CR1 which initializes I2C sequence and cause the
                                          // I2C interface to enter master mode.
  while( !( I2C1->SR1 & I2C_SR1_SB )) ;   // Wait for SB bit in SR1 to be set indicating start occurred [EV5:1]
}


void
I2C_address( uint8_t address )
{
  I2C1->DR = address<<1;                  // Load address into DR and send it out [EV5:2]
                                          // Note that the address is shifted 1 bit to the left, to make room
                                          // for the read/write bit in the lsb position.
  while( !( I2C1->SR1 & I2C_SR1_ADDR )) ; // Wait for ADDR bit in SR1 to be set indicating the address was
                                          // transferred [EV5:3] [EV6:1].
  uint8_t temp = I2C1->SR1 | I2C1->SR2;   // Read SR1 and SR2 to clear ADDR bit [EV6:2]
}


void
I2C_write( uint8_t data )
{
  while( !( I2C1->SR1 & I2C_SR1_TXE )) ;  // Wait for TXE bit in SR1 to be set indicating DR is empty [EV8_1:1]
  I2C1->DR = data;                        // Load data to write, and send it out [EV8_1:2]
  while( !( I2C1->SR1 & I2C_SR1_BTF )) ;  // Wait for BTF bit in SR1 to be set indicating the byte was
                                          // transferred [EV8_2:1]
}


void
I2C_stop( void )
{
  I2C1->CR1 |= I2C_CR1_STOP;              // Set STOP bit in CR1 indicating end of I2C transmission and
                                          // return to slave mode [EV8_2:2]
}


int
main()
{
  uint8_t  light=0;
  I2C1_init();


  while( 1 )
  {
    I2C_start();
    I2C_address( 0x3f );
    I2C_write( 1<<light );
    I2C_stop();

    delay_ms( 100 );
    if( ++light>7 )
      light = 0;
  }  
  return 1;
}  
