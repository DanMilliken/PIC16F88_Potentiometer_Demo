/******************************************************************************
 *
 * PIC16F88_Potentiometer_Demo
 *
 * Author: Dan Milliken
 * Date: 2014-12-16
 * Project: PIC16F88_Potentiometer_Demo
 * Description: Demonstrates using the A/D module on the PIC16F88
 * Microcontroller with a MAX232 IC to produce RS232 output to a PC.
 *
 * License: Licensed under the Creative Commons Attribution-ShareAlike 4.0
 * International License (http://creativecommons.org/licenses/by-sa/4.0/)
 *
*******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include <stdio.h>         /* C standard IO */

#define _XTAL_FREQ  4000000

#pragma config LVP = OFF        // disable low voltage programming
#pragma config FCMEN = OFF      // disable fail safe clock monitor
#pragma config IESO = OFF       // disable internal/external oscillator switchover
#pragma config BOREN = OFF      // disable brown out reset
#pragma config PWRTE = ON       // enable power up timer
#pragma config WDTE = OFF       // disable watchdog timer
#pragma config FOSC = INTOSCIO  // Internal oscillator

// function prototypes
void mod_init_uart(void);
void mod_init_ad(void);
void interrupt isr(void);

int main(void)
{
    // Set all I/O pins to output
    TRISA = 0;
    TRISB = 0;

    // Set the clock
    #assert _XTAL_FREQ == 4000000  // Make sure _XTAL_FREQ is set correctly
    OSCCONbits.IRCF = 0b110;       // Set internal RC oscillator to 4 MHz
    while(!OSCCONbits.IOFS);       // Wait for frequency to stabalize

    mod_init_uart();     // initialize the UART module
    mod_init_ad();       // initialize the A/D

    INTCONbits.PEIE = 1; // enable peripheral interrupts
    INTCONbits.GIE = 1;  // enable interrupts

    printf("*** A/D demo system startup ***\n");
    
    while(1)
    {
        PIR1bits.ADIF = 0;      // Reset the A/D interrupt flag
        PIE1bits.ADIE = 1;      // enable the A/D interrupt
        ADCON0bits.GO_DONE = 1; // start the A/D
        __delay_ms(25);
    }
}

void mod_init_ad(void)
{
    TRISA |= 0b00000001;   // Configure RA0 as input (for potentiometer)
    ANSELbits.ANS0 = 1;    // Set RA0 to analog
    ADCON0bits.ADON = 1;   // Turn on the A/D
    ADCON0bits.CHS = 0;    // Use channel AN0 for A/D
    ADCON1bits.ADFM = 1;   // Result right justified
    ADCON1bits.ADCS2 = 0;  // 8 * TOSC = 2us
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;

    return;
}

void mod_init_uart(void)
{
    TRISB |= 0b00100100; // TRISB<5,2> as input. Others as output.
    TXSTAbits.BRGH = 1;  // high baud rate
    TXSTAbits.SYNC = 0;  // asynchronous mode
    TXSTAbits.TX9  = 0;  // 8-bit transmission
    RCSTAbits.CREN = 1;  // continuous receive enable

    #assert _XTAL_FREQ == 4000000  // SPBRG is based on 4 MHz clock
    SPBRG = 25;                    // 9600 baud @ 4MHz with BRGH = 1

    PIE1bits.RCIE  = 1;
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1; // enable transmitter

    return;
}

// Override putch called by printf
void putch(unsigned char byte)
{
    while (!TXSTAbits.TRMT);
    TXREG = byte;
    if ('\n' == byte)
    {
        while (!TXSTAbits.TRMT);
        TXREG = '\r';
    }

    return;
}

void process_ad(void)
{
    PIE1bits.ADIE = 0;  // disable the A/D interrupt
    PIR1bits.ADIF = 0;  // Reset the A/D interrupt flag
    unsigned int ad_result = ADRESH * 256 + ADRESL;
    printf("A/D Result: %u\n", ad_result);
    return;
}

void interrupt isr(void)
{
    // A/D Complete Interrupt Flag bit
    if (1 == PIR1bits.ADIF)
        process_ad();

    return;
}
