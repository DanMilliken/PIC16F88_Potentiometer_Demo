/******************************************************************************
 *
 * PIC16F88_Potentiometer_Demo
 *
 * Author: Dan Milliken
 * Date: 2014-10-27
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

#define _XTAL_FREQ  20000000

#pragma config LVP = OFF        // disable low voltage programming
#pragma config FCMEN = OFF      // disable fail safe clock monitor
#pragma config IESO = OFF       // disable internal/external oscillator switchover
#pragma config BOREN = OFF      // disable brown out reset
#pragma config PWRTE = ON       // enable power up timer
#pragma config WDTE = OFF       // disable watchdog timer
#pragma config FOSC = HS        // external crystal

// function prototypes
void UART_write(unsigned char c);
void UART_init(void);
void timer2_init(void);
void ADInit(void);
void interrupt ISR(void);
unsigned long GetSystemTime(void);

// globals
unsigned long system_time = 0;   // ms - SYSTEM RUNNING TIME LIMIT: 49 days

int main(void)
{
    unsigned long long outputTime = 0; // Time of last pot value output.
    const unsigned int outputIntervalMs = 200; // in ms, interval to output pot value.

    timer2_init(); // initialize the system time
    UART_init();   // initialize the UART module
    ADInit();      // initialize the A/D

    INTCONbits.PEIE = 1; // enable peripheral interrupts
    INTCONbits.GIE = 1;  // enable interrupts

    while(1) {
        if (GetSystemTime() > (outputTime + outputIntervalMs))
        {
            PIR1bits.ADIF = 0;  // Reset the A/D interrupt flag
            PIE1bits.ADIE = 1;  // enable the A/D interrupt
            ADCON0bits.GO_DONE = 1; // start the A/D

            outputTime = GetSystemTime();
        }
    }
}

void ADInit(void)
{
    TRISAbits.TRISA0 = 1;  // Configure RA0 as input (for potentiometer)
    ANSELbits.ANS0 = 1;    // Set RA0 to analog
    ADCON0bits.ADON = 1;   // Turn on the A/D
    ADCON0bits.CHS = 0;    // Use channel AN0 for A/D
    ADCON1bits.ADFM = 1;   // Result right justified
    ADCON1bits.ADCS2 = 0;  // 64 TOSC
    ADCON0bits.ADCS1 = 1;
    ADCON0bits.ADCS0 = 0;

    return;
}

void timer2_init(void)
{
    INTCONbits.PEIE = 1; // enable peripheral interrupts

    T2CONbits.T2CKPS1 = 0;   // Prescalar = 1:4
    T2CONbits.T2CKPS0 = 1;

    T2CONbits.TOUTPS3 = 1;   // Postscalar = 1:9
    T2CONbits.TOUTPS2 = 0;
    T2CONbits.TOUTPS1 = 0;
    T2CONbits.TOUTPS0 = 0;

    PR2 = 139;               // Period register 139

    T2CONbits.TMR2ON = 1;    // Timer2 ON

    PIR1bits.TMR2IF = 0;     // Timer2 flag clear
    PIE1bits.TMR2IE = 1;     // Timer2 interrupt enable
}

void UART_init(void)
{
    TXSTAbits.BRGH = 1; // high baud rate
    TXSTAbits.SYNC = 0; // asynchronous mode
    TXSTAbits.TX9  = 0; // 8-bit transmission
    RCSTAbits.CREN = 1; // continuous receive enable

    SPBRG = 129;        // 9600 baud @ 20MHz

    PIE1bits.RCIE  = 1;
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1; // enable transmitter

    return;
}

void UART_write(unsigned char c) {
    while (!TXSTAbits.TRMT);
    TXREG = c;

    return;
}

// Override putch called by printf
void putch(unsigned char byte)
{
    UART_write(byte);
    if ('\n' == byte)
        UART_write('\r');
    return;
}

unsigned long GetSystemTime()
{
    unsigned long TIME;
    INTCONbits.TMR0IE = 0;   // Timer0 interrupt disable
    TIME = system_time;
    INTCONbits.TMR0IE = 1;   // Timer0 interrupt enable
    return TIME;
}

void Process_AD(void)
{
    PIE1bits.ADIE = 0;  // disable the A/D interrupt
    PIR1bits.ADIF = 0;  // Reset the A/D interrupt flag
    unsigned int RESULT = ADRESH * 256 + ADRESL;
    printf("%lu  Result: %u\n", GetSystemTime(), RESULT);
    return;
}

void interrupt ISR(void)
{
    // AUSART Receive Interrupt Flag bit
    if (1 == PIR1bits.RCIF)
        UART_write(RCREG);

    // A/D Complete Interrupt Flag bit
    if (1 == PIR1bits.ADIF)
        Process_AD();

    // Timer 2 overflow interrupt
    if (1 == PIR1bits.TMR2IF)
    {
        PIR1bits.TMR2IF = 0;
        system_time = system_time + 1;
    }

    return;
}
