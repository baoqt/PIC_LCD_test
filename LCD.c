/*
 * File:   LCD.c
 * Author: BAO
 *
 * Created on March 21, 2019, 9:54 AM
 */
// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits->Oscillator not enabled
#pragma config RSTOSC = HFINT1    // Power-up default value for COSC bits->HFINTOSC
#pragma config CLKOUTEN = OFF    // Clock Out Enable bit->CLKOUT function is disabled; I/O or oscillator function on OSC2
#pragma config CSWEN = ON    // Clock Switch Enable bit->Writing to NOSC and NDIV is allowed
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config MCLRE = ON    // Master Clear Enable bit->MCLR/VPP pin function is MCLR; Weak pull-up enabled
#pragma config PWRTE = OFF    // Power-up Timer Enable bit->PWRT disabled
#pragma config WDTE = OFF    // Watchdog Timer Enable bits->WDT disabled; SWDTEN is ignored
#pragma config LPBOREN = OFF    // Low-power BOR enable bit->ULPBOR disabled
#pragma config BOREN = ON    // Brown-out Reset Enable bits->Brown-out Reset enabled, SBOREN bit ignored
#pragma config BORV = LOW    // Brown-out Reset Voltage selection bit->Brown-out voltage (Vbor) set to 2.45V
#pragma config PPS1WAY = ON    // PPSLOCK bit One-Way Set Enable bit->The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a Reset
#pragma config DEBUG = OFF    // Debugger enable bit->Background debugger disabled

// CONFIG3
#pragma config WRT = OFF    // User NVM self-write protection bits->Write protection off
#pragma config LVP = ON    // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.

// CONFIG4
#pragma config CP = OFF    // User NVM Program Memory Code Protection bit->User NVM code protection disabled
#pragma config CPD = OFF    // Data NVM Memory Code Protection bit->Data NVM code protection disabled

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define displayOn 0xAF
#define displayOff 0xAE
#define ADCSelectNormal 0xA0
#define ADCSelectReverse 0xA1
#define displaySelectNormal 0xA7
#define displaySelectReverse 0xA1
#define allPointsOn 0xA5
#define normalPoints 0xA4
#define biasSet 0xA2
#define modeSelectNormal 0xC0
#define controlSet 0x2F
#define ratioSet 0x21
#define volumeModeSet 0x81
#define volumeRegSet 0x20

uint16_t count = 0;
unsigned char packet;
uint8_t ADCValue = 0;
uint8_t packetIndex = 0;
uint8_t pollingIndex = 0;
uint8_t bitIndex = 0;
bool transmitFlag = 0;
bool receiveFlag = 0;

void command(unsigned char command);
void data(unsigned char data);

void LCD_init(void);
void LCD_clear(void);
void LCD_goto(uint8_t page, uint8_t offset);
void LCD_write(unsigned char character);

void ADC_read(void);

void main(void) {
    OSCCON1 = 0x60; // HFINTOSC   
    OSCFRQ = 0x03;  // HFFRQ 4_MHz; 

    TRISA = 0b11011001;     // RA1, RA2 outputs (LCD)
                            // RA4 input, RA5 output (ACD)
    TRISC = 0b11001000;     // RC0, RC1, RC2 outputs RC5 activity LED(LCD)
                            // RC4 outputs (ADC)
    ANSELA = 0b00000000;    // Disable analog input
    ANSELC = 0b00000000;
    
    GIE = 1;                // Enable global interrupts
    PEIE = 1;               // Enable peripheral interrupts
    TMR0IE = 1;             // Enable overflow interrupt
    
    T0EN = 1;               // Enable timer0
    TMR0 = 0;               // 8 bit timer
    T0CON1 ^= 0b01000000;   // FOSC/4, 1:1 prescaler
    TMR0H = 0b10111111;     // Clock period
    
    LATA1 = 1;              // LCD chip select N
    LATA2 = 1;              // LCD reset N
    LATC1 = 1;              // LCD clock high
    LATC2 = 1;              // LCD data high
    LATC5 = 0;              // LED off
    
    LATC4 = 1;              // ADC clock high
    LATA5 = 1;              // ADC off
    
    LCD_init();
    LCD_clear();
    
    while (1)
    {
        ADC_read();
    
        LCD_goto(1, 0);
        LCD_write('A');
        LCD_write('D');
        LCD_write('C');
        LCD_write(' ');
        LCD_write(':');
        LCD_write(' ');
        LCD_write('0');
        LCD_write('x');
        LCD_write(((ADCValue >> 4) < 0x0A) ? (ADCValue >> 4) + 0x30 : (ADCValue >> 4) + 0x37);
        LCD_write(((ADCValue & 0x0F) < 0x0A) ? (ADCValue & 0x0F) + 0x30 : (ADCValue & 0x0F) + 0x37);
    }
    
    return;
}

void command(unsigned char command)
{
    LATA1 = 0;                  // Enable LCD
    LATC0 = 0;                  // Command
    
    packet = command;           // Pass command to global for interrupt
    
    transmitFlag = 1;           // Enable transmit interrupt
    while (transmitFlag);      // Wait for transmission to finish
    
    LATA1 = 0;                  // Disable LCD
    
    return;
}

void data(unsigned char data)
{
    LATA1 = 0;                  // LATA1 : LCD chip select
    LATC0 = 1;                  // LATA0 : LCD command (1), data (0)
    
    packet = data;
    
    transmitFlag = 1;
    while (transmitFlag);
    
    LATA1 = 0;
    
    return;
}

void LCD_init(void)
{
    command(displayOff);
    command(ADCSelectNormal);
    command(modeSelectNormal);
    command(biasSet);
    command(controlSet);
    command(ratioSet);
    command(volumeModeSet);
    command(volumeRegSet);
    command(displayOn);
    
    return;
}

void ADC_read(void)
{
    receiveFlag = 1;
    
    while (receiveFlag);
    
    return;
}

void LCD_clear(void)
{
    uint8_t n = 0;
    int8_t j = 0;
    
    for (j = 3; j >= 0; j--)
    {
        command(0xB0 + j);              // Set page address to 0
        command(0x10);                  // Set cursor address to 0
        command(0x00);
        
        for (n = 0; n < 132; n++)
        {
            data(0x00);
        }
    }
    
    return;
}

void LCD_goto(uint8_t page, uint8_t offset)
{
    command(0xB0 + page);
    command(0x10 + (offset & 0xF));
    command(0x00 + (offset & 0x0F));
    
    return;
}

void LCD_write(unsigned char character)
{
    switch (character) {
        case 'A' :
        {
            data(0x3F);
            data(0x48);
            data(0x48);
            data(0x48);
            data(0x3F);
            data(0x00);
            break;
        }
        case 'B' :
        {
            data(0x7F);
            data(0x49);
            data(0x49);
            data(0x49);
            data(0x36);
            data(0x00);
            break;
        }
        case 'C' :
        {
            data(0x3E);
            data(0x41);
            data(0x41);
            data(0x41);
            data(0x22);
            data(0x00);
            break;
        }
        case 'D' :
        {
            data(0x7F);
            data(0x41);
            data(0x41);
            data(0x41);
            data(0x3E);
            data(0x00);
            break;
        }
        case 'E' :
        {
            data(0x7F);
            data(0x49);
            data(0x49);
            data(0x41);
            data(0x41);
            data(0x00);
            break;
        }
        case 'F' :
        {
            data(0x7F);
            data(0x50);
            data(0x50);
            data(0x40);
            data(0x40);
            data(0x00);
            break;
        }
        case 'G' :
        {
            data(0x3E);
            data(0x41);
            data(0x49);
            data(0x49);
            data(0x2E);
            data(0x00);
            break;
        }
        case 'H' :
        {
            data(0x7F);
            data(0x08);
            data(0x08);
            data(0x08);
            data(0x7F);
            data(0x00);
            break;
        }
        case 'I' :
        {
            data(0x00);
            data(0x41);
            data(0x7F);
            data(0x41);
            data(0x00);
            data(0x00);
            break;
        }
        case 'J' :
        {
            data(0x06);
            data(0x41);
            data(0x41);
            data(0x41);
            data(0x7E);
            data(0x00);
            break;
        }
        case 'K' :
        {
            data(0x7F);
            data(0x8);
            data(0x14);
            data(0x22);
            data(0x41);
            data(0x00);
            break;
        }
        case 'L' :
        {
            data(0x7F);
            data(0x01);
            data(0x01);
            data(0x01);
            data(0x01);
            data(0x00);
            break;
        }
        case 'M' :
        {
            data(0x7F);
            data(0x20);
            data(0x10);
            data(0x20);
            data(0x7F);
            data(0x00);
            break;
        }
        case 'N' :
        {
            data(0x7F);
            data(0x10);
            data(0x08);
            data(0x04);
            data(0x7F);
            data(0x00);
            break;
        }
        case 'O' :
        {
            data(0x3E);
            data(0x41);
            data(0x41);
            data(0x41);
            data(0x3E);
            data(0x00);
            break;
        }
        case 'P' :
        {
            data(0x7F);
            data(0x48);
            data(0x48);
            data(0x48);
            data(0x30);
            data(0x00);
            break;
        }
        case 'Q' :
        {
            data(0x3E);
            data(0x41);
            data(0x41);
            data(0x42);
            data(0x3D);
            data(0x00);
            break;
        }
        case 'R' :
        {
            data(0x7F);
            data(0x48);
            data(0x48);
            data(0x4C);
            data(0x33);
            data(0x00);
            break;
        }
        case 'S' :
        {
            data(0x32);
            data(0x49);
            data(0x49);
            data(0x49);
            data(0x26);
            data(0x00);
            break;
        }
        case 'T' :
        {
            data(0x40);
            data(0x40);
            data(0x7F);
            data(0x40);
            data(0x40);
            data(0x00);
            break;
        }
        case 'U' :
        {
            data(0x7E);
            data(0x01);
            data(0x01);
            data(0x01);
            data(0x7E);
            data(0x00);
            break;
        }
        case 'V' :
        {
            data(0x70);
            data(0x0C);
            data(0x03);
            data(0x0C);
            data(0x70);
            data(0x00);
            break;
        }
        case 'W' :
        {
            data(0x7C);
            data(0x03);
            data(0x1C);
            data(0x03);
            data(0x7C);
            data(0x00);
            break;
        }
        case 'X' :
        {
            data(0x63);
            data(0x14);
            data(0x08);
            data(0x14);
            data(0x63);
            data(0x00);
            break;
        }
        case 'x' :
        {
            data(0x09);
            data(0x12);
            data(0x0E);
            data(0x09);
            data(0x12);
            data(0x00);
            break;
        }
        case 'Y' :
        {
            data(0x60);
            data(0x10);
            data(0x0F);
            data(0x10);
            data(0x60);
            data(0x00);
            break;
        }
        case 'Z' :
        {
            data(0x43);
            data(0x45);
            data(0x49);
            data(0x51);
            data(0x61);
            data(0x00);
            break;
        }
        case '0' :
        {
            data(0x3E);
            data(0x51);
            data(0x49);
            data(0x45);
            data(0x3E);
            data(0x00);
            break;
        }
        case '1' :
        {
            data(0x00);
            data(0x21);
            data(0x7F);
            data(0x01);
            data(0x00);
            data(0x00);
            break;
        }
        case '2' :
        {
            data(0x33);
            data(0x45);
            data(0x49);
            data(0x49);
            data(0x31);
            data(0x00);
            break;
        }
        case '3' :
        {
            data(0x22);
            data(0x41);
            data(0x49);
            data(0x49);
            data(0x36);
            data(0x00);
            break;
        }
        case '4' :
        {
            data(0x78);
            data(0x08);
            data(0x08);
            data(0x08);
            data(0x7F);
            data(0x00);
            break;
        }
        case '5' :
        {
            data(0x7A);
            data(0x49);
            data(0x49);
            data(0x49);
            data(0x46);
            data(0x00);
            break;
        }
        case '6' :
        {
            data(0x3E);
            data(0x49);
            data(0x49);
            data(0x49);
            data(0x26);
            data(0x00);
            break;
        }
        case '7' :
        {
            data(0x60);
            data(0x41);
            data(0x46);
            data(0x58);
            data(0x60);
            data(0x00);
            break;
        }
        case '8' :
        {
            data(0x36);
            data(0x49);
            data(0x49);
            data(0x49);
            data(0x36);
            data(0x00);
            break;
        }
        case '9' :
        {
            data(0x32);
            data(0x49);
            data(0x49);
            data(0x49);
            data(0x3E);
            data(0x00);
            break;
        }
        case ' ' :
        {
            data(0x00);
            data(0x00);
            data(0x00);
            data(0x00);
            data(0x00);
            data(0x00);
            break;
        }
        case ':' :
        {
            data(0x00);
            data(0x00);
            data(0x24);
            data(0x00);
            data(0x00);
            data(0x00);
        }
    }
}

void __interrupt() displayClock(void)
{
    if (TMR0IF && TMR0IE && transmitFlag) 
    {
        if (packetIndex < 8)
        {
            LATC1 = ~LATC1;     // Toggle clock
            LATC5 = ~LATC5;
            
            if (LATC1 == 0)     // Clock low
            {                   // Push bit onto line
                LATC2 = (packet & 0b10000000) && 1;
            }                   // LATC2 : LCD data line
            else                // Clock high
            {
                packet <<= 1;   // Shift for next bit
                packetIndex++;
            }
        }
        else                    // Transmission finished
        {
            packetIndex = 0;
            transmitFlag = 0;
            LATC5 = 0;
        }
        
        TMR0IF = 0;             // Reset interrupt flag
    }
    else if (TMR0IF && TMR0IE && receiveFlag)
    {
        if (pollingIndex == 7)
        {
            LATC4 = 0;          // LATC4 : ACD clock
        }
        else if (pollingIndex == 11)
        {
            if (bitIndex == 1)
            {
                ADCValue = 0;
            }
            else if ((bitIndex > 1) && (bitIndex < 10))
            {
                ADCValue += RA4 << (9 - bitIndex);
            }
        }
        else if (pollingIndex == 15)
        {
            LATC4 = 1;          // ADC clock high
            
            if (bitIndex == 0)
            {
                LATA5 = 0;      // LATA5 : ADC chip select (negative)
            }
            else if (bitIndex == 11)
            {                   // ADC value received
                LATA5 = 1;      // Disable chip
                receiveFlag = 0;
            }
            
            bitIndex = (bitIndex > 10) ? 0 : bitIndex + 1;
        }
        
        pollingIndex = (pollingIndex > 14) ? 0 : pollingIndex + 1;
        
        TMR0IF = 0;
    }
    
    return;
}