/*
 * lab09_pre
 * File:   lab09_pre_prg.c
 * Author: Gerardo Paz - 20173
 * Servo en RC
 * 
 * Created on April 25, 2022, 8:52 PM
 */

#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>         // registros del PIC
#include <stdint.h>

/*------------ CONSTANTES -----------------*/
#define _XTAL_FREQ 500000  // oscilador 

// potenciómetro
#define IN_MIN 0
#define IN_MAX 255

// servomotor
#define OUT_MIN 30
#define OUT_MAX 63

/*------------- VARIABLES GLOBALES ------------------*/
unsigned short CCPR = 0;

/*--------------- PROTOTIPOS DE FUNCIONES --------------*/
/*--------------- FUNCIONES DE INTERRUPCIONES -----------------*/

/*---------------- FUNCIONES PRINCIPALES ---------------*/
void clk(void){
    OSCCONbits.IRCF = 0b011;    // Tiempo
    OSCCONbits.SCS = 1;         // oscilador interno
    return;
}

void setup_adc(void){
    ADCON1bits.ADFM = 0;    // justificado izquierda
    ADCON1bits.VCFG0 = 0;   // VDD
    ADCON1bits.VCFG1 = 0;   // VSS
    
    ADCON0bits.ADCS = 0b11; // FRC
    ADCON0bits.CHS = 0;     // AN0
    ADCON0bits.ADON = 1;    // habilitar módulo ADC
    __delay_us(40);         // tiempo para que cargue el capacitor
    return;
}

void setup_int(void){
    INTCONbits.GIE = 1;     // globales
    INTCONbits.PEIE = 1;    // periféricas
    PIE1bits.ADIE = 1;      // ADC
    
    PIR1bits.ADIF = 0;      // ADC bandera
    return;
}

void timer2(void){
    PIR1bits.TMR2IF = 0;        // bandera Timer 2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // encender Timer 2
    while(!PIR1bits.TMR2IF);    // un ciclo de timer 2
    PIR1bits.TMR2IF = 0;        // bandera timer 2 luego del primer ciclo
    return;
}

void setup_pwm(void){
    CCP1CON = 0;            // apagar CCP1
    TRISCbits.TRISC2 = 1;   // RC2/CCP1 out disabaled
    PR2 = 155;              // periodo de señal
    return;
}

void setup_ccp(void){
    CCP1CONbits.P1M = 0;        // single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    
    CCPR1L = 30>>2;                 // ciclo de trabajo base
    CCP1CONbits.DC1B = 30 & 0b11;   // 1ms ancho de pulso
    return;
}

void setup(void){
    ANSEL = 0b00000001;      // AN0 
    ANSELH = 0;
    
    TRISA = 0b00000001;      // A in
    TRISC = 0;      // C out
    
    PORTA = 0;
    PORTC = 0;      // limpiar
    return;
}

/*------------ CÓDIGO PRINCIPAL ---------------*/
/*
 * x0: mínimo ADC           y0: mínimo ancho de pulso
 * x: actual ADC            y: resultado interpolación
 * x1: máximo ADC           y1: máximo ancho de pulso
 * ADRESH (8 bits) y Ancho de pulso (10 bits) 
 * y = y0 + [(y1 - y0)/(x1-x0)]*(x-x0)
 * 
 */
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)((y1-y0)/(x1-x0)))*(x-x0));
}

void __interrupt() isr (void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){
            CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR1L = (uint8_t)(CCPR>>2);    // 8 bits más significativos de CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // 2 bits más significativos de DC1B
        }
        PIR1bits.ADIF = 0;
    }
    return;
}

void main(void){
    setup();
    clk();
    setup_adc();
    setup_int();
    setup_pwm();
    setup_ccp();
    timer2();
    TRISCbits.TRISC2 = 0;   // salida PWM
    while(1){   // principal loop
        if(ADCON0bits.GO == 0)
            ADCON0bits.GO = 1;   // iniciar conversión si no hay una
    }
}

