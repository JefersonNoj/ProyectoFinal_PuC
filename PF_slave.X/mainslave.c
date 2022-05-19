/* 
 * File:   mainslave.c
 * Author: usuario
 *
 * Created on 15 de mayo de 2022, 07:37 PM
 */

// CONFIG1
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

#include <xc.h>
#include <stdint.h>

// CONSTANTES ------------------------------------------------------------------
#define _XTAL_FREQ 1000000
#define FLAG_SPI 0xFF
#define IN_MIN 0            // Valor minimo de entrada del potenciometro
#define IN_MAX 255          // Valor máximo de entrada del potenciometro
#define OUT_MIN 61          // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 126         // Valor máximo de ancho de pulso de señal PWM

// VARIABLES -------------------------------------------------------------------
char val_temp = 0;
char POT1_slave = 0, POT2_slave = 0;
unsigned short CCPR = 0, CCPR_2;   // Variable para almacenar ancho de pulso al hacer la interpolación lineal
uint8_t cont = 0;

// PROTOTIPO DE FUNCIONES ------------------------------------------------------
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

// INTERRUPCIONES --------------------------------------------------------------
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){
           
        val_temp = SSPBUF;
        
        if (cont == 0){
            CCPR = map(val_temp, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
            CCPR1L = (uint8_t)(CCPR>>2);    // Guardar los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11; // Guardar los 2 bits menos significativos en DC1B
            cont = 1;
        }
            
        else if (cont == 1){
            CCPR_2 = map(val_temp, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Obtener valor del ancho de pulso
            CCPR2L = (uint8_t)(CCPR_2>>2);      // Guardar los 8 bits mas significativos en CPR2L
            CCP2CONbits.DC2B0 = CCPR_2 & 0b01;  // Guardar los 2 bits menos significativos en DC2B
            CCP2CONbits.DC2B0 = (CCPR_2 & 0b10)>>1;
            cont = 0;
        }
        
        PIR1bits.SSPIF = 0;             // Limpiamos bandera de interrupción
    }
    
    return;
}

// CICLO PRINCIPAL -------------------------------------------------------------
void main(void) {
    setup();
    while(1){        
        // Envio y recepcion de datos en maestro
        PORTB = CCPR1L;
        PORTD = CCPR2L;
    }
    return;
}
// CONFIGURACION ---------------------------------------------------------------
void setup(void){
    ANSEL = 0;                  // I/O digitales
    ANSELH = 0;
    
    TRISA = 0b00100000;         // RA5 (SS) como entrada
    PORTA = 0;
    TRISB = 0;
    PORTB = 0; 
    TRISD = 0;
    PORTD = 0; 
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitar salida de CCP1
    TRISCbits.TRISC1 = 1;       // Deshabilitar salida de CCP2
    PR2 = 61;                   // Perido de 4 ms
    
    // CCP1
    CCP1CON = 0;                // Apagar CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    CCPR1L = 63>>2;
    CCP1CONbits.DC1B = 63 & 0b11;  
    
    // CCP2
    CCP2CON = 0;                // Apagar CCP2
    CCP2CONbits.CCP2M = 0b1100; // PWM
    CCPR2L = 61>>2;
    CCP2CONbits.DC2B0 = 61 & 0b01;
    CCP2CONbits.DC2B1 = (61 & 0b10)>>1;
    
    // TMR2
    PIR1bits.TMR2IF = 0;        // Limpiar bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encender TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiar bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitar salida de PWM para CCP1
    TRISCbits.TRISC1 = 0;       // Habilitar salida de PWM para CCP2
    
    // Configuracion de SPI
    // Configs del esclavo
    TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
    PORTC = 0;

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // SPI Esclavo, SS hablitado
    SSPCONbits.CKP = 0;         // Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // Dato al final del pulso de reloj

    PIR1bits.SSPIF = 0;         // Limpiar bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitar int. de SPI
    INTCONbits.PEIE = 1;        // Habilitar int. de periféricos
    INTCONbits.GIE = 1;         // Habilitar int. globales
    
    return;
}

// Función de mapeo (interpolación)
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}