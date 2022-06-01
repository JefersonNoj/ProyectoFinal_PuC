/* 
 * File:   mainslave.c
 * Author: Jeferson Noj
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
#define IN_MIN 0            // Valor minimo de entrada del potenciometro
#define IN_MAX 255          // Valor máximo de entrada del potenciometro
#define OUT_MIN 60          // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 130        // Valor máximo de ancho de pulso de señal PWM

// VARIABLES -------------------------------------------------------------------
char val_temp = 0, POT1_slave = 0, POT2_slave = 0, SERIAL = 0, POT_compu = 0;;
unsigned short CCPR = 0, CCPR_2;   // Variable para almacenar ancho de pulso al hacer la interpolación lineal
uint8_t cont = 0, modo = 0, selector = 0, POTc = 0, flag = 0;

// PROTOTIPO DE FUNCIONES ------------------------------------------------------
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

// INTERRUPCIONES --------------------------------------------------------------
void __interrupt() isr (void){
    if (PIR1bits.SSPIF){        // Verificar interrupción del SPI
           
        val_temp = SSPBUF;      // Recuperar valor de la comunicación SPI
        
            if (cont == 0){     // Evaluar bandera para determinar si se quiere controlar el primer servo del slave
                CCPR = map(val_temp, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardar los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardar los 2 bits menos significativos en DC1B
                cont = 1;
            }

            else if (cont == 1){    // Evaluar bandera para determinar si se quiere controlar el segundo servo del slave
                CCPR_2 = map(val_temp, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Obtener valor del ancho de pulso
                CCPR2L = (uint8_t)(CCPR_2>>2);      // Guardar los 8 bits mas significativos en CPR2L
                CCP2CONbits.DC2B0 = CCPR_2 & 0b01;  // Guardar los 2 bits menos significativos en DC2B
                CCP2CONbits.DC2B0 = (CCPR_2 & 0b10)>>1;
                cont = 0;
            }
        
        PIR1bits.SSPIF = 0;     // Limpiar bandera de interrupción
    }
    
    if(PIR1bits.RCIF){
        SERIAL = RCREG;         // Guardar valor de la comunicación serial con la PC
    }
    
    if (INTCONbits.RBIF){       // Verificar si ocurrió interrupción del PORTB
        if (!PORTBbits.RB0){    // Evaluar si se presionó el boton en RB0
            modo++;             // Cambiar de modo
            if (modo > 2)       // Reiniciar modo si es mayor a 2
                modo = 0;
        }
        INTCONbits.RBIF = 0;    // Limpiar bandera de interrupción del PORTB
    }
    
    return;
}

// CICLO PRINCIPAL -------------------------------------------------------------
void main(void) {
    setup();
    while(1){        
        // Envio y recepcion de datos en maestro
        //PORTE = modo;
        if (SERIAL == 0 || SERIAL == 1){    // Verificar si se quieren controlar los servos del máster
            flag = 0;                       // Desactviar bandera para control

        }
        else if (SERIAL == 2 || SERIAL == 3){   // Verificar si se quieren controlar los servos del slave
            flag = 1;                           // Activar bandera para control
            selector = SERIAL;                  // Asignar el serial (boton) a la variable que selecciona los servos
        }
        else
            POTc = SERIAL;                  // Asigar valor de la perilla a la variable POTc
                
        if (flag){
            switch(selector){           // Evaluar cuál de los dos servos del máster se esta moviendo con la interfaz
                case 2:        // Para el servo 1...
                    CCPR1L = (POTc>>2);                 // Guardar los 8 bits mas significativos en CPR1L
                    CCP1CONbits.DC1B = POTc & 0b11;;    // Guardar los 2 bits menos significativos en DC1B
                    break;
                case 3:        // Para el servo 2... 
                    CCPR2L = (POTc>>2);                 // Guardar los 8 bits mas significativos en CPR2L
                    CCP2CONbits.DC2B0 = POTc & 0b01;    // Guardar los 2 bits menos significativos en DC2B
                    CCP2CONbits.DC2B0 = (POTc & 0b10)>>1;
                    break;
            }
        }
    }
    return;
}
// CONFIGURACION ---------------------------------------------------------------
void setup(void){
    ANSEL = 0;                  // I/O digitales
    ANSELH = 0;
    
    TRISA = 0b00100000;         // RA5 (SS) como entrada
    PORTA = 0;
    TRISB = 0b00000001;         // RB0 como entrada
    PORTB = 0; 
    TRISE = 0;
    PORTE = 0; 
    
    OPTION_REGbits.nRBPU = 0;   // Habilitar resistencias pull-up
    WPUB = 0b00000001;          // Habilitar pull-up para RB0, RB1 y RB2
    IOCB = 0b00000001;          // Habilitar interrupciones On_change para RB0, RB1 y RB2
    
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
    TRISCbits.TRISC4 = 1;       // SDI y SCK entradas, SD0 como salida
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC5 = 0;
    PORTC = 0;

    // SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;   // SPI Esclavo, SS hablitado
    SSPCONbits.CKP = 0;         // Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de SPI
    // SSPSTAT<7:6>
    SSPSTATbits.CKE = 1;        // Dato enviado cada flanco de subida
    SSPSTATbits.SMP = 0;        // Dato al final del pulso de reloj

    //Configuración de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitar comunicación
    RCSTAbits.RX9 = 0;          // Utilizar solo 8 bits para recepción 
    
    TXSTAbits.TXEN = 1;       // Habilitar transmisor
    RCSTAbits.CREN = 1;         // Habilitar receptor
    
    PIR1bits.SSPIF = 0;         // Limpiar bandera de SPI
    PIE1bits.SSPIE = 1;         // Habilitar int. de SPI
    INTCONbits.PEIE = 1;        // Habilitar int. de periféricos
    INTCONbits.GIE = 1;         // Habilitar int. globales
    INTCONbits.RBIE = 1;        // Habilitar int. del PORTB
    INTCONbits.RBIF = 0;        // Limpiar bandera del PORTB
    PIE1bits.RCIE = 1;          // Habilitar Interrupciones de recepción

    
    return;
}

// Función de mapeo (interpolación)
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}