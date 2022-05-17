/* 
 * File:   mainL12.c
 * Author: Jeferson Noj
 *
 * Created on 15 de mayo de 2022, 09:16 PM
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

// VARIABLES -------------------------------------------------------------------
uint8_t bandera;
uint8_t address = 0, POT = 0;

// PROTOTIPO DE FUNCIONES ------------------------------------------------------
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

// INTERRUPCIONES --------------------------------------------------------------
void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // Verificar si ocurrió interrupción del ADC
        if(ADCON0bits.CHS == 0)     // Verificar que se esta leyendo el AN0
            POT = ADRESH;           // Guardar el valor de la conversión
            PORTD = ADRESH;
        PIR1bits.ADIF = 0;          // Limpiar bandera de interrupción del ADC
    }
    
    else if (INTCONbits.RBIF){      // Verificar si ocurrió interrupción del PORTB
        bandera = 0;                // Limpiar bandera del sleep
        if (!PORTBbits.RB1)                 // Verificar si se presionó el boton de escritura
            write_EEPROM(address, POT);     // Escribir en la EEPROM
        else if (!PORTBbits.RB2)            // Verificar si se presionó el boton de lectura
            PORTC = read_EEPROM(address);   // Leer de la EEPROM
        INTCONbits.RBIF = 0;        // Limpiar bandera de interrupción del PORTB
    }
    
    return;
}
// CICLO PRINCIPAL -------------------------------------------------------------
void main(void) {
    setup();
    while(!bandera){                // Ejecutar mientras la bandera sea 0
        if(ADCON0bits.GO == 0)      // Verificar que no hay proceso de conversión
            ADCON0bits.GO = 1;      // Iniciar proceso de conversión
        if(!PORTBbits.RB0)          // Verificar si se presionó el boton de sleep
            bandera = 1;            // Activar bandera del sleep
    }
    SLEEP();                        // PIC en modo sleep
    return;
}

// CONFIGURACION ---------------------------------------------------------------
void setup(void){
    ANSEL = 0b1;                // AN0 como entrada analógica
    ANSELH = 0;                 // I/O digitales para el PORTB
    
    TRISB = 0b111;              // RB0 - RB2 como entradas
    PORTB = 0;                  // Limpiar PORTB
    
    TRISA = 0b1;                // RA0 como entrada
    TRISC = 0;                  // PORTC como salida
    TRISD = 0;                  // PORTD como salida
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    
    OPTION_REGbits.nRBPU = 0;   // Habilitar resistencias pull-up
    WPUB = 0b00000111;          // Habilitar pull-up para RB0, RB1 y RB2
    IOCB = 0b00000110;          // Habilitar interrupciones On_change para RB1 y RB2
    
    // Reloj interno
    OSCCONbits.IRCF = 0b110;    // FOSC = 4 MHz 
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionar el canal AN0
    ADCON1bits.ADFM = 0;        // Justificar a la izquierda
    ADCON0bits.ADON = 1;        // Habilitar modulo ADC
    __delay_us(40);             // Tiempo de muestreo
    
    INTCONbits.RBIE = 1;        // Habilitar int. del PORTB
    INTCONbits.RBIF = 0;        // Limpiar bandera del PORTB
    INTCONbits.GIE = 1;         // Habilitar int. globales
    INTCONbits.PEIE = 1;        // Habilitar int. de perifericos
    PIE1bits.ADIE = 1;          // Habilitar interrupción del ADC
    PIR1bits.ADIF = 0;          // Limpiar bandera del ADC
}

uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}