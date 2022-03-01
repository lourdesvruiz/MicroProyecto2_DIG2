/*
 * File:   main.c
 * Author: Lourdes Ruiz
 *
 * Created on Jan 30, 2022, 18:50 PM
 * 
 * Descripcion: 
 */


// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <pic16f887.h>
#include "LCD.h"
#include "I2C.h"
#include "UART.h"


// Define la frecuencia para poder usar los delays
#define _XTAL_FREQ 8000000

/*
 * Constantes
 */

/*
 * Variables 
 */

uint8_t contador;
uint8_t centena, decena, unidad;
uint8_t z;
uint8_t dato;
uint8_t semaforo;
/*
 * Prototipos de funciones
 */
void setup(void);
void conversion_volt(void);
void division(uint8_t counter);
void string2(char *str);
void give(char bit_cadena);

/*
 * Interrupcion 
 */

void __interrupt() isr (void)
{
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            //__delay_us(7);
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            //__delay_us(2);
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepción se complete
            contador = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){ //Escritura
            z = SSPBUF;
            BF = 0;
            SSPBUF =  semaforo;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
        
        PIR1bits.SSPIF = 0;    
    }
}


/*
 * Codigo Principal
 */

void main (void){
    
    setup();
    //unsigned int a;
    //Se inicializa el LCD
    
    
    while(1){
       //Cambio de canales de ADC
        PORTB = ~PORTB;
       __delay_ms(500);
        
        if(contador == 0){
           PORTA = 1; 
           // __delay_ms(100);
           
            
        }
        
        if(contador == 1){
            PORTA = 2;
            //__delay_ms(100);
           
        }
        
        if(contador == 2){
            PORTA = 4;
            //__delay_ms(100);
            
        }
        
 
  }
  return;  
}


/*
 * Funciones
 */

void setup(void){
    
      //Configuracion de oscilador
    OSCCONbits.IRCF = 0b0111; //8MHz
    OSCCONbits.SCS = 1; //ocsilador interno
    
    // Configuraciones de entradas y salidas 
    ANSEL = 0;  //RE0 y RE1 como entradas analógicas para ADC 
    ANSELH = 0;
    
    TRISA = 0;
    TRISB = 0;
    
    //valores iniciales
    PORTB = 0;
    PORTA = 0;
   
    I2C_Slave_Init(0x50);
   
    return;
}


void give(char bit_cadena)
{
    //while(TXSTAbits.TRMT ==0);
    while (PIR1bits.TXIF ==0); //se espera a que el registro del transmisor este vacio
        TXREG = bit_cadena;    //le envia el dato a cada "bit" de la cadena. 
           
}

void string2(char *str)  //regresa un pointer hacia un caracter
{
    while (*str != '\0')  //siempre y cuando el pointer sea distinto a caracter nulo (Que no se haya terminado la cadena)
    {
        give(*str);       //la funcion de give ira tomando caracter por caracter
        str++;            //aumentando para ir mandando bit por bit 
                
    }
}

void division(uint8_t counter)
//% es el operador modulo, el cual produce el residuo de una division
{
     centena = ((counter/100));
     decena = ((counter/10)%10);
     unidad = (counter%10);
     
     return;
}

