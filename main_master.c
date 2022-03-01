/*
 * File:   micro proyecto 2.c
 * Author: Lourdes Ruiz
 * Creado:25/02/2022
 * Descripcion: comunicacion I2C
 */
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
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

//*****************************************************************************
// Definición e importación de librerías
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "LCD.h"
#include "USART.h"
#include <xc.h>
//-----------------------------------------------------------------------------
//---------------------------------| VARIABLES |------------------------------                    
//-----------------------------------------------------------------------------
#define _XTAL_FREQ 8000000

uint8_t contador = 0;
char ESP32;


//-----------------------------------------------------------------------------
//-------------------------| PROTOTIPO DE FUNCIONES |--------------------------                    
//-----------------------------------------------------------------------------
void config_ports(void);
void config_oscilador(void);
void config_I2C_MASTER(void);

//-----------------------------------------------------------------------------
//---------------------------------| INTERRUPCION |----------------------------                   
//-----------------------------------------------------------------------------


void __interrupt() isr (void)
{
 
//                          Iinterrupción del USART
//----------------------------------------------------------------------------- 
    if (PIR1bits.RCIF){       //Si se recibe algo (BUS DE REPECION ESTA LLENO)
        ESP32 = RCREG;          
        PIR1bits.RCIF = 0;    //Apagar la badera de interrupción (cuando se lee)
                              //el registro RCREG
    }
    return;
}

//-----------------------------------------------------------------------------
//-------------------------------| LOOP PRINCIPAL |-----------------------------                    
//-----------------------------------------------------------------------------

void main() {
    config_ports();
    config_oscilador();
    config_I2C_MASTER();
    USART_init(_8MHz, _trON, _rcON);  //Config del modulo USART
    config_interrupts();
    
    Lcd_Init();                         //inicializar la LCD
    Lcd_Clear();                        //limpiar LCD
    
    Lcd_Set_Cursor(1,1);                //indicar la posicion
    Lcd_Write_String("S1:      Sem:");

    while(1){
        
        I2C_Master_Start();
        I2C_Master_Write(0x50);
        I2C_Master_Write(contador);
        I2C_Master_Stop();
        __delay_ms(200);
        
        /*
        I2C_Master_Start();
        I2C_Master_Write(0x51);
        PORTD = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
         * */
        Lcd_Set_Cursor(2,1);
      
        if (ESP32 == 1){
            contador = 0;
            Lcd_Write_String("           Verde");
        }
        else if (ESP32 == 2){
            contador = 1;
            Lcd_Write_String("        Amarillo");
        }
        else if (ESP32 == 3){
            contador = 2;
            Lcd_Write_String("            Rojo");
        }
        
        /*
        if(contador == 0){
            
            Lcd_Write_String("           Verde");
            //__delay_ms(100);
        }
        
        else if(contador == 1){
            
            Lcd_Write_String("        Amarillo");
            PORTAbits.RA1 = 1;
            PORTAbits.RA2 = 0;
            //__delay_ms(100);
        }
        
        else if(contador == 2){
            Lcd_Write_String("            Rojo");
            PORTAbits.RA1 = 0;
            PORTAbits.RA2 = 1;
            contador = 255;
            //__delay_ms(100);
        }
        
        __delay_ms(800);
        contador++; 
         * */  
        PORTA = contador;
        
    }
    
    return;
}

//****************************************************************************
//-------------------------------| FUNCIONES |-------------------------------                    
//****************************************************************************

//-----------------------------------------------------------------------------
//                     CONFIG de entradas y salidas
//-----------------------------------------------------------------------------    
void config_ports(void)
{
    ANSEL = 0;  
    ANSELH = 0;         //pines del puerto B como digitales
    
    TRISA = 0;                  //PORTA como salida     
    TRISB = 0;                  //PORTD como salida
    TRISD = 0;                  //PORTD como salida
    //TRISE = 0;                  //PORTD como salida
    
    PORTA = 0;                  //Limpiar puertos
    PORTB = 0;
    PORTD = 0;   
    //PORTE = 0;
}

//-----------------------------------------------------------------------------
//                      CONFIG oscilador 
//-----------------------------------------------------------------------------
void config_oscilador(void)
{
    OSCCONbits.IRCF = 0b0111;   //reloj a 8 MHz
    OSCCONbits.SCS = 1;         //bit 0 en 1 para colocar el R.Interno
}

//-----------------------------------------------------------------------------
//                      CONFIG SPI PARA EL MASTER 
//-----------------------------------------------------------------------------
void config_I2C_MASTER(void)
{
    I2C_Master_Init(100000);        // Inicializar Comuncación I2C

}
