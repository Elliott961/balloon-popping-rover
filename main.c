/*
 * File:   main.c
 * Author: Elliott
 *
 * Created on November 21, 2018, 11:00 AM
 */

#include<xc.h>
#include<math.h>
#include <stdio.h>
#include<pic18f4550.h>    
#pragma config FOSC=HS
#pragma config FCMEN=ON
#pragma config WDT=OFF
#pragma config IESO=ON
#pragma config XINST=OFF
#pragma config LVP=OFF
#pragma config PBADEN = OFF	
#define _XTAL_FREQ 10000000	

//rover
#define MR1 PORTBbits.RB2 
#define MR2 PORTBbits.RB3 
#define ML1 PORTBbits.RB4 
#define ML2 PORTBbits.RB5 

//lcd
#define data PORTD
#define RS PORTBbits.RB0
#define EN PORTBbits.RB1

//communication
#define Tx  PORTCbits.RC6 //
#define Rx  PORTCbits.RC7 //
#define pic PORTAbits.RA1 // connected to Tx of the Pi

// HC-SR04
#define Trigger_Pulse LATA1
#define Echo_Pulse LATA2

float ADC (int channel);
void right( int disp);
void left( int disp);
void lcdcmd( char value);
void lcddata( char value);
void lcd_delay( int time);
void NEXT_LINE();
void CLEAR_LCD();
void Trigger_Pulse_10us();
void num_to_ascii(int);
    char input,a0, a1, a2, a3, a4;
    int n,a,distance,Time,x,ang;
    float digital ;

void main(){
    TRISAbits.TRISA0 = 1;
    TRISCbits.TRISC7 = 1; //input (Rx)
    TRISCbits.TRISC6 = 0; //output (Tx)
    TRISAbits.TRISA1 = 0; //output (pic)
    TRISB=0;        //output (motors + lcd)
    TRISD=0;       //output (LCD)

    //Initialization Commands for communication (UART)
    TXSTA = 0x20; // low baud rate, 8-bit
    RCSTA = 0x90; //enable serial port and receiver
    SPBRG = 15; //9600 baud rate, crystal = 10 MHz 
    
                MR1= 0;
                MR2= 0;
                ML1= 0;
                ML2= 0;
    a0='0';
    a1='0';
    a2='0';
    a3='0';
    a4='0';
    x=0;
    EN = 0;
        
    //Initialization Commands for LCD
    lcd_delay(250);
	lcdcmd(0x38);	//init. LCD 2 lines, 5x7 matrix
	lcd_delay(250);
	lcdcmd(0x0E);	//Display on, cursor on
	lcd_delay(15);
	lcdcmd(0x01);	//clear LCD
	lcd_delay(15);
    
    
  
    //Initialization Commands for ADC
    ADCON1=0x0E;
    ADCON2=0x8F;
    ADRESH=0;
    ADRESL=0;
    pic=1;
    
while (1) {
    lcddata('X');
while (PIR1bits.RCIF == 0){
    lcddata('B');
       }
        
input = RCREG;
//lcddata('c');
lcddata(input);
 switch (input) {
    case 'F' : 
                MR1= 0;
                MR2= 1;
                ML1= 0;
                ML2= 1;
       
       break;
    case 'P' : 
                MR1= 1;
                MR2= 0;
                ML1= 0;
                ML2= 1;
       
       break;
    case 'L' : 
                pic=0;
                ang = RCREG;
                ang=ang-48;           
                right(ang);
                pic=1;
       
       break;
    case 'R' :  
                pic=0;
                ang = RCREG;
                ang=ang-48;           
                left(ang);
                pic=1;
       
       break;
    case 'S' : 
                MR1= 0;
                MR2= 0;
                ML1= 0;
                ML2= 0;
       
       break;
    default : 
                MR1= MR1;
                MR2= MR2;
                ML1= ML1;
                ML2= ML2;
       break;}
 
 
 
}
}
void Trigger_Pulse_10us()
{
    Trigger_Pulse = 1;
    __delay_us(10);
    Trigger_Pulse = 0;
}


void lcdcmd( char value){
data = value;
RS = 0; //Register select(cleared);sending commands to LCD 
EN = 1;
lcd_delay(1);
EN = 0;
}

void lcddata( char value){
data = value;
RS = 1;//set since we are sending data to LCD 
EN = 1;
lcd_delay(1);
EN = 0;
}

void NEXT_LINE(){    
    lcdcmd(0xc4); //new line
    lcd_delay(15);
}

void CLEAR_LCD(){
    lcdcmd(0x01);	//clear LCD
	lcd_delay(15);
}


void delay( int itime){
unsigned int i, j;
for(i=0;i<itime;i++)
for(j=0;j<180;j++);
}

void left( int disp){
    int a;
        a= disp*70;
        MR1= 1;
        MR2= 0;
        ML1= 0;
        ML2= 1;
        lcd_delay(a); 
        MR1=0;
        MR2= 0;
        ML1= 0;
        ML2= 0;
}

void right( int disp){
    int a;
        a= disp*70;
        MR1= 0;
        MR2= 1;
        ML1= 1;
        ML2= 0;
        lcd_delay(a); 
        MR1= 0;
        MR2= 0;
        ML1= 0;
        ML2= 0;
}


void lcd_delay( int itime){
unsigned int i, j;
for(i=0;i<itime;i++)
for(j=0;j<180;j++);
}

void num_to_ascii(int value){ 
    char x,d1,d2,d3,d4,d5;
    CLEAR_LCD();
    x=value/10;
	d1=value%10;
	d2=x%10;
    x=value/100;
	 a0 = 0x30 | d1;
	 a1 = 0x30 | d2;

    lcddata(' ');	
	lcd_delay(15); 
    lcddata(' ');	
	lcd_delay(15); 
    lcddata(' ');	
	lcd_delay(15); 
    lcddata(' ');	
	lcd_delay(15); 
    lcddata(a1);	
	lcd_delay(15); 
    lcddata(a0);	
	lcd_delay(15); 
}

float ADC (int channel){
    float dig;
    ADCON0 =(ADCON0 & 0b11000011)|((channel<<2) & 0b00111100);
    ADCON0 |= ((1<<ADON)|(1<<GO));
    while(ADCON0bits.GO_nDONE==1);
    dig = (ADRESH <<8) | (ADRESL);
    return (dig);
}
