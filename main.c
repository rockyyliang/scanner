/*******************************************************
This program was created by the CodeWizardAVR V3.27 UL 
Automatic Program Generator
© Copyright 1998-2016 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 11/27/2017
Author  : 
Company : 
Comments: code for scanner project


Chip type               : ATmega2560
Program type            : Application
AVR Core Clock frequency: 16 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 2048
*******************************************************/
/* the LCD module is connected to PORTC */
#asm
.equ __lcd_port = 0x08
#endasm

#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <spi.h>
#include <delay.h>
#include "USART0.h"

#define SIGPIN 0x80   //Arduino pin 30
#define SIGPORT PORTC
#define SIGDDR DDRC
#define SIGREAD PINC

// Declare your global variables here
//char charASCII[17];
char i;
// encoder channel statuses
char statusA;
char statusB;
signed int pos = 0;
long longpos, relpos, initialpos;
char statusCh = 1;
char duty = 50;
char runduty = 50;
char incr = 1;
float range = 960;

static int last = 0;
int rads;
signed int deltaTheta; 
signed int e, elast;
signed int r = 2000;
signed long c, clast;
signed long m;
unsigned char timer1_overflow_count=0;

int encoder(void){
	//read encoder
	static int shaftCnt;
	/*
	//test code to turn on LEDs connected to port A, based on encoder high/low
	//encoder channelA:PINL6; channelB:PINL7
	if(((PINL & 0b10000000) >> 7) == 1){
		//if this channel is high, turn on LED7
		PORTA = PORTA & 0b01111111;
	}
	if(((PINL & 0b10000000) >> 7) == 0){
		//if this channel is low, turn off LED7
		PORTA = PORTA | 0b10000000;
	}
	if(((PINL & 0b01000000) >> 6) == 1){
		PORTA = PORTA & 0b10111111;
	}
	if(((PINL & 0b01000000) >> 6) == 0){
		PORTA = PORTA | 0b01000000;
	}
	*/
	
	/*put this in void main
	statusA = ((PINL & 0b01000000) >> 6);
	statusB = ((PINL & 0b10000000) >> 7);
	*/
    
	if (statusCh ==1){
		if (statusB != ((PINL & 0b10000000) >> 7)){
			statusB = ((PINL & 0b10000000) >> 7);
			statusCh = 2;
			shaftCnt++;
		}
		if (statusA != ((PINL & 0b01000000) >> 6)){
			statusA = ((PINL & 0b01000000) >> 6);
			statusCh = 4;
			shaftCnt--;
		}
	}
	if (statusCh == 2){
		if (statusA != ((PINL & 0b01000000) >> 6)){
			statusA = ((PINL & 0b01000000) >> 6);
			statusCh = 3;
			shaftCnt++;
		}
		if (statusB != ((PINL & 0b10000000) >> 7)){
			statusB = ((PINL & 0b10000000) >> 7);
			statusCh = 1;
			shaftCnt--;
		}
	}
	if (statusCh == 3){
		if (statusA != ((PINL & 0b01000000) >> 6)){
			statusA = ((PINL & 0b01000000) >> 6);
			statusCh = 2;
			shaftCnt--;
		}
		if (statusB != ((PINL & 0b10000000) >> 7)){
			statusB = ((PINL & 0b10000000) >> 7);
			statusCh = 4;
			shaftCnt++;
		}
	}
	if (statusCh == 4){
		if (statusA != ((PINL & 0b01000000) >> 6)){
			statusA = ((PINL & 0b01000000) >> 6);
			statusCh = 1;
			shaftCnt++;
		}
		if (statusB != ((PINL & 0b10000000) >> 7)){
			statusB = ((PINL & 0b10000000) >> 7);
			statusCh = 3;
			shaftCnt--;
		}
	}
return shaftCnt;
}

//Calculates the RPM using the change given
int findRads(int change){
    //calculate RPM of motor
    int rpm;
    rpm = 3*change;//(100 * change * 60) / 2000;
    rads = rpm*(2*3.14159/60);
    return rads;
}

int c2a(int count){
    /*convert encoder counts to angle
    range is a global var, stores sweep range in encoder counts
    */
    static int angle;
    angle = (count/range)*360;
    return angle;
}

void initPWM(void){
	DDRB = 0xFF;
    //initializes timer for PWM
    //PWM output comes from PB7(OC0A, pin13 on mega)
    // Timer/Counter 0 initialization
    //FAST PWN mode, non inverting
    TCCR0A=(1<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
    //no prescaling
    TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (1<<CS00);
    TCNT0=0x00;
	//start PWM at 50%, 0 velocity command
    OCR0A=128;
    
    // Timer/Counter 0 Interrupt(s) initialization
    //TIMSK0=(0<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0);
}

void PWM(unsigned char duty){
    //changes duty cycle, takes 0-100
    OCR0A = (duty*255)/100;
}

void PWM8b(unsigned char duty){
    //changes duty cycle, takes 0-255
    OCR0A = duty;
}

void LS7366_Init(void){   delay_ms(10);
   
   //chip select encoder
   //select x axis
   //PORTB = PORTB & 0b11101111;
   //select y axis
   PORTH = PORTH & 0b10111111;
   
   spi(0x88); 
   spi(0x03); // x4 encoding
   spi(0x90); 
   spi(0x00); // put in 1 byte mode
   
   //chip deselect, x
   //PORTB = PORTB | 0b00010000;
   //deselect y
   PORTH = PORTH | 0b01000000;
}

long getEncoderValue(void)
{
    unsigned char count1Value, count2Value, count3Value, count4Value;

    long result;
    
    //selectEncoder(encoder);
    PORTB = PORTB & 0b11101111;
    
    spi(0x60); // Request count
    count1Value = spi(0x00); // Read highest order byte
    count2Value = spi(0x00);
    count3Value = spi(0x00);
    count4Value = spi(0x00); // Read lowest order byte
    
    //deselectEncoder(encoder);
    PORTB = PORTB | 0b00010000;
    

    result= (((long)count1Value)<<24) + (((long)count2Value)<<16) + (((long)count3Value)<<8) + (long)count4Value;
    
    return result;
} 

int intEncoder(void){
    unsigned char count1Value, count2Value, count3Value, count4Value;

    int result;
    
    //select x axis
    //PORTB = PORTB & 0b11101111;
    //select y axis
    PORTH = PORTH & 0b10111111;
    
    spi(0x60); // Request count
    count1Value = spi(0x00); // Read highest order byte
    count2Value = spi(0x00);
    count3Value = spi(0x00);
    count4Value = spi(0x00); // Read lowest order byte
    
    //chip deselect, x
    //PORTB = PORTB | 0b00010000;
    //deselect y
    PORTH = PORTH | 0b01000000;
    

    result= (((int)count3Value)<<8) + (int)count4Value;
    //putchar(count3Value);
    return result;
} 

unsigned int scan(){
   float duration;
  int dist;
  char highpulse;
  highpulse =1;
  duration=0;
  //send 5 second pulse to SIG pin (30) (PORTC.7)
  SIGDDR = SIGDDR | SIGPIN;
  SIGPORT = SIGPORT | SIGPIN;
  delay_us(5);
  SIGPORT = SIGPORT & (~SIGPIN);
  SIGDDR = SIGDDR & (~SIGPIN); //set SIG pin to read
  
 //disable timer2 to avoid being interrupted
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);
 timer1_overflow_count=0;
 #asm("sei")
   
  //read pulse and calculate distance
  //blocking function while we wait for a high pulse
  while((SIGREAD & SIGPIN)==0){
	  if (timer1_overflow_count>25) {
		  highpulse=0;
		  break;
	  }
	  } 

//reset and use timer1 to measure pulse width
     #asm("cli")
	 TCNT1H=0;
     TCNT1L=0;
	 timer1_overflow_count=0;
     #asm("sei")                                         
     
//blocking function while we wait for the pulse to go low	   
  while(SIGREAD & SIGPIN && (highpulse)){ 
	  if (timer1_overflow_count>25) {
		  break;
	  }
	  }
  
  duration = (TCNT1L+250*(int)timer1_overflow_count)/250.0;//convert to ms
  dist = (float)duration*17.0*highpulse;//calculate cm from ms
  if(dist>300){dist=3;}
  #asm("cli")
  return dist;
}

interrupt [TIM2_COMPA] void timer2_compa_isr(void){
/*
putchar(intEncoder());

// PORTD = ~PORTD
//relpos = longpos - initialpos;
//ramp duty cycle
//duty = duty + incr;
//if(duty >= 100) incr = -1;
//if(duty <= 0) incr = 1;
//PWM(duty);

*/
int dist;
long pos;

dist = scan();
pos = initialpos - intEncoder()+range;

//serial communication
putchar(0x0A);
putchar(pos>>8);
putchar(pos);
putchar(0x0A);
putchar(dist>>8);
putchar(dist); 
putchar(0x0A);
	                                                   

while((initialpos - intEncoder()) < -range){
runduty = 60;
break;
}

while((initialpos - intEncoder()) > range){
runduty = 40;
break;
}

         
PWM(runduty);



//reset and enable timer2
TCNT2 =0x00;
TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);

/*
//communications
putchar(intEncoder());
//PORTK = 0;
//PORTF = ((initialpos - pos)>>8);
//PORTK = ~PORTK;
PORTK = ~PORTK;
*/
}

// Timer1 output compare A interrupt service routine
interrupt [TIM1_COMPA] void timer1_compa_isr(void){
 timer1_overflow_count++;
}

void main(void)
{
// Declare your local variables here
//DDRB = 0xFF;

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port A initialization
// Function: in 
DDRA= 0x00;
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=0x00;

// Port B initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (1<<DDB4) | (0<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRC=0x00;
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTC=0x00;

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Port E initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRE=(0<<DDE7) | (0<<DDE6) | (0<<DDE5) | (0<<DDE4) | (0<<DDE3) | (0<<DDE2) | (0<<DDE1) | (0<<DDE0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTE=(0<<PORTE7) | (0<<PORTE6) | (0<<PORTE5) | (0<<PORTE4) | (0<<PORTE3) | (0<<PORTE2) | (0<<PORTE1) | (0<<PORTE0);

// Port F initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRF=0xFF;
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTF=(0<<PORTF7) | (0<<PORTF6) | (0<<PORTF5) | (0<<PORTF4) | (0<<PORTF3) | (0<<PORTF2) | (0<<PORTF1) | (0<<PORTF0);

// Port G initialization
// Function: Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRG=(0<<DDG5) | (0<<DDG4) | (0<<DDG3) | (0<<DDG2) | (0<<DDG1) | (0<<DDG0);
// State: Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTG=(0<<PORTG5) | (0<<PORTG4) | (0<<PORTG3) | (0<<PORTG2) | (0<<PORTG1) | (0<<PORTG0);

// Port H initialization
// Function: out 
DDRH= 0xFF;
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTH=(0<<PORTH7) | (0<<PORTH6) | (0<<PORTH5) | (1<<PORTH4) | (0<<PORTH3) | (0<<PORTH2) | (0<<PORTH1) | (0<<PORTH0);

// Port J initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRJ=(0<<DDJ7) | (0<<DDJ6) | (0<<DDJ5) | (0<<DDJ4) | (0<<DDJ3) | (0<<DDJ2) | (0<<DDJ1) | (0<<DDJ0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTJ=(0<<PORTJ7) | (0<<PORTJ6) | (0<<PORTJ5) | (0<<PORTJ4) | (0<<PORTJ3) | (0<<PORTJ2) | (0<<PORTJ1) | (0<<PORTJ0);

// Port K initialization
// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRK=0xFF;
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTK=(0<<PORTK7) | (0<<PORTK6) | (0<<5) | (0<<PORTK4) | (0<<PORTK3) | (0<<PORTK2) | (0<<PORTK1) | (0<<PORTK0);

// Port L initialization
// Function: Bit7=In Bit6=In Bit5=out Bit4=out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRL=(0<<DDL7) | (0<<DDL6) | (1<<DDL5) | (1<<DDL4) | (0<<DDL3) | (0<<DDL2) | (0<<DDL1) | (0<<DDL0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTL=(0<<PORTL7) | (0<<PORTL6) | (0<<PORTL5) | (0<<PORTL4) | (0<<PORTL3) | (0<<PORTL2) | (0<<PORTL1) | (0<<PORTL0);

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 250.000 kHz
// Mode: CTC top=OCR1A
// OC1A output: Disconnected
// OC1B output: Disconnected
// OC1C output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 1 ms
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: On
// Compare B Match Interrupt: Off
// Compare C Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<COM1C1) | (0<<COM1C0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0xF9;
OCR1BH=0x00;
OCR1BL=0x00;
OCR1CH=0x00;
OCR1CL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 15.625 kHz
// Mode: CTC top=OCR2A
// OC2A output: Disconnected
// OC2B output: Disconnected
// Timer Period: 3.392 ms (300 Hz interrupt)
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20);
TCNT2=0x00;
OCR2A=0x34;
OCR2B=0x00;

// SPI initialization
// SPI Type: Master
// SPI Clock Rate: 4000.000 kHz
// SPI Clock Phase: Cycle Start
// SPI Clock Polarity: Low
// SPI Data Order: MSB First
SPCR=(0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);
SPSR=(0<<SPI2X);


//lcd_init();
//lcd_clear();

/*
statusA = ((PINL & 0b01000000) >> 6);
statusB = ((PINL & 0b10000000) >> 7);

delay_ms(50);

pos = encoder();

*/


// Globally enable interrupts
LS7366_Init();
initusart();
initPWM();
#asm("sei")

//wait for signal from LabVIEW before beginning data acquisition on Arduino
getchar();

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1C) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);

initialpos = intEncoder();

PWM(45);

while (1)
      {



}
   
      	  
//	  itoa(shaftCnt, charASCII);
//	  lcd_clear();
//	  lcd_gotoxy(18,1);
//	  lcd_puts(charASCII);
	  
      }
	  //PORTL = PORTL & 0b11101111;

