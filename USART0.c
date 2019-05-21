/*******************************************************
This program was created by the CodeWizardAVR V3.27 UL 
Automatic Program Generator
© Copyright 1998-2016 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : lab13
Version : 1
Date    : 11/15/2017
Author  : Chris Nguyen & Rocky Liang
Company : UW Madison

Chip type               : ATmega2560
Program type            : Application
AVR Core Clock frequency: 14.745600 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 2048
*******************************************************/

#include <io.h>
#include "USART0.h"


// USART0 Receiver buffer

char rx_buffer0[RX_BUFFER_SIZE0];

#if RX_BUFFER_SIZE0 <= 256
volatile unsigned char rx_wr_index0=0,rx_rd_index0=0;
#else
volatile unsigned int rx_wr_index0=0,rx_rd_index0=0;
#endif

#if RX_BUFFER_SIZE0 < 256
volatile unsigned char rx_counter0=0;
#else
volatile unsigned int rx_counter0=0;
#endif

// This flag is set on USART0 Receiver buffer overflow
bit rx_buffer_overflow0;

 
/*******************************************************
Description: 
USART0 Receiver interrupt service routine : checks if there are characters in the buffer

*******************************************************/
interrupt [USART0_RXC] void usart0_rx_isr(void){
unsigned char status;
char data;
status=UCSR0A;
data=UDR0;
if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
   {
   rx_buffer0[rx_wr_index0++]=data;
#if RX_BUFFER_SIZE0 == 256
   // special case for receiver buffer size=256
   if (++rx_counter0 == 0) rx_buffer_overflow0=1;
#else
   if (rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
   if (++rx_counter0 == RX_BUFFER_SIZE0)
      {
      rx_counter0=0;
      rx_buffer_overflow0=1;
      }
#endif
   }
}

// Get a character from the USART0 Receiver buffer
#define _ALTERNATE_GETCHAR_
#pragma used+
/*******************************************************
Description: 
Function that returns the next character in the receiver buffer

Returns: 
        data: the next character in the rx buffer
*******************************************************/
char getchar(void)
{
char data;
while (rx_counter0==0);
data=rx_buffer0[rx_rd_index0++];
#if RX_BUFFER_SIZE0 != 256
if (rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
#endif
#asm("cli")
--rx_counter0;
#asm("sei")
return data;
}
#pragma used-

// USART0 Transmitter buffer
#define TX_BUFFER_SIZE0 16
char tx_buffer0[TX_BUFFER_SIZE0];

#if TX_BUFFER_SIZE0 <= 256
volatile unsigned char tx_wr_index0=0,tx_rd_index0=0;
#else
volatile unsigned int tx_wr_index0=0,tx_rd_index0=0;
#endif

#if TX_BUFFER_SIZE0 < 256
volatile unsigned char tx_counter0=0;
#else
volatile unsigned int tx_counter0=0;
#endif

/*******************************************************
Description: 
USART0 Transmitter interrupt service routinee : grabs the next character from the buffer

*******************************************************/
interrupt [USART0_TXC] void usart0_tx_isr(void){
#asm("sei");
if (tx_counter0)
   {
   --tx_counter0;
   UDR0=tx_buffer0[tx_rd_index0++];
#if TX_BUFFER_SIZE0 != 256
   if (tx_rd_index0 == TX_BUFFER_SIZE0) tx_rd_index0=0;
#endif
   }
}

// 
/*******************************************************
Description: 
Write a character to the USART0 Transmitter buffer

Parameters:
    c: a character to put to the transmitter buffer
*******************************************************/
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c){
while (tx_counter0 == TX_BUFFER_SIZE0);
#asm("cli")
if (tx_counter0 || ((UCSR0A & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer0[tx_wr_index0++]=c;
#if TX_BUFFER_SIZE0 != 256
   if (tx_wr_index0 == TX_BUFFER_SIZE0) tx_wr_index0=0;
#endif
   ++tx_counter0;
   }
else
   UDR0=c;
#asm("sei")
}
#pragma used-

#include <stdio.h>

/*******************************************************
Description: 
Initializes the USART for serial communication in an external source file
*******************************************************/

void initusart(void){
// USART0 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART0 Receiver: On
// USART0 Transmitter: On
// USART0 Mode: Asynchronous
// USART0 Baud Rate: 38400
UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
UCSR0B=(1<<RXCIE0) | (1<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
UBRR0H=0x00;
UBRR0L=0x19;
}