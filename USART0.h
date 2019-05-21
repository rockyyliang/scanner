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

#define RX_BUFFER_SIZE0 8
#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE (1<<RXC0)
#define FRAMING_ERROR (1<<FE0)
#define PARITY_ERROR (1<<UPE0)
#define DATA_OVERRUN (1<<DOR0)

interrupt [USART0_RXC] void usart0_rx_isr(void);
char getchar(void);
interrupt [USART0_TXC] void usart0_tx_isr(void);
void putchar(char);
void initusart(void);

#include <stdio.h>


