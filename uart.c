/*
 * uart.h
 * Version 1
 * Created: 14.12.2019 18:07:06
 *  Author: gfcwfzkm
 */ 

#include "uart.h"

#if defined(__AVR_ATmega4809__)
#define UART0_PORT_NORMAL	PORTA
#define UART0_TX_NORMAL		PIN0_bm
#define UART0_RX_NORMAL		PIN1_bm
#define UART0_PORT_MUX		PORTA
#define UART0_TX_MUX		PIN4_bm
#define UART0_RX_MUX		PIN5_bm

#define UART1_PORT_NORMAL	PORTC
#define UART1_TX_NORMAL		PIN0_bm
#define UART1_RX_NORMAL		PIN1_bm
#define UART1_PORT_MUX		PORTC
#define UART1_TX_MUX		PIN4_bm
#define UART1_RX_MUX		PIN5_bm

#define UART2_PORT_NORMAL	PORTF
#define UART2_TX_NORMAL		PIN0_bm
#define UART2_RX_NORMAL		PIN1_bm
#define UART2_PORT_MUX		PORTF
#define UART2_TX_MUX		PIN4_bm
#define UART2_RX_MUX		PIN5_bm

#define UART3_PORT_NORMAL	PORTB
#define UART3_TX_NORMAL		PIN0_bm
#define UART3_RX_NORMAL		PIN1_bm
#define UART3_PORT_MUX		PORTB
#define UART3_TX_MUX		PIN4_bm
#define UART3_RX_MUX		PIN5_bm
#else
#error "Fehler! Pindefinitionen fuer den Controller hinzufuegen!"
#endif


int8_t _SigRowError = 0;
void process_receive_interrupt(BUF_UART_t *temp);		// Interner Interrupt-Handler
void process_transmit_interrupt(BUF_UART_t *temp);		// Interner Interrupt-Handler

BUF_UART_t *_usart0 = 0;
ISR(USART0_RXC_vect)
{
	process_receive_interrupt(_usart0);
}
ISR(USART0_DRE_vect)
{
	process_transmit_interrupt(_usart0);
}
#if defined(USART1_RXC_vect)
BUF_UART_t *_usart1 = 0;
ISR(USART1_RXC_vect)
{
	process_receive_interrupt(_usart1);
}
ISR(USART1_DRE_vect)
{
	process_transmit_interrupt(_usart1);
}
#endif
#if defined(USART2_RXC_vect)
BUF_UART_t *_usart2 = 0;
ISR(USART2_RXC_vect)
{
	process_receive_interrupt(_usart2);
}
ISR(USART2_DRE_vect)
{
	process_transmit_interrupt(_usart2);
}
#endif
#if defined(USART3_RXC_vect)
BUF_UART_t *_usart3 = 0;
ISR(USART3_RXC_vect)
{
	process_receive_interrupt(_usart3);
}
ISR(USART3_DRE_vect)
{
	process_transmit_interrupt(_usart3);
}
#endif

/**
 * @brief Interrupt-Empfangsfunktion (Interrupt Service Routine)
 */
void process_receive_interrupt(BUF_UART_t *temp)
{
	uint8_t tmphead,data,status,lastRxError = 0;
		
	/* Sicherstellen das der Pointer überhaupt wohin zeigt! */
	if (temp == 0)	return;
	
	/* Statusregister abarbeiten */
	status = (*temp->hw_usart).RXDATAH;
	if (status & USART_BUFOVF_bm)	lastRxError |= UART_OVERRUN_ERROR;
	if (status & USART_FERR_bm)		lastRxError |= UART_FRAME_ERROR;
	if (status & USART_PERR_bm)		lastRxError |= UART_PARITY_ERROR;
	
	/* UART Datenregister einlesen */
	data = (*temp->hw_usart).RXDATAL;	
	
	/* Neuer Bufferindex berechnen und überprüfen */
	tmphead = (temp->rxBufHead + 1) & (temp->rxBufLen - 1);
	
	if (tmphead == temp->rxBufTail)
	{
		/* Fehler! Der Empfangsbuffer ist voll! */
		lastRxError |= UART_BUFFER_OVERFLOW;
	}
	else
	{
		lastRxError |= UART_DATA_AVAILABLE;
		/* Neuer Bufferindex speichern */
		temp->rxBufHead = tmphead;
		/* Empfangenes Byte im Buffer speichern */
		temp->rxBuffer[tmphead] = data;
		/* Empfangenes Byte ggf. überprüfen */
		if ( (temp->lookForChar != 00) && (temp->lookForChar == data) )
		{
			temp->specialCharFound = 1;
		}
	}
	temp->lastError |= lastRxError;
	
	/* Empfangs / Rx Interrupt-Flag löscht sich beim Lesevorgang von selber */
}

/**
 * @brief Interrupt-Sendefunktion (Interrupt Service Routine)
 */
void process_transmit_interrupt(BUF_UART_t *temp)
{
	uint8_t tmptail;
		
	/* Sicherstellen das der Pointer überhaupt wohin zeigt! */
	if (temp == 0)	return;
	
	if (temp->txBufHead != temp->txBufTail)
	{
		/* Neuer Bufferindex berechnen und speichern */
		tmptail = (temp->txBufTail + 1) & (temp->txBufLen - 1);
		temp->txBufTail = tmptail;
		/* Byte aus dem Buffer ins UART Senderegister legen */
		(*temp->hw_usart).TXDATAL = temp->txBuffer[tmptail];
	}
	else
	{
		/* Tx Buffer leer, Sendeinterrupt deaktivieren */
		(*temp->hw_usart).CTRLA &=~ USART_DREIE_bm;
	}
}

void uart_storeFreqCompValue(int8_t SigRowError)
{
	_SigRowError = SigRowError;
}

void uart_init(BUF_UART_t *uartPtr, volatile USART_t *_usart, uint32_t _baud, enum UART_SETTING _setting, uint8_t *recivBuf,  const uint8_t recivBufLen, uint8_t *transBuf, const uint8_t transBufLen, enum uart_useMUX useMUX)
{
	uint8_t muxbits;
	int32_t baud_setting;
	volatile PORT_t *_port;
	uint8_t rxPin, txPin;
	
	/* Port und Pins zuweisen */
	if (_usart == (&USART0))
	{
		muxbits = PORTMUX_USART0_ALT1_gc;
		if (useMUX)
		{
			rxPin = UART0_RX_MUX;
			txPin = UART0_TX_MUX;
			_port = &UART0_PORT_MUX;
		}
		else
		{
			rxPin = UART0_RX_NORMAL;
			txPin = UART0_TX_NORMAL;
			_port = &UART0_PORT_NORMAL;
		}
		_usart0 = uartPtr;
	}
#if defined(USART1_TXC_vect)
	else if (_usart == (&USART1))
	{
		muxbits = PORTMUX_USART1_ALT1_gc;
		if (useMUX)
		{
			rxPin = UART1_RX_MUX;
			txPin = UART1_TX_MUX;
			_port = &UART1_PORT_MUX;
		}
		else
		{
			rxPin = UART1_RX_NORMAL;
			txPin = UART1_TX_NORMAL;
			_port = &UART1_PORT_NORMAL;
		}
		_usart1 = uartPtr;
	}
#endif
#if defined(USART2_TXC_vect)
	else if (_usart == (&USART2))
	{
		muxbits = PORTMUX_USART2_ALT1_gc;
		if (useMUX)
		{
			rxPin = UART2_RX_MUX;
			txPin = UART2_TX_MUX;
			_port = &UART2_PORT_MUX;
		}
		else
		{
			rxPin = UART2_RX_NORMAL;
			txPin = UART2_TX_NORMAL;
			_port = &UART2_PORT_NORMAL;
		}
		_usart2 = uartPtr;
	}
#endif
#if defined(USART3_TXC_vect)
	else if (_usart == &USART3)
	{
		muxbits = PORTMUX_USART3_ALT1_gc;
		if (useMUX)
		{
			rxPin = UART3_RX_MUX;
			txPin = UART3_TX_MUX;
			_port = &UART3_PORT_MUX;
		}
		else
		{
			rxPin = UART3_RX_NORMAL;
			txPin = UART3_TX_NORMAL;
			_port = &UART3_PORT_NORMAL;
		}
		_usart3 = uartPtr;
	}
#endif
	else
	{
		/* Unbekanntes UART Register */
		return;
	}
	
	/* UARTInstanz Pointer sauber zurücksetzten / definieren */
	uartPtr->rxBuffer = recivBuf;
	uartPtr->rxBufLen = recivBufLen;
	uartPtr->rxBufHead = 0;
	uartPtr->rxBufTail = 0;
	uartPtr->txBuffer = transBuf;
	uartPtr->txBufLen = transBufLen;
	uartPtr->txBufTail = 0;
	uartPtr->txBufHead = 0;
	uartPtr->lastError = 0;
	uartPtr->lookForChar = 0;
	uartPtr->specialCharFound = 0;
	uartPtr->hw_usart = _usart;
	
	/* Port MUX einstellen */
	if (useMUX)	PORTMUX.USARTROUTEA |= muxbits;
	
	
	/* Interrupt deaktivieren während der Initialisierung */
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		/* Baudrate berechnen, gegebenenfalls Kalibrierungswerte verwenden */
		baud_setting = (((8 * F_CPU) / _baud) + 1) / 2;
		/* Double-Speed deaktivieren */
		(*_usart).CTRLB &= (~USART_RXMODE_CLK2X_gc);
		(*_usart).CTRLB |= USART_RXMODE_NORMAL_gc;
		
		baud_setting = (((8 * F_CPU) / _baud) + 1) / 2;
		if (_SigRowError != 0)	baud_setting += (baud_setting * _SigRowError) / 1024;
		
		(*_usart).BAUD = (int16_t) baud_setting;
		
		/* UART Modus setzten */
		(*_usart).CTRLC = _setting;
		
		/* Sender und Empfänger aktivieren */
		(*_usart).CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);
		(*_usart).CTRLA |= USART_RXCIE_bm;
		
		/* I/Os einstellen */
		(*_port).DIRCLR = rxPin;
		
		(*_port).OUTSET = txPin;
		(*_port).DIRSET = txPin;
	}
}

void uart_putc(BUF_UART_t *uartPtr, uint8_t data)
{
	uint8_t tmphead, tmpBufTail;
	
	
	tmphead = (uartPtr->txBufHead + 1) & (uartPtr->txBufLen - 1);
		
	do 
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			tmpBufTail = uartPtr->txBufTail;
		}
		asm("nop");	/* Warten bis Platz im Buffer ist */		
	}while(tmphead == tmpBufTail);
	
	/* Zu sendendes Byte in den Buffer legen */
	uartPtr->txBuffer[tmphead] = data;
	uartPtr->txBufHead = tmphead;
	
	/* 'Data Register Empty' Interrupt aktivieren */
	(*uartPtr->hw_usart).CTRLA |= USART_DREIE_bm;
}

void uart_send(BUF_UART_t *uartPtr, uint8_t *dataBuffer, uint8_t bytesToSend)
{
	/* Anzahl Bytes vom Buffer senden */
	for (uint8_t i = 0; i < bytesToSend; i++)
	{
		uart_putc(uartPtr, dataBuffer[i]);
	}
}

void uart_print(BUF_UART_t *uartPtr, char *data)
{
	/* String bis Null-Terminator senden */
	while(*data)
	{
		uart_putc(uartPtr, *data++);
	}
}

void uart_print_p(BUF_UART_t *uartPtr, const char *progmem_s )
{
	register char c;
	/* String bis Null-Terminator vom FLASH-Speicher senden */
	while ( (c = pgm_read_byte(progmem_s++)) )
	{
		uart_putc(uartPtr, c);
	}

}

UART_ERROR uart_rxStatus(BUF_UART_t *uartPtr)
{
	uint8_t lastRxError = uartPtr->lastError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		lastRxError |= UART_NO_DATA;
	}
	
	/* Aktuellster Status zurückgeben, ohne ihn zu löschen */
	return (lastRxError);
}

uint16_t uart_peek(BUF_UART_t *uartPtr)
{
	uint8_t data, tmptail, lastRxError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		return (UART_NO_DATA << 8);	// Keine Daten!
	}
	
	/* Bufferindex berechnen */
	tmptail = (uartPtr->rxBufTail + 1) & (uartPtr->rxBufLen);
	
	/* FIFO Buffer -> ältestes empfangene Byte aus dem Buffer lesen */
	data = uartPtr->rxBuffer[tmptail];
	lastRxError = uartPtr->lastError;
	
	return ((lastRxError << 8) | data);
}

uint16_t uart_getc(BUF_UART_t *uartPtr)
{
	uint8_t data, tmptail, lastRxError;
	
	if (uartPtr->rxBufHead == uartPtr->rxBufTail)
	{
		return (UART_NO_DATA << 8);	// Keine Daten!
	}
	
	/* Bufferindex berechnen */
	tmptail = (uartPtr->rxBufTail + 1) & (uartPtr->rxBufLen - 1);
	
	/* FIFO Buffer -> ältestes empfangene Byte aus dem Buffer lesen */
	data = uartPtr->rxBuffer[tmptail];
	lastRxError = uartPtr->lastError;
	
	/* Bufferindex speichern */
	uartPtr->rxBufTail = tmptail;
	uartPtr->specialCharFound = 0;
	
	uartPtr->lastError = UART_DATA_AVAILABLE;
	return ((lastRxError << 8) | data);
}

UART_ERROR uart_isTxFull(BUF_UART_t *uartPtr)
{
	uint8_t tmphead = (uartPtr->txBufHead + 1) & (uartPtr->txBufLen - 1);
	
	if (tmphead == uartPtr->txBufTail)	return UART_TX_BUFFER_FULL;
	return UART_DATA_AVAILABLE;
}

uint8_t uart_charDetected(BUF_UART_t *uartPtr)
{
	uint8_t tmpHead,tmpTail,tmpLen,txtlen = 0;
	
	if (uartPtr->specialCharFound)
	{
		tmpHead = uartPtr->rxBufHead;
		tmpTail = uartPtr->rxBufTail;
		tmpLen = uartPtr->rxBufLen - 1;
		
		do
		{
			tmpTail = (tmpTail + 1) & tmpLen;
			
			txtlen++;
			
			if (tmpTail == tmpHead)
			{
				txtlen = 0;
				break;
			}
		} while (uartPtr->rxBuffer[tmpTail] != uartPtr->lookForChar);
		
		uartPtr->specialCharFound = 0;
	}
	
	return txtlen;
}

void uart_searchForCharacter(BUF_UART_t *uartPtr, char charToSearch)
{
	uartPtr->specialCharFound = 0;
	uartPtr->lookForChar = charToSearch;
}