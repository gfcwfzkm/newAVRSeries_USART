/*
 * uart.h
 * Version 1
 * Created: 14.12.2019 18:07:06
 *  Author: gfcwfzkm
 */ 


#ifndef UART_H_
#define UART_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

/**
 * @brief Einstellung der serielle Schnittstelle
 *
 * Die erste Zahl steht f�r die Anzahl zu �bertragende Bits,
 * der Buchstabe f�r entweder kein Parit�tsbit (N), gerades bit (E) oder ungerades Parit�tsbit (O),
 * zuguterletzt wird mit der letzten Zahl die Anzahl Stopbits �bergeben
 */
enum UART_SETTING {
	SERIAL_5N1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_5BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc),
	SERIAL_6N1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_6BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc),
	SERIAL_7N1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_7BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc),
	SERIAL_8N1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc),
	
	SERIAL_5N2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_5BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_2BIT_gc),
	SERIAL_6N2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_6BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_2BIT_gc),
	SERIAL_7N2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_7BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_2BIT_gc),
	SERIAL_8N2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_2BIT_gc),

	SERIAL_5E1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_5BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_1BIT_gc),
	SERIAL_6E1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_6BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_1BIT_gc),
	SERIAL_7E1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_7BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_1BIT_gc),
	SERIAL_8E1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_1BIT_gc),

	SERIAL_5E2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_5BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_2BIT_gc),
	SERIAL_6E2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_6BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_2BIT_gc),
	SERIAL_7E2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_7BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_2BIT_gc),
	SERIAL_8E2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_2BIT_gc),

	SERIAL_5O1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_5BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_1BIT_gc),
	SERIAL_6O1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_6BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_1BIT_gc),
	SERIAL_7O1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_7BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_1BIT_gc),
	SERIAL_8O1 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_1BIT_gc),

	SERIAL_5O2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_5BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_2BIT_gc),
	SERIAL_6O2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_6BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_2BIT_gc),
	SERIAL_7O2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_7BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_2BIT_gc),
	SERIAL_8O2 = (USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_ODD_gc | USART_SBMODE_2BIT_gc)
};

/**
 * @brief Fehlerr�ckgabewert / Status des UART-Empfanges
 */
typedef enum {
	UART_DATA_AVAILABLE		= 0x00,	/**< Daten vorhanden / Tx Buffer frei */
	UART_NO_DATA			= 0x01,	/**< Keine Daten vorhanden */
	UART_BUFFER_OVERFLOW	= 0x02,	/**< Buffer l�uft �ber */
	UART_PARITY_ERROR		= 0x04,	/**< Parit�tsbit fehlerhaft */
	UART_OVERRUN_ERROR		= 0x08,	/**< UART-Hardware �berrannt / l�uft �ber */
	UART_FRAME_ERROR		= 0x10,	/**< Timingfehler */
	UART_TX_BUFFER_FULL		= 0x20	/**< Sendebuffer ist voll */
}UART_ERROR;


enum uart_useMUX {
	NORMAL_UART_PINPOS=0,	/**< UART Pin MUX Position unver�ndert lassen */
	MUXED_UART_PINPOS=1		/**< UART Pin MUX Position ver�ndern */
};

/**
 * @brief Typendefinition f�r die serielle Schnittstelle(n)
 *
 * Beinhaltet das Grundregister der seriellen Schnittstelle,
 * die Buffern und den Ringbuffer.
 */
typedef struct {
	uint8_t *rxBuffer;	/**< uint8_t Zeiger zum Empfangsbuffer, maximal 255 Bytes */
	uint8_t *txBuffer;	/**< uint8_t Zeiger zum Sendebuffer, maximal 255 Bytes */
	uint8_t rxBufLen;	/**< uint8_t Buffergr�sse des Empfangsbuffers in Bytes */
	uint8_t txBufLen;	/**< uint8_t Buffergr�sse des Sendebuffers in Bytes */
	
	uint8_t rxBufHead;	/**< uint8_t Ringbuffer-Z�hler */
	uint8_t rxBufTail;	/**< uint8_t Ringbuffer-Z�hler */
	uint8_t txBufHead;	/**< uint8_t Ringbuffer-Z�hler */
	uint8_t txBufTail;	/**< uint8_t Ringbuffer-Z�hler */
	
	UART_ERROR lastError:6;	/**< UART_ERROR Empfangsfehlern werden hier gespeichert */
	uint8_t lookForChar; /**< Zu suchendes Byte im Empfangsbuffer */
	
	uint8_t specialCharFound:1; /**< Das zu suchende Byte wurde gefunden. */
	
	volatile USART_t *hw_usart;	/**< USART_t Pointer zum Grundregister der ausgew�hlten, serielle Schnittstelle */
}BUF_UART_t;

/* @brief Frequenzoffset laden (optional)
 *
 * L�dt den Frequenzoffset vom 16MHz / 20MHz Quarz, welcher im
 * Controller gespeichert ist, f�r eine pr�sizere BAUD-Rate berechnung.
 * Beispiel: \n \code{.c}
 * //uart_storeFreqCompValue(SIGROW.OSC16ERR3V);
 * // uart_storeFreqCompValue(SIGROW.OSC16ERR5V);
 * // uart_storeFreqCompValue(SIGROW.OSC20ERR3V);
 * uart_storeFreqCompValue(SIGROW.OSC20ERR5V);
 * \endcode L�dt die von der Fabrik gespeicherten Offset und �bergibt es an die Funktion.
 * @brief SigRowError Fehlerkaliblrierungswert
 */
void uart_storeFreqCompValue(int8_t SigRowError);

/**
 * @brief Serielle Schnittstelle Initialisieren
 *
 * Initialisiert die serielle Schnittstelle, setzt die Flags f�r die Interrupts und
 * stellt die Baudrate in den Registern ein.
 * Beispiel: \n \code{.c}
 * // UART0 ohne MUX mit 19200 Baud (ein Stoppbit, kein Parit�tsbit) initialisieren
 * #define UART_BUFFER_SIZE	32
 * #define UART_BAUD_RATE	19200
 *
 * uint8_t receiveBuffer[UART_BUFFER_SIZE];
 * uint8_t transmitBuffer[UART_BUFFER_SIZE];
 * BUF_UART_t uart_instance;
 * uart_init(&uart_instance, &USART0, UART_BAUD_RATE, SERIAL_8N1, \
 *     receiveBuffer, UART_BUFFER_SIZE, transmitBuffer, UART_BUFFER_SIZE, NORMAL_UART_PINPOS);
 * sei();	// Interrupts Global zulassen
 * \endcode Im Beispielcode wird die Bibliothek mit 19200 Baud auf UART0 ohne MUX initialisiert.
 * @param uartPtr	Pointer zur unbelegten \a BUF_UART_t Instanz
 * @param _usart	Pointer zum Hardware-UART Register
 * @param _baud		Die gew�nschte Baudrate
 * @param _setting	Die gew�nschte Datengr�sse, Parit�ts- und Stoppbiteinstellung
 * @param receivBuf	Pointer zum Empfangsbuffer
 * @param recivBufLen	Gr�sse des Empfangsbuffers
 * @param transBuf	Pointer zum Sendebuffer
 * @param transBufLen	Gr�sse des Sendebuffers
 * @param useMUX	Alternative Pinposition verwenden (1) oder nicht (0)
verwendet.
 */
void uart_init(BUF_UART_t *uartPtr, volatile USART_t *_usart, uint32_t _baud, enum UART_SETTING _setting, uint8_t *recivBuf, const uint8_t recivBufLen, uint8_t *transBuf, const uint8_t transBufLen, enum uart_useMUX useMUX);

void uart_notifyOnChar(BUF_UART_t *uartPtr, uint8_t notifyChar);

/**
 * @brief Ein Byte senden
 *
 * Legt ein Byte im Buffer ab und startet die Sendeinterrupts.
 * Beispiel: \n \code{.c}
 * char data = '0';
 * uart_putc(&uart_instance, data);
 * uart_putc(&uart_instance, 'y');
 * \endcode Sendet ein Byte. Wenn der Buffer voll ist, wird gewartet bis dieser Platz hat.
 * Erfordert das \a uart_init() vorher aufgerufen wurde damit die Funktion korrekt funktioniert.
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @param data		8-Bit Variable
 */
void uart_putc(BUF_UART_t *uartPtr, uint8_t data);

/**
 * @brief Mehrere Bytes senden
 *
 * Sendet eine Anzahl Bytes vom �bergebenden Pointer per serielle Schnittstelle raus.
 * Beispiel: \n \code{.c}
 * uart_send(&uart_instance, &someArray, 12);
 * \endcode Sendet 12 Bytes vom someArray raus.
 * Erfordert das \a uart_init() vorher aufgerufen wurde damit die Funktion korrekt funktioniert.
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @param dataBuffer	Pointer zum Buffer
 * @param bytesToSend	Anzahl zu sendende Bytes aus dem Buffer
 */
void uart_send(BUF_UART_t *uartPtr, uint8_t *dataBuffer, uint8_t bytesToSend);

/**
 * @brief Sendet einen String (Zeichenkette)
 *
 * Die Funktion legt selbstst�ndig die Daten vom String-Bytearray in den Buffer und startet die �bertragung.
 * Beispiel: \n \code{.c}
 * char *MyArray = "This is a Test";
 * uart_puts(&uart_instance, &MyArray);
 * uart_puts(&uart_instance, "Dies ist ein Test\n\r");
 * \endcode In diesem Beispiel wird ein Satz der in einem Bytearray namens "MyArray" gesendet. Falls der
 * Buffer zu klein ist oder nicht genug Platz vorhanden ist, alles auf ein Mal in den Buffer zu kopieren,
 * wird jedes einzelne Zeichen in den Buffer kopiert und gewartet, bis wieder genug freier Speicher f�r ein
 * weiteres Zeichen vorhanden ist.
 * Erfordert das \a uart_init() vorher aufgerufen wurde damit die Funktion korrekt funktioniert.
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @param data		Pointer zum Bytearray
 */
void uart_print(BUF_UART_t *uartPtr, char *data);

/**
 * @brief Sendet einen Zeichenkette, die im Programmspeicher (flash) liegt.
 *
 * Sendet ein Array / Variablen die im Programmspeicher liegt
 * Beispiel: \n \code{.c}
 * uart_puts_p(&uart_instance, variable_text_inProgMem);
 * \endcode
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @param s			Programmspeicher String
 * @see uart_puts_P
 */
void uart_print_p(BUF_UART_t *uartPtr, const char *progmem_s);

/**
 * @brief Sendet einen Zeichenkette, die im Programmspeicher (flash) liegt.
 *
 * Sendet die Zeichenkette, erlaubt jedoch die Nutzung einer Zeichenkette, welche im
 * Programmspeicher liegt. Die Funktion legt den Text im Flash ab und liest diesen, um
 * Arbeitsspeicherschonender zu sein.
 * Beispiel: \n \code{.c}
 * uart_puts_P(&uart_instance, "Dieser Text ist im Programmspeicher");
 * \endcode
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @param s			Programmspeicher String
 */
#define uart_print_P(uartptr,__s)	uart_print_p(uartptr,PSTR(__s))

/* @brief Fehlerspeicher und Bufferstatus auslesen
 * 
 * Gibt den aktuellen Fehlerspeicher zur�ck, aber l�scht ihn nicht. Gibt auch
 * zur�ck, ob Daten im Buffer liegen oder ob er leer ist.
 * Beispiel: \n \code{.c}
 * UART_ERROR processError = uart_rxStatus(&uart_instance);
 * switch(processError)
 * {
	 *    case UART_DATA_AVAILABLE:
	 *       // Daten verf�gbar und alles ok
	 *       break;
	 *    case UART_NO_DATA:
	 *       // Keine Daten verf�gbar
	 *       break;
	 * 	  case UART_FRAME_ERROR:
	 *        // UART Framing Error!
	 *       break;
	 *    case UART_OVERRUN_ERROR:
	 *       // UART Overrun condition Error
	 *       break;
	 *    case UART_PARITY_ERROR:
	 *       // UART Parity Error
	 *       break;
	 *    case UART_BUFFER_OVERFLOW:
	 *       // UART Receive Ringbuffer Overflow
	 *       break;
	 *    default:
	 *       // Mehrere Fehler auf ein Mal
	 *       break;
 * }
 * \endcode In diesem Beispiel wird der Fehlerbuffer �berpr�ft und verarbeitet.
 * Erfordert das \a uart_init() vorher aufgerufen wurde damit die Funktion korrekt funktioniert.
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @return Gibt den Fehlerspeicher und Status zur�ck, siehe \a UART_ERROR
 */
UART_ERROR uart_rxStatus(BUF_UART_t *uartPtr);

/**
 * @brief Ein Byte lesen aber den Pointer/Buffer nicht ver�ndern
 *
 * Sehr �hnlich wie \uart_getc() , jedoch mit dem Unterschied, dass der
 * Buffer nicht aktualisiert wird und der Fehlerspeicher nicht bereinigt wird.
 * Erfordert das \a uart_init() vorher aufgerufen wurde damit die Funktion korrekt funktioniert.
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @return			Gibt das Zeichen (LSB) und die Fehlermeldung (MSB) zur�ck
 */
uint16_t uart_peek(BUF_UART_t *uartPtr);

/**
 * @brief Ein Byte lesen
 *
 * Liest das zuletzt empfangene Byte vom Empfangsbuffer und gibt es zur�ck. Falls der Buffer voll ist 
 * oder es einen Fehler gab, geben die hochwertigen 8 Bits diese zur�ck. Die niederwertigen 8 Bits geben
 * das empfangene Byte zur�ck. Beim Lesevorgang wird der Fehlerspeicher gel�scht.
 * Beispiel: \n \code{.c}
 * uint16_t data = uart_getc(&uart_instance);
 * switch(data >> 8)
 * {
 *    case UART_DATA_AVAILABLE:
 *       // Daten verf�gbar und alles ok
 *       break;
 *    case UART_NO_DATA:
 *       // Keine Daten verf�gbar
 *       break;
 * 	  case UART_FRAME_ERROR:
 *        // UART Framing Error!
 *       break;
 *    case UART_OVERRUN_ERROR:
 *       // UART Overrun condition Error
 *       break;
 *    case UART_PARITY_ERROR:
 *       // UART Parity Error
 *       break;
 *    case UART_BUFFER_OVERFLOW:
 *       // UART Receive Ringbuffer Overflow
 *       break;
 *    default:
 *       // Mehrere Fehler auf ein Mal
 *       break;
 * }
 * \endcode In diesem Beispiel wird 1 Zeichen vom Buffer gelesen.
 * Erfordert das \a uart_init() vorher aufgerufen wurde damit die Funktion korrekt funktioniert.
 * @param uartPtr	Pointer zur initialisierten \a BUF_UART_t Instanz
 * @return			Gibt das Zeichen (LSB) und die Fehlermeldung (MSB) zur�ck
 */
uint16_t uart_getc(BUF_UART_t *uartPtr);

/**
 * @brief �berpr�ft, ob der Sendebuffer voll ist
 *
 * Beispiel: \n\code{.c}
 * if(!(uart_isTxFull(&myUART)))
 * {
 *    uart_putc(&myUART, 0xAA);	// Zeichen senden wenn Buffer nicht voll ist
 * }
 * \endcode In diesem Beispiel wird immer, wenn Platz im Buffer ist, 0xAA gesendet.
 * @param uartPtr Pointer zur initialisierten \a BUF_UART_t Instanz
 * @return Gibt 1 zur�ck wenn der Buffer voll ist, sonst 0
 */
UART_ERROR uart_isTxFull(BUF_UART_t *uartPtr);

/**
 * @brief Spezialzeichen empfangen?
 *
 * �berpr�ft, ob das zu suchende Symbol empfangen wurde
 * Beispiel: \n\code{.c}
 * if(uart_charDetected(&myUART))
 * {
 *    // Iwas empfangen
 * }
 * \endcode 
 * @param uartPtr Pointer zur initialisierten \a BUF_UART_t Instanz
 * @return Gibt die Anzahl FIFO-Bytes im Rx-Buffer bis zum Spezialzeichen zur�ck wenn das Zeichen gefunden wurde, sonst 0
 */
uint8_t uart_charDetected(BUF_UART_t *uartPtr);

/**
 * @brief Zeichen Suchen
 *
 * Empfangsinterrupt nach einem gewissen Zeichen �berpr�fen. Der Empfang des Zeichens kann mit 
 * \a uart_charDetected �berpr�ft werden.
 * Beispiel: \n\code{.c}
 * uart_searchForCharacter(&myUART, '\n');
 * ...
 * if(uart_charDetected(&myUART))
 * {
 *    // Iwas empfangen
 * }
 * \endcode 
 * @param uartPtr Pointer zur initialisierten \a BUF_UART_t Instanz
 * @return Gibt die Anzahl FIFO-Bytes im Rx-Buffer bis zum Spezialzeichen zur�ck wenn das Zeichen gefunden wurde, sonst 0
 */
void uart_searchForCharacter(BUF_UART_t *uartPtr, char charToSearch);

#endif /* UART_H_ */