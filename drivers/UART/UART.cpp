#include "UART.h"

//////////////////////////////////////////////////////////////////////////
//  ��������� ������ ��� ������ � ��������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ����� ������ ������ (���������)
 * @note ������ ������������ UART_RX_BUFFER_SIZE
 * @warning �������� ��� volatile ��� ������ � �����������
 */
static volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

/**
 * @brief ����� �������� ������ (���������)
 * @note ������ ������������ UART_TX_BUFFER_SIZE
 * @warning �������� ��� volatile ��� ������ � �����������
 */
static volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];

/**
 * @brief ������ ������ (������) ������ ������
 * @note ��������� �� ��������� ������� ��� ������
 */
static volatile uint8_t uart_rx_head = 0;

/**
 * @brief ������ ������ (������) ������ ������
 * @note ��������� �� ��������� ������� ��� ������
 */
static volatile uint8_t uart_rx_tail = 0;

/**
 * @brief ������ ������ (������) ������ ��������
 * @note ��������� �� ��������� ������� ��� ������
 */
static volatile uint8_t uart_tx_head = 0;

/**
 * @brief ������ ������ (������) ������ ��������
 * @note ��������� �� ��������� ������� ��� ������
 */
static volatile uint8_t uart_tx_tail = 0;

//////////////////////////////////////////////////////////////////////////
//  CALLBACK-������� ��� ������������ ������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief Callback-������� ��� ��������� �������� ������
 * @note ���� �����������, ���������� ������ �����������
 * @warning ������ ���� ������� (����������� � ����������)
 */
static UART_RxCallback uart_rx_callback = NULL;

//////////////////////////////////////////////////////////////////////////
//  �������� ������� UART
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ����������� ������������� UART
 * @param state ��������� UART (���/����)
 * @param dataBits ���������� ��� ������
 * @param stopBits ���������� ����-�����
 * @param parity �������� ��������
 * @param interrupt ����� ����������
 * 
 * @details �������� �������������:
 * 1. ������ � ��������� �������� ������� (UBRR)
 * 2. ��������� ������� ����� (UCSRC)
 * 3. ��������� ���������/����������� (UCSRB)
 * 4. ��������� ���������� �������� ����������
 */
void UART_Init(UART_State state, uint32_t baud, UART_DataBits dataBits, UART_StopBits stopBits, UART_Parity parity, UART_Interrupt interrupt)
{
    // 1. ������ � ��������� �������� �������
    UART_SetBaudRate(baud);
    
    // 2. ��������� ������� ����� (��������, ����-����, ��������)
    UCSRC = (1 << URSEL) | (parity << UPM0) | (stopBits << USBS) | (dataBits << UCSZ0);
    
    // 3. ��������� ��������� � �����������
    UCSRB = (1 << RXEN) | (1 << TXEN);
    
    // 4. �������������� ��������� ��� 9-������� ������
    if(dataBits == UART_9_BITS)
    {
        UCSRB |= (1 << UCSZ2);
    }
    
    // 5. ��������� ���������� �������� ���������� ������
    switch(interrupt) 
	{
        case UART_INTERRUPT_RX_ENABLED:
            UCSRB |= (1 << RXCIE);
            break;
        case UART_INTERRUPT_TX_ENABLED:
            UCSRB |= (1 << TXCIE);
            break;
        case UART_INTERRUPT_BOTH_ENABLED:
            UCSRB |= (1 << RXCIE) | (1 << TXCIE);
            break;
        default:
            // ���������� ���������
            break;
    }
    
    // 6. ���������/���������� UART �������� ���������
    if(state == UART_DISABLED) {
        UART_Disable();
    }
}

/**
 * @brief ���������� ������������� UART
 * @param state ��������� UART (���/����)
 * @note ���������� ��������� �� ���������:
 * - 8 ��� ������, 1 ����-���, ��� �������� ��������
 * - �������� 9600 ���
 * - ���������� ������ ��������
 */
void UART_Init(UART_State state)
{
    if(state == UART_ENABLED) 
	{
        // 9600 ���
        UART_SetBaudRate(9600);
        
        // ��������� ������� �����: 8N1
        UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
        
        // ��������� ���������, ����������� � ���������� ������
        UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
    } 
	else 
	{
        // ������ ���������� UART
        UCSRB = 0;  // ��������� �����, �������� � ����������
    }
}

/**
 * @brief ��������� UART
 * @note ���������� ���� RXEN � TXEN � �������� UCSRB
 */
void UART_Enable(void) 
{
    UCSRB |= (1 << RXEN) | (1 << TXEN);
}

/**
 * @brief ���������� UART
 * @note ������������ ���� RXEN � TXEN � �������� UCSRB
 * @warning ��������� ��� ������� �������� ��������/������
 */
void UART_Disable(void) 
{
    UCSRB &= ~((1 << RXEN) | (1 << TXEN));
}

/**
 * @brief ��������� �������� ��������
 * @param baud �������� � ����� (300-115200)
 * @note ������������� �������� UBRR ��� �������� ��������
 */
void UART_SetBaudRate(uint32_t baud) 
{
    uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;
}

//////////////////////////////////////////////////////////////////////////
//  ������� �������� ������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief �������� ������ �����
 * @param byte ���� ��� ��������
 * @note ��������� ���������� �� ���������� �����������
 */
void UART_SendByte(uint8_t byte) 
{
    while(!(UCSRA & (1 << UDRE)));		// �������� ����������
    UDR = byte;							// ������ ������
}


/**
 * @brief �������� ��������� �������
 * @param buffer ��������� �� ������
 * @param length ����� ������ � ������
 */
void UART_SendBuffer(const uint8_t *buffer, uint16_t length) 
{
    while(length--) {
	    UART_SendByte(*buffer++);
    }
}

//////////////////////////////////////////////////////////////////////////
//  ������� ������ ������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ���������� ����� ������ �����
 * @return �������� ������ (1 ����)
 * @note ��������� ���������� �� ��������� ������
 * @warning ��� 9-������� ������ ������� ��������� ���������
 */
char UART_ReceiveByte(void) 
{
    while(!UART_DataAvailable());  // �������� ������
    uint8_t data = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
    return data;
}

/**
 * @brief ����� ������� ������
 * @param buffer ����� ��� ������
 * @param length ������������ ����� ������
 * @return ���������� ���������� �������� ����
 */
uint8_t UART_ReceiveBuffer(uint8_t *buffer, uint16_t length) 
{
    uint16_t i;
    for(i = 0; i < length && UART_DataAvailable(); i++) {
        buffer[i] = uart_rx_buffer[uart_rx_tail];
        uart_rx_tail = (uart_rx_tail + 1) % UART_RX_BUFFER_SIZE;
    }
    return i;
}

/**
 * @brief �������� ������� ������ � ������
 * @return ���������� ��������� ���� (0 ���� ����� ����)
 */
uint8_t UART_DataAvailable(void) 
{
    return (uart_rx_head != uart_rx_tail);
}

/**
 * @brief ������� ������ ������
 * @note ���������� ������� � ������� ��� ������������� ������
 */
void UART_FlushRxBuffer(void) 
{
    uart_rx_head = uart_rx_tail = 0;
}

/**
 * @brief ��������� callback-������� ��� ������
 * @param callback ������� ��������� ������
 * @note NULL ��������� callback � �������� �����������
 */
void UART_SetRxCallback(UART_RxCallback callback)
{
    uart_rx_callback = callback;
}

//////////////////////////////////////////////////////////////////////////
//  ����������� ����������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ���������� ���������� ������
 * @note ���������� ��� ��������� ������� �����
 */
ISR(USART_RXC_vect)
{
    char c = UDR;  // ������ ����������� �����
    
    if (uart_rx_callback != NULL)
    {
        // ����� callback-������� (���� �����������)
        uart_rx_callback(c);
    }
    else
    {
        // ����������� ��������� - ������ � ��������� �����
        uint8_t next_head = (uart_rx_head + 1) % UART_RX_BUFFER_SIZE;
        if (next_head != uart_rx_tail)  // �������� ������������
        {
            uart_rx_buffer[uart_rx_head] = c;
            uart_rx_head = next_head;
        }
    }
}

/**
 * @brief ���������� ���������� ��������
 * @note ���������� ��� ���������� �����������
 */
ISR(USART_TXC_vect) 
{
    if (uart_tx_head != uart_tx_tail)
    {
        // �������� ���������� ����� �� ������
        UDR = uart_tx_buffer[uart_tx_tail];
        uart_tx_tail = (uart_tx_tail + 1) % UART_TX_BUFFER_SIZE;
    }
}