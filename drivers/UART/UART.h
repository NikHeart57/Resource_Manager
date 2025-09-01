/**
 * @note        ����������� ������������ ���������� deepseek 29.08.25
 *
 * @file        UART.h
 * @brief       ������� UART ��� AVR ATmega32 - �������������� ����������
 * @details     ������������� ������ ����� ������� ��� ����������� ���������������� �����
 *              � ���������� ��������� ������� � callback-�������.
 * 
 * @note        �����������:
 *              - ��������� ��������� �� 115200 ���
 *              - ��������� ������ ������/�������� (64 �����)
 *              - ������������� ��������� ����� (��������, ��������, ����-����)
 *              - ������ ���������� ��� ������/��������
 *              - Callback-�������� ��� ������������ ������
 * 
 * @warning     �����������:
 *              - ����������� ������� ������ (������� ��������� � �������� �����)
 *              - ������������ ������ ������� (64 �����)
 * 
 * @attention   ��� ������ ����������:
 *              - ����������� <avr/io.h> � <avr/interrupt.h>
 *              - ���������� ��������� F_CPU
 *              - ���������� ���������� ���������� (sei())
 * 
 *
 *
 * @section     ���������������� define
 * 
 * @def         F_CPU
 *              ������� ������� ���������������� (��) - ������������ ��������
 * @note        ������ ���� ���������� ����� ���� �����������: -DF_CPU=16000000UL
 * @warning     ������������ �������� �������� � �������� �������� UART!
 *
 * @def         UART_RX_BUFFER_SIZE
 *              ������ ������ ������ (64 �����)
 *
 * @def         UART_TX_BUFFER_SIZE  
 *              ������ ������ �������� (64 �����)
 * 
 *
 *
 * @section     ������������ (enum)
 * 
 * @enum        UART_State
 *              ��������� UART (���/����)
 * @value       UART_DISABLED
 *              UART �������� (����������������)
 * @value       UART_ENABLED
 *              UART ������� � ����� � ������
 * 
 * @enum        UART_DataBits
 *              ������ �������� ������
 * @value       UART_5_BITS
 *              5 ��� ������ (����� ������������)
 * @value       UART_6_BITS
 *              6 ��� ������
 * @value       UART_7_BITS  
 *              7 ��� ������ (��� ASCII)
 * @value       UART_8_BITS
 *              8 ��� ������ (����������� �����)
 * @value       UART_9_BITS
 *              9 ��� ������ (������ �����)
 * 
 * @enum        UART_StopBits
 *              ���������� ����-�����
 * @value       UART_1_STOP_BIT
 *              1 ����-��� (��������)
 * @value       UART_2_STOP_BITS
 *              2 ����-���� (��� ������� �����)
 * 
 * @enum        UART_Parity
 *              ������ �������� ��������
 * @value       UART_PARITY_NONE
 *              ��� �������� ��������
 * @value       UART_PARITY_EVEN
 *              ������ �������
 * @value       UART_PARITY_ODD
 *              �������� �������
 * 
 * @enum        UART_Interrupt
 *              ������ ���������� UART
 * @value       UART_INTERRUPT_DISABLED
 *              ���������� ���������
 * @value       UART_INTERRUPT_RX_ENABLED
 *              ���������� �� ������
 * @value       UART_INTERRUPT_TX_ENABLED
 *              ���������� �� ���������� �����������
 * @value       UART_INTERRUPT_BOTH_ENABLED
 *              ��� ���������� ��������
 * 
 *
 *
 * @section     ���� ������
 * 
 * @typedef     UART_RxCallback
 *              ��� callback-������� ��� ��������� �������� ������
 * @param       c �������� ������ (1 ���� ������)
 * @note        ������� ���������� �� ���������� ��� ��������� ������� �������
 * @warning     ������ ���� ����������� ������� (�� ����������� ����������)
 * 
 *
 *
 * @section     ������� �������������
 * 
 * @fn			  void	UART_Init(UART_State state, uint32_t baud, UART_DataBits dataBits, 
 *								  UART_StopBits stopBits, UART_Parity parity, 
 *								  UART_Interrupt interrupt)						������������� UART
 * @fn            void	UART_Init(UART_State state)								������� ������������� UART � ����������� �� ���������
 * @fn            void	UART_Enable(void)										��������� ����������������� UART
 * @fn            void	UART_Disable(void)										���������� ����������������� UART
 * @fn            void	UART_SetBaudRate(uint32_t baud)							��������� �������� �������� (�������)
 * 
 *
 *
 * @section     ������� �������� ������
 * 
 * @fn            void	UART_SendByte(uint8_t byte)								���������� �������� ������ �����
 * @fn            void	UART_SendBuffer(const uint8_t *buffer, uint16_t length)	�������� ��������� ������� ������
 * 
 *
 *
 * @section     ������� ������ ������
 * 
 * @fn            char	UART_ReceiveByte(void)									���������� ����� ������ �������
 * @fn         uint8_t	UART_ReceiveBuffer(uint8_t *buffer, uint16_t length)	����� ������� ������ � �����
 * @fn         uint8_t	UART_DataAvailable(void)								�������� ������� ������ � ������ ������
 * @fn            void	UART_FlushRxBuffer(void)								������� ������ ������
 * @fn            void	UART_SetRxCallback(UART_RxCallback callback)			��������� callback-������� ��� ������������ ������
 * 
 *
 *
 * @section     ������� �������������
 * 
 * @example     ������� �������������:
 * @code
 * UART_Init(UART_ENABLED, 9600, UART_8_BITS, UART_1_STOP_BIT, 
 *          UART_PARITY_NONE, UART_INTERRUPT_RX_ENABLED);
 * UART_SendByte(12);
 * @endcode
 * 
 * @example     ������������� callback:
 * @code
 * void my_callback(char data) {
 *     // ��������� ����������� �������
 * }
 * 
 * UART_SetRxCallback(my_callback);
 * @endcode
 * 
 *
 *
 * @author      ������� ������
 * @date        2025-08-29
 * @version     1.0
 * 
 * @copyright   MIT License
 */


#pragma once
//////////////////////////////////////////////////////////////////////////
//  ��������� ������������ �����
//////////////////////////////////////////////////////////////////////////

#include <avr/io.h>          ///< �������� �����-������ AVR
#include <avr/interrupt.h>   ///< ������ � ������������
#include <stddef.h>          ///< ����������� ����������� �������� �����


//////////////////////////////////////////////////////////////////////////
//  �������� ��������� C++11
//////////////////////////////////////////////////////////////////////////

#if __cplusplus < 201103L
#error "C++11 support required! Add -std=c++11 to compiler flags"
#endif


//////////////////////////////////////////////////////////////////////////
//  �������� ��������� ���������������� (ATmega32 ONLY)
//////////////////////////////////////////////////////////////////////////

#ifndef __AVR_ATmega32__
#error "UART driver requires ATmega32 MCU! Check your target device."
#endif


//////////////////////////////////////////////////////////////////////////
//  �������� ��������� UART
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ������� ������� ���������������� (��) - ������ ���� ����������
 * @details ������� CPU �������� ��� ������� ��������� � ��������� UART.
 * 
 * @note ������������ ����� ���� �����������: -DF_CPU=16000000UL
 * @note ��� ���� ����� ���������� ����� ���������: #define F_CPU 16000000UL
 * 
 * @warning ������ ����� ��������������� �������� ������� ������/����������!
 * @warning ������������ �������� �������� � �������� �������� UART!
 * 
 * @example ���������� �����������:
 * @code
 * // � ���������� �����������:
 * -DF_CPU=16000000UL
 * 
 * // ��� � ���� (����� #include "UART.h"):
 * #define F_CPU 16000000UL
 * @endcode
 * 
 * @error ���������� ��������� � ����������, ���� F_CPU �� ����������
 */
#ifndef F_CPU
#error "UART.h - F_CPU must be defined! Add F_CPU=16000000UL to compiler flags"
#endif

/**
 * @brief ������ ������ ������ (����)
 * @note ������������� ������� ������ (64, 128, 256)
 * @warning ����������� ����������� RAM
 */
#define UART_RX_BUFFER_SIZE 64

/**
 * @brief ������ ������ �������� (����)
 * @note ������������� ������� ������ (64, 128, 256)
 * @warning ����������� ����������� RAM
 */
#define UART_TX_BUFFER_SIZE 64

//////////////////////////////////////////////////////////////////////////
//  ������������ UART (������������)
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ��������� UART (���/����)
 * @note ����������� ������ RXEN/TXEN � �������� UCSRB
 */
typedef enum {
    UART_DISABLED = 0,  ///< UART �������� (����������������)
    UART_ENABLED         ///< UART ������� � ����� � ������
} UART_State;

/**
 * @brief ������ �������� ������
 * @note ��� 9-������� ������ ��������� ������������� ���������� ��� UCSZ2
 */
typedef enum {
    UART_5_BITS = 0,  ///< 5 ��� ������ (����� ������������)
    UART_6_BITS,      ///< 6 ��� ������
    UART_7_BITS,      ///< 7 ��� ������ (��� ASCII)
    UART_8_BITS,      ///< 8 ��� ������ (����������� �����)
    UART_9_BITS = 7   ///< 9 ��� ������ (������ �����)
} UART_DataBits;

/**
 * @brief ���������� ����-�����
 * @note ����������� ��������� ���������� 1 ����-���
 */
typedef enum {
    UART_1_STOP_BIT = 0,  ///< 1 ����-��� (��������)
    UART_2_STOP_BITS       ///< 2 ����-���� (��� ������� �����)
} UART_StopBits;

/**
 * @brief ������ �������� ��������
 * @note ������������ ��� ����������� ������ ��������
 */
typedef enum {
    UART_PARITY_NONE = 0,  ///< ��� �������� ��������
    UART_PARITY_EVEN = 2,   ///< ������ �������
    UART_PARITY_ODD = 3     ///< �������� �������
} UART_Parity;

/**
 * @brief ������ ���������� UART
 * @note ��������� ������������ ������� � ������� ������
 */
typedef enum {
    UART_INTERRUPT_DISABLED = 0,    ///< ���������� ���������
    UART_INTERRUPT_RX_ENABLED,      ///< ���������� �� ������
    UART_INTERRUPT_TX_ENABLED,      ///< ���������� �� ���������� �����������
    UART_INTERRUPT_BOTH_ENABLED     ///< ��� ���������� ��������
} UART_Interrupt;

//////////////////////////////////////////////////////////////////////////
//  CALLBACK-�������� ��� ������������ ������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ��� callback-������� ��� ��������� �������� ������
 * @param[in] c �������� ������ (1 ���� ������)
 * @note ������� ���������� �� ���������� ��� ��������� ������� �������
 * @warning ������ ���� ����������� ������� (�� ����������� ����������)
 * @example 
 * void my_callback(char data) {
 *     // ��������� ����������� �������
 * }
 */
typedef void (*UART_RxCallback)(char c);

//////////////////////////////////////////////////////////////////////////
//  �������� ��������� UART API
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ����������� ������������� UART
 * @param[in] state ��������� (UART_ENABLED/UART_DISABLED)
 * @param[in] baud �������� � ����� (300-115200)
 * @param[in] dataBits ���������� ��� ������ (5-9 ���)
 * @param[in] stopBits ���������� ����-����� (1 ��� 2)
 * @param[in] parity �������� �������� (NONE/EVEN/ODD)
 * @param[in] interrupt ����� ���������� (RX/TX/BOTH)
 * @note ��� 9-������� ������ ������������� ��������������� ��� UCSZ2
 * @warning ������������ ��������� ����� �������� � ������� �����!
 */
void UART_Init(UART_State state, uint32_t baud, UART_DataBits dataBits, UART_StopBits stopBits, UART_Parity parity, UART_Interrupt interrupt);

/**
 * @brief ������� ������������� UART � ����������� �� ���������
 * @param[in] state ��������� (UART_ENABLED/UART_DISABLED)
 * @details ������������ ���������:
 * - 8 ��� ������
 * - 1 ����-���
 * - ��� �������� ��������
 * - ���������� ������ ��������
 * - �������� 9600 ���
 */
void UART_Init(UART_State state);

/**
 * @brief ��������� ����������������� UART
 * @note ���������� ���� RXEN � TXEN � �������� UCSRB
 */
void UART_Enable(void);

/**
 * @brief ���������� ����������������� UART
 * @note ������������ ���� RXEN � TXEN � �������� UCSRB
 * @warning ������������� ��� ������� ��������
 */
void UART_Disable(void);

/**
 * @brief ��������� �������� �������� (�������)
 * @param[in] baud �������� � ����� (300-115200)
 * @note ������������� �������� UBRR �� �������:
 *       UBRR = (F_CPU / (16 * baud)) - 1
 * @warning �� ������ ������� ��������� ������� �����
 */
void UART_SetBaudRate(uint32_t baud);

//////////////////////////////////////////////////////////////////////////
//  ������� �������� ������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ���������� �������� ������ �����
 * @param[in] byte ���� ��� �������� (1 ����)
 * @note ��������� ���������� �� ���������� �����������
 * @warning ��� 9-������� ������ ������� ��������� ���������
 */
void UART_SendByte(uint8_t byte);

/**
 * @brief �������� ��������� ������� ������
 * @param[in] buffer ��������� �� ����� ������
 * @param[in] length ���������� ���� ��� ��������
 * @note ��������� �������� � ��������� ������� (������� ������� �����)
 */
void UART_SendBuffer(const uint8_t *buffer, uint16_t length);

//////////////////////////////////////////////////////////////////////////
//  ������� ������ ������
//////////////////////////////////////////////////////////////////////////

/**
 * @brief ���������� ����� ������ �����
 * @return �������� ������ (1 ����)
 * @note ��������� ���������� �� ��������� ������
 * @warning ��� 9-������� ������ ������� ��������� ���������
 */
char UART_ReceiveByte(void);

/**
 * @brief ����� ������� ������ � �����
 * @param[out] buffer ����� ��� ������ �������� ������
 * @param[in] length ������������ ���������� ���� ��� ������
 * @return ���������� ���������� �������� ����
 * @note ������ ��� ��������� ������ (�� �� ����� length ����)
 */
uint8_t UART_ReceiveBuffer(uint8_t *buffer, uint16_t length);

/**
 * @brief �������� ������� ������ � ������ ������
 * @return ���������� ��������� ��� ������ ����
 * @note ������������� �������
 */
uint8_t UART_DataAvailable(void);

/**
 * @brief ������� ������ ������
 * @note ���������� ������� ������ � ������ ������
 * @warning ������� ��� ������������� ������!
 */
void UART_FlushRxBuffer(void);

/**
 * @brief ��������� callback-������� ��� ������������ ������
 * @param[in] callback ������� ��������� ������ (NULL ��� ����������)
 * @note ��� ��������� callback ����������� ����� �� ������������
 * @warning Callback ����������� � ��������� ����������!
 */
void UART_SetRxCallback(UART_RxCallback callback);