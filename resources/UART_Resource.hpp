/**
 * @note        ����������� ������������ ���������� deepseek 29.08.25
 *
 * @file        UART_Resource.hpp
 * @brief       ������ UART - ��������������� ���������� ��� UART ���������
 * @details     ������������� ��������-��������������� ��������� ��� ������ � UART,
 *              ������������ �������������� ������� � ������������ ������� API
 *              � ������������ � ���������� ���.
 * 
 * @note        �����������:
 *              - ������ ������������ ���������� ����������
 *              - ��������� ��������� ������������ UART
 *              - ������������� ��������� ��� ������/������
 *              - Callback-�������� ��� ������������ ������
 * 
 * @warning     ��� ������ ������� ��������������� ������������� �������� UART
 * 
 *
 *
 * @section     ��������� ������
 * 
 * @struct      UART_Resource::Config
 * @brief       ��������� ������������ ���������� UART
 * @var         Config::state
 *              ��������� UART (�������/��������)
 * @var         Config::dataBits
 *              ���������� ��� ������ (5-9)
 * @var         Config::stopBits  
 *              ���������� ����-����� (1-2)
 * @var         Config::parity
 *              �������� ��������
 * @var         Config::interrupt
 *              ����� ����������
 * @var         Config::baudRate
 *              �������� �������� (���)
 * 
 *
 *
 * @section     ��������� ������ ������
 *
 * @fn					UART_Resource(const Config& config)				����������� � �������������	
 * @fn				   ~UART_Resource()									���������� (������������� ��������� UART)
 *
 *
 *
 * @section     ������ ���������/����������
 *
 * @fn            bool	start()											������ UART � �������� �����������	
 * @fn            bool	start(uint32_t baudRate)						������ UART � ��������� ���������
 * @fn            bool	start(const Config& config)						������ UART � ��������� �������������
 * @fn			  bool	stop()											��������� UART
 * @fn            bool	isWorking() const								�������� ������� �� UART
 *
 *
 *
 * @section     ������ ������/������
 *
 * @fn         uint8_t  bytesAvailable() const							���������� ���������� ����, ��������� ��� ������ � ������ ������ UART
 * @fn             int	read()											������ ������ �����
 * @fn          size_t	read(uint8_t* buffer, size_t size)				������ ������� ������
 * @fn            bool	write(uint8_t data)								������ ������ �����
 * @fn          size_t	write(const uint8_t* data, size_t size)			������ ������� ������
 * @fn          size_t	print(const char* str)							������ ������ 
 * @fn          size_t	println(const char* str)						������ ������ � ��������� ������
 * @fn            void	flush()											������� ������ ������
 *
 *
 *
 * @section     ��������� ������ 
 *
 * @fn            bool	setBaudRate(uint32_t baudRate)					��������� �������� ��������
 * @fn          Config	getConfig() const								��������� ������� ������������
 * @fn			  void	setRxCallback(UART_RxCallback callback)			��������� callback-������� ��� ������������ ������
 * 
 *
 *
 * @section     ��������� ����
 * 
 * @var         UART_Resource::config_				������� ������������ UART
 * @var         UART_Resource::isWorking_			���� ����������������� UART
 * 
 *
 *
 * @example     ������ �������������:
 * @code
 * // ������ �������� ��������� � UART
 * UART_Resource uart;
 * uart.start(115200);
 * uart.println("Hello World!");
 * 
 * // ������ ������������ ������
 * void myCallback(char c) { ## ��������� ## }
 * uart.setRxCallback(myCallback);
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

#include <stdint.h>
#include <stddef.h>
#include "../drivers/UART/UART.h"

/**
 * @class UART_Resource
 * @brief �����-������ ��� ������ � UART �����������
 * 
 * ������������� ���������������� UART, ������������ ��������������� API
 * ��� ���������� ���������������� ������ ����������������.
 */
class UART_Resource
{
public:
    /**
     * @struct Config
     * @brief ��������� ������������ ���������� UART
     */
    struct Config
    {
        UART_State state;           ///< ��������� UART (�������/��������)
        UART_DataBits dataBits;     ///< ���������� ��� ������ (5-9)
        UART_StopBits stopBits;     ///< ���������� ����-����� (1-2)
        UART_Parity parity;         ///< �������� ��������
        UART_Interrupt interrupt;   ///< ����� ����������
        uint32_t baudRate;          ///< �������� �������� (���)
        
        /**
         * @brief ����������� �� ���������
         * @details �������������� ��������� ���������� �� ���������:
         * - 8 ��� ������, 1 ����-���, ��� �������� ��������
         * - ���������� ������ ��������, �������� 9600 ���
         */
        Config() :
            state(UART_ENABLED),
            dataBits(UART_8_BITS),
            stopBits(UART_1_STOP_BIT),
            parity(UART_PARITY_NONE),
            interrupt(UART_INTERRUPT_RX_ENABLED),
            baudRate(9600)
        {}
    };

public:    
    /**
     * @brief ����������� UART �������
     * @param[in] config ������������ UART (�� ��������� ������������ ����������� ���������)
     */
    explicit UART_Resource(const Config& config = Config());
    
    /**
     * @brief ���������� UART �������
     * @note ������������� ��������� UART ��� ����������� �������
     */
    ~UART_Resource();
    
    // ������ ����������� � �����������
    UART_Resource(const UART_Resource&) = delete;
    UART_Resource& operator=(const UART_Resource&) = delete;
    UART_Resource(UART_Resource&&) = delete;
    UART_Resource& operator=(UART_Resource&&) = delete;
    
public:
    /**
     * @brief ������ UART � �������� �����������
     * @return true ���� UART ������� �������, false � ������ ������
     */
    bool start();
    
    /**
     * @brief ������ UART � ��������� ���������
     * @param[in] baudRate �������� �������� � �����
     * @return true ���� UART ������� �������, false � ������ ������
     */
    bool start(uint32_t baudRate);
    
    /**
     * @brief ������ UART � ��������� �������������
     * @param[in] config ������������ UART
     * @return true ���� UART ������� �������, false � ������ ������
     */
    bool start(const Config& config);
	
	/**
     * @brief ��������� UART 
     * @return true ���� UART ������� ����������, false � ������ ������
     */
    bool stop();
    
    /**
     * @brief �������� ����������������� UART
     * @return true ���� UART �������� � ����� � ������ �������
     */
    bool isWorking() const;
    
    /**
     * @brief ��������� ���������� ��������� ��� ������ ����
     * @return ���������� ���� � ������ ������
     */
    uint8_t bytesAvailable() const;
    
    /**
     * @brief ������ ������ ����� �� ������ ������
     * @return ����������� ���� ��� -1 ���� ������ ���
     */
    int read();
    
    /**
     * @brief ������ ������� ������ �� ������ ������
     * @param[out] buffer ����� ��� ������ ������
     * @param[in] size ������ ������
     * @return ���������� ���������� ����������� ����
     */
    size_t read(uint8_t* buffer, size_t size);
    
    /**
     * @brief ������ ������ ����� � UART
     * @param[in] data ���� ��� ��������
     * @return true ���� ������ �������, false � ������ ������
     */
    bool write(uint8_t data);
    
    /**
     * @brief ������ ������� ������ � UART
     * @param[in] data ��������� �� ������ ��� ��������
     * @param[in] size ������ ������ � ������
     * @return ���������� ���������� ���������� ����
     */
    size_t write(const uint8_t* data, size_t size);
    
    /**
     * @brief ������ ������ � UART
     * @param[in] str ����-��������������� ������
     * @return ���������� ���������� ��������
     */
    size_t print(const char* str);
    
    /**
     * @brief ������ ������ � ��������� ������
     * @param[in] str ����-��������������� ������
     * @return ���������� ���������� �������� (������� \r\n)
     */
    size_t println(const char* str);
    
    /**
     * @brief ������� ������ ������
     */
    void flush();
    
    /**
     * @brief ��������� �������� ��������
     * @param[in] baudRate �������� � �����
     * @return true ���� �������� ������� ��������, false � ������ ������
     */
    bool setBaudRate(uint32_t baudRate);
    
    /**
     * @brief ��������� ������� ������������ UART
     * @return ������� ������������
     */
    Config getConfig() const;
    
    /**
     * @brief ��������� callback-������� ��� ������������ ������
     * @param[in] callback ������� ��������� ������
     */
    void setRxCallback(UART_RxCallback callback);
    
private:
    Config config_;     ///< ������� ������������ UART
    bool isWorking_;    ///< ���� ����������������� UART
};