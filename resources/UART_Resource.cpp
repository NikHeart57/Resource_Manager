#include "UART_Resource.hpp"

/**
 * @brief ����������� UART �������
 * @param[in] config ������������ UART (�� ��������� ������������ ����������� ���������)
 */
UART_Resource::UART_Resource(const Config& config) :
config_(config),				// ������������� ���� config_
isWorking_(false)				// ������������� ���� isWorking_
{
}

/**
 * @brief ���������� UART �������
 * @note ������������� ��������� UART ��� ����������� �������
 */
UART_Resource::~UART_Resource()
{
	if (isWorking_) 
	{
		UART_Disable();
		isWorking_ = false;
	}
}

/**
 * @brief ������ UART � �������� �����������
 */
bool UART_Resource::start()
{
	// ������������� � �������� ����������� config_
	UART_Init(this->config_.state, this->config_.baudRate, this->config_.dataBits,
	this->config_.stopBits, this->config_.parity, this->config_.interrupt);
	
	this->isWorking_ = (this->config_.state == UART_ENABLED);
	return this->isWorking_;
}


/**
 * @brief ������ UART � ��������� ���������
 */
bool UART_Resource::start(uint32_t baudRate)
{
	this->config_.baudRate = baudRate;   // ��������� ������������
	return start();						// �������� ������ ��� ����������
}


/**
 * @brief ������ UART � ��������� �������������
 */
bool UART_Resource::start(const Config& config)
{
	this->config_ = config;				// ��������� ������������
	return start();						// �������� ������ ��� ����������
}

/**
 * @brief ��������� UART 
 * @return true ���� UART ������� ����������, false � ������ ������
 */
bool UART_Resource::stop()
{
	if (!isWorking_) 
	{
		// UART ��� ����������
		return false;
	}
	
	// ��������� UART
	UART_Disable();
	
	// ���������� ���� �����������������
	isWorking_ = false;
	
	// ��������� ��������� � ������������
	config_.state = UART_DISABLED;
	
	return true;
}

/**
 * @brief �������� ����������������� UART
 */
bool UART_Resource::isWorking() const
{
	return this->isWorking_;
}


/**
 * @brief ��������� ���������� ��������� ��� ������ ����
 * @return ���������� ���� � ������ ������
 */
uint8_t UART_Resource::bytesAvailable() const
{
	if (!isWorking_) return 0;
	return UART_DataAvailable();
}

/**
 * @brief ������ ������ ����� �� ������ ������
 * @return ����������� ���� ��� -1 ���� ������ ���
 */
int UART_Resource::read()
{
	if (!this->isWorking_ || !bytesAvailable()) return -1;
	return UART_ReceiveByte();
}

/**
 * @brief ������ ������� ������ �� ������ ������
 * @param[out] buffer ����� ��� ������ ������
 * @param[in] size ������ ������
 * @return ���������� ���������� ����������� ����
 */
size_t UART_Resource::read(uint8_t* buffer, size_t size)
{
	if (!this->isWorking_ || !buffer || size == 0) return 0;
	
	uint8_t available_bytes = bytesAvailable();
	if (available_bytes == 0) return 0;
	
	size_t bytes_to_read = (size < available_bytes) ? size : available_bytes;
	return UART_ReceiveBuffer(buffer, bytes_to_read);
}


/**
     * @brief ������ ������ ����� � UART
     * @param[in] data ���� ��� ��������
     * @return true ���� ������ �������, false � ������ ������
     */
bool UART_Resource::write(uint8_t data)
{
	if (!this->isWorking_) return false;
	
	UART_SendByte(data);
	return true;
}

/**
     * @brief ������ ������� ������ � UART
     * @param[in] data ��������� �� ������ ��� ��������
     * @param[in] size ������ ������ � ������
     * @return ���������� ���������� ���������� ����
     */
size_t UART_Resource::write(const uint8_t* data, size_t size)
{
	if (!this->isWorking_ || !data || size == 0) return 0;
	
	UART_SendBuffer(data, size);
	
	return size;
}

/**
     * @brief ������ ������ � UART
     * @param[in] str ����-��������������� ������
     * @return ���������� ���������� ��������
     */
size_t UART_Resource::print(const char* str)
{
	if (!this->isWorking_ || !str) return 0;
	
	size_t length = 0;
	while (*str)
	{
		UART_SendByte(*str++);
		length++;
	}
	
	return length;
}

/**
     * @brief ������ ������ � ��������� ������
     * @param[in] str ����-��������������� ������
     * @return ���������� ���������� �������� (������� \r\n)
     */
size_t UART_Resource::println(const char* str)
{
	size_t length = print(str);
	length += print("\r\n");
	return length;
}


/**
     * @brief ������� ������ ������
     */
void UART_Resource::flush()
{
	if (this->isWorking_)
	{
		UART_FlushRxBuffer();
	}
}

/**
     * @brief ��������� �������� ��������
     * @param[in] baudRate �������� � �����
     * @return true ���� �������� ������� ��������, false � ������ ������
     */
bool UART_Resource::setBaudRate(uint32_t baudRate)
{
	if (!isWorking_) return false;
	
	// ���� �������� �� ����������, ������ �� ������
	if (config_.baudRate == baudRate) 
	{
		return true;
	}
	
	config_.baudRate = baudRate;
	UART_SetBaudRate(baudRate);
	return true;
}

/**
     * @brief ��������� ������� ������������ UART
     * @return ������� ������������
     */
UART_Resource::Config UART_Resource::getConfig() const
{
	return this->config_;
}

/**
     * @brief ��������� callback-������� ��� ������������ ������
     * @param[in] callback ������� ��������� ������
     */
void UART_Resource::setRxCallback(UART_RxCallback callback)
{
	if (this->isWorking_)
	{
		UART_SetRxCallback(callback);
	}
}
