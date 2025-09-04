/**
 * SystemKernel не только владельц аппаратных ресурсов (UART, SPI и т.д.), но и владелец и планировщик задач.
 *
 */
#pragma once
#include "../resources/UART_Resource.hpp"
#include <stdlib.h>
#include <stdint.h>

enum class TaskPriority {
	LOW,
	NORMAL,
	HIGH,
	CRITICAL // Для прерываний
};


class SystemKernel 
{
private:
	// ВЛАДЕНИЕ: SystemKernel - единоличный владелец этих ресурсов.
	UART_Resource UART_;			// Ресурс UART
	bool isUARTBusy_ = false;		// Флаг занятости ресурса
	
public:
	SystemKernel();
	void initialize();
	
	// Метод для ЗАИМСТВОВАНИЯ ресурса другими компонентами
	UART_Resource& getUART();
	void releaseUART();
};