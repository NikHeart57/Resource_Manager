#include "SystemKernel.hpp"


SystemKernel::SystemKernel() : UART_(UART_Resource::Config{}) // Инициализируем ресурс здесь
{
	
}

void SystemKernel::initialize() 
{
	// Владелец инициализирует свои ресурсы
	UART_.start(57600);
	UART_.println("System Kernel initialized");
}



UART_Resource& SystemKernel::getUART() 
{
	if (isUARTBusy_) 
	{
		// Можно вернуть ссылку на "заглушку", бросить исключение,
		// или подождать освобождения в цикле
		// Пока просто вернем реальный ресурс, но логируем ошибку
		UART_.println("ERROR: UART is already busy!");
	}
	isUARTBusy_ = true;
	return UART_;
}





void SystemKernel::releaseUART() 
{
	isUARTBusy_ = false;
}