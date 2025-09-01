#include "Application.hpp"

Application::Application(SystemKernel& system) : system_(&system) // Берем адрес ссылки
{
}

void Application::run()
{
	// Переменные приложения
	SharedBuffer buffer;

	auto& uart = system_->getUART();	// Запрашиваем ресурс
	uart.println("Hello");				// Используем его
	system_->releaseUART();				// Возвращаем ресурс ядру
	
	while(true)
	{
	}
}