#include "main.hpp"

int main(void)
{
	SystemKernel system;		// Создаем ядро
	system.initialize();		// Инициализируем ресурсы

	Application app(system);	// Создаем приложение, передаем ему ядро
	app.run();					// Запускаем прикладную логику

	return 1; 
}