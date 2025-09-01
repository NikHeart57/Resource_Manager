#pragma once

#include "../core/SystemKernel.hpp"
class SystemKernel; // Forward declaration

class Application 
{
	private:
	SystemKernel* system_; // Указатель вместо ссылки
	long int counter_ = 0;
	union SharedBuffer 
	{
		char as_char[32];      // Для строк (itoa)
		uint8_t as_uint8[32];  // Для сырых байт
		uint32_t as_uint32[8]; // Для 32-битных значений
		float as_float[8];     // Для чисел с плавающей точкой
	};

	public:
	explicit Application(SystemKernel& system);
	void run(); // Сюда переносим основной цикл приложения
};