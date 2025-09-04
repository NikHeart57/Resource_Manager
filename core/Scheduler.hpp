#pragma once
#include "Task.hpp"

namespace SimpleRTOS {

	class Scheduler {
	private:
		static const uint8_t MAX_TASKS = 10;    // Максимальное количество задач
		Task taskPool[MAX_TASKS];				// Массив объектов задач
		Task* taskList[MAX_TASKS];				// Массив указателей на задачи  
		uint8_t taskCount = 0;                  // Текущее количество зарегистрированных задач
		uint32_t systemTicks = 0;               // Счетчик системных тиков
		Task* currentTask = nullptr;            // Указатель на задачу, которая выполняется в данный момент
		
	public:	
		// Конструктор объявлен, но будет определен в .cpp файле
		Scheduler();
		~Scheduler() = default;

		
	public:
		// Удаляем конструкторы копирования и присваивания
		Scheduler(const Scheduler&) = delete;
		void operator=(const Scheduler&) = delete;

		// Статический метод для получения единственного экземпляра планировщика
		static Scheduler& getInstance();

		// Инициализация планировщика
		void init();

		// Главный диспетчер
		void dispatch();

		// Регистрация новой задачи в планировщике
		bool addTask(TaskFunction function, TaskPriority priority);

		// Изменение состояния задачи (более универсально, чем stop/run)
		void suspendTask(TaskFunction function);
		void resumeTask(TaskFunction function);

		// Системный тик. Вызывается из прерывания таймера!
		void tick();

		// Функция для перевода задачи в режим сна на указанное время
		void sleep(uint32_t ticksToSleep);

		// Получить текущее системное время
		uint32_t getSystemTicks() const { return systemTicks; }
	};

	// Глобальный экземпляр для удобства (опционально)
	extern Scheduler& scheduler;

} // namespace SimpleRTOS