#include "main.hpp"
#include "core/Scheduler.hpp"

// Делаем system глобальной, чтобы задачи имели к ней доступ.
// В более сложном проекте можно было бы передать указатель через параметр задачи,
// но для учебных целей глобальная переменная - приемлемое упрощение.
SystemKernel sysKernel;

// Объявляем функции-задачи
void heartbeatTask(void)
{
	/**
	 * Что происходит:
	 * Задача просыпается
	 * Говорит ядру: "Дай мне UART"
	 * Печатает сообщение
	 * Говорит: "Верни UART"
	 * Говорит планировщику: "Разбуди меня через 1000мс"
	 * Засыпает
	 */
	
	auto& uart = sysKernel.getUART();
	uart.println("[HEARTBEAT] System is alive!");
	sysKernel.releaseUART();

	// Усыпляем задачу на 1000 тиков (1000 мс)
	SimpleRTOS::scheduler.sleep(1000);
}

void sensorReadTask(void)
{
	auto& uart = sysKernel.getUART();
	uart.println("[SENSOR READ] Read sensor info");
	sysKernel.releaseUART(); // <-- Здесь была лишняя точка, исправляем на ;

	// Усыпляем задачу на 500 мс
	SimpleRTOS::scheduler.sleep(500);
}

int main(void)
{
	// SystemKernel system; // Убираем локальную переменную, т.к. теперь она глобальная
	sysKernel.initialize();

	// Инициализируем планировщик (настраивает таймер)
	SimpleRTOS::scheduler.init();

	// Регистрируем задачи в планировщике
	SimpleRTOS::scheduler.addTask(heartbeatTask, SimpleRTOS::TaskPriority::NORMAL);
	SimpleRTOS::scheduler.addTask(sensorReadTask, SimpleRTOS::TaskPriority::HIGH); // Чтение датчика важнее

	// Бесконечный цикл ядра - постоянно вызывает диспетчер
	while(true) {
		SimpleRTOS::scheduler.dispatch();
	}

	return 1;
}