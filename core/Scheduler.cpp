#include "Scheduler.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

namespace SimpleRTOS {

	// ����������� ������������
	Scheduler::Scheduler()
	{
		taskCount = 0;
		systemTicks = 0;
		currentTask = nullptr;
		
		// ������������� ������� ����������
		for(uint8_t i = 0; i < MAX_TASKS; i++)
		{
			taskList[i] = nullptr;
		}
		// taskPool ������������� ���������������� �������������� �� ���������
	}

	static Scheduler schedulerInstance;

	Scheduler& Scheduler::getInstance() {
		return schedulerInstance;
	}

	void Scheduler::init() {
		// ����� ����� ��������� ��������� ������ (��������, Timer1),
		// ������� ����� ������������ ���������� ��� � 1 �� (1 ���).
		// ��� � ����� ���� "��������� ����".

		// ��������� Timer1 ��� ��������� ���������� �� ���������� ������ 1 ��
		// (������ ��� ATmega328P, F_CPU=16MHz)
		TCCR1A = 0;
		TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler 64
		OCR1A = 249; // Compare value ��� 1ms: (16000000 / 64 / 1000) - 1 = 249
		TIMSK = (1 << OCIE1A); // Enable compare match interrupt

		sei(); // ���������� ���������� ����������
	}

	// ���������� ���������� �������
	ISR(TIMER1_COMPA_vect) {
		SimpleRTOS::scheduler.tick();
	}

	void Scheduler::tick() {
		systemTicks++;

		// ��������� ��������� ���� �����
		for(uint8_t i = 0; i < taskCount; i++) {
			Task* task = taskList[i];
			if(task->state == TaskState::SLEEPING) {
				if(task->tickCounter > 0) {
					task->tickCounter--;
				}
				if(task->tickCounter == 0) {
					task->state = TaskState::READY; // ������ ����������
				}
			}
		}
	}

	void Scheduler::dispatch() {
		// ����� ������ � ��������� �����������, ������� � ���������� (READY)
		int8_t highestPriorityTaskIndex = -1;
		TaskPriority highestPriority = TaskPriority::IDLE;

		for(uint8_t i = 0; i < taskCount; i++) {
			Task* task = taskList[i];
			if((task->state == TaskState::READY) && (task->priority > highestPriority)) {
				highestPriority = task->priority;
				highestPriorityTaskIndex = i;
			}
		}

		// ���� ����� ������� ������ - ��������� ��
		if(highestPriorityTaskIndex >= 0) {
			currentTask = taskList[highestPriorityTaskIndex]; // <-- ���������� ������� ������
			currentTask->state = TaskState::RUNNING;
			currentTask->lastRunTime = systemTicks;

			// ��������� ���� ������� ������
			currentTask->function();

			// ����� ���������� ��������� ������ ������� � ��������� READY,
			// ���� ��� �� ���� ���������� � SLEEPING ��� SUSPENDED ������ ����� ����.
			if(currentTask->state == TaskState::RUNNING) {
				currentTask->state = TaskState::READY;
			}
			currentTask = nullptr; // <-- ���������� ��������� ����� ����������
		}

		// ���� �� ���� ������ �� ������, ����� ��������� ������ ��������� ���� (idle)
		// ��� ��������� �� � ����� ����������� �����������������.
	}

	bool Scheduler::addTask(TaskFunction function, TaskPriority priority)
	{
		if(taskCount >= MAX_TASKS) return false;
		
		// �������������� ������ � ������� ���������� ������
		taskList[taskCount] = &taskPool[taskCount];
		*taskList[taskCount] = Task(function, priority); // ���������� �����������
		taskCount++;
		return true;
	}

	void Scheduler::suspendTask(TaskFunction function) {
		for(uint8_t i = 0; i < taskCount; i++) {
			if(taskList[i]->function == function) {
				taskList[i]->state = TaskState::SUSPENDED;
				return;
			}
		}
	}

	void Scheduler::resumeTask(TaskFunction function) {
		for(uint8_t i = 0; i < taskCount; i++) {
			if(taskList[i]->function == function) {
				taskList[i]->state = TaskState::READY;
				return;
			}
		}
	}

	void Scheduler::sleep(uint32_t ticksToSleep) {
		// ��� ������� ���������� ������� ����� ������.
		// �� �����, ����� ������ ����������� � ������ ������ (currentTask != nullptr)
		if (currentTask) {
			currentTask->state = TaskState::SLEEPING;
			currentTask->tickCounter = ticksToSleep;
		}
		// ���� sleep ������ �� �� ������ (��� ����������), ������ ����������.
	}

	// ���������� ���������� ���������
	Scheduler& scheduler = Scheduler::getInstance();

} // namespace SimpleRTOS