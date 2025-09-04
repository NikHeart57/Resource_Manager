#pragma once
#include "Task.hpp"

namespace SimpleRTOS {

	class Scheduler {
	private:
		static const uint8_t MAX_TASKS = 10;    // ������������ ���������� �����
		Task taskPool[MAX_TASKS];				// ������ �������� �����
		Task* taskList[MAX_TASKS];				// ������ ���������� �� ������  
		uint8_t taskCount = 0;                  // ������� ���������� ������������������ �����
		uint32_t systemTicks = 0;               // ������� ��������� �����
		Task* currentTask = nullptr;            // ��������� �� ������, ������� ����������� � ������ ������
		
	public:	
		// ����������� ��������, �� ����� ��������� � .cpp �����
		Scheduler();
		~Scheduler() = default;

		
	public:
		// ������� ������������ ����������� � ������������
		Scheduler(const Scheduler&) = delete;
		void operator=(const Scheduler&) = delete;

		// ����������� ����� ��� ��������� ������������� ���������� ������������
		static Scheduler& getInstance();

		// ������������� ������������
		void init();

		// ������� ���������
		void dispatch();

		// ����������� ����� ������ � ������������
		bool addTask(TaskFunction function, TaskPriority priority);

		// ��������� ��������� ������ (����� ������������, ��� stop/run)
		void suspendTask(TaskFunction function);
		void resumeTask(TaskFunction function);

		// ��������� ���. ���������� �� ���������� �������!
		void tick();

		// ������� ��� �������� ������ � ����� ��� �� ��������� �����
		void sleep(uint32_t ticksToSleep);

		// �������� ������� ��������� �����
		uint32_t getSystemTicks() const { return systemTicks; }
	};

	// ���������� ��������� ��� �������� (�����������)
	extern Scheduler& scheduler;

} // namespace SimpleRTOS