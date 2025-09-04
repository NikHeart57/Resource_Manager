#pragma once
#include <stdint.h>

// ������������ ���� ��� ����� RTOS ��� �������� ����
namespace SimpleRTOS {

	// ��������� ������
	enum class TaskState {
		READY,      // ������ � ����������
		RUNNING,    // ����������� � ������ ������
		SLEEPING,   // ������� �� ������������ �����
		SUSPENDED   // �������������� ���� (������� stop())
	};

	// ��������� ������. ��� ����� ������, ��� ��������� ����.
	// ������ � ������ ����������� ����� ����������� ������.
	enum class TaskPriority {
		IDLE = 0,   // ������ ��������� ���� (����� ������)
		LOW = 1,
		NORMAL = 2,
		HIGH = 3,
		SYSTEM = 4  // ��� ��������� ��������� ����� (����� �������)
	};

	// ��� ��������� �� ������� ������.
	// ������ - ��� ������ ������� ���� void(*)(void).
	typedef void (*TaskFunction)(void);

	/**
	 * @brief ���������, ����������� ���� ������ � �������.
	 * @detail ����������� ����� �������� � �������� ����� ��������.
	 */
	struct Task {
		TaskFunction function;    // ��������� �� ������� ������
		TaskState state;          // ������� ���������
		TaskPriority priority;    // ���������
		uint32_t tickCounter;     // ������� ����� �� ����������� (��� SLEEPING)
		uint32_t tickInterval;    // �������� ��� ������������� �����
		uint32_t lastRunTime;     // ����� ���������� ������� (�� ������ ��������� �����)

		// ����������� �� ���������
		Task() : function(nullptr), state(TaskState::READY), priority(TaskPriority::IDLE), 
				 tickCounter(0), tickInterval(0), lastRunTime(0) {}
    
		// ����������� ��� �������� ��������
		Task(TaskFunction func, TaskPriority prio) : function(func), state(TaskState::READY), 
			 priority(prio), tickCounter(0), tickInterval(0), lastRunTime(0) {}
	};

} // namespace SimpleRTOS