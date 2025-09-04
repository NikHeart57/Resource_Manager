#include "main.hpp"
#include "core/Scheduler.hpp"

// ������ system ����������, ����� ������ ����� � ��� ������.
// � ����� ������� ������� ����� ���� �� �������� ��������� ����� �������� ������,
// �� ��� ������� ����� ���������� ���������� - ���������� ���������.
SystemKernel sysKernel;

// ��������� �������-������
void heartbeatTask(void)
{
	/**
	 * ��� ����������:
	 * ������ �����������
	 * ������� ����: "��� ��� UART"
	 * �������� ���������
	 * �������: "����� UART"
	 * ������� ������������: "������� ���� ����� 1000��"
	 * ��������
	 */
	
	auto& uart = sysKernel.getUART();
	uart.println("[HEARTBEAT] System is alive!");
	sysKernel.releaseUART();

	// �������� ������ �� 1000 ����� (1000 ��)
	SimpleRTOS::scheduler.sleep(1000);
}

void sensorReadTask(void)
{
	auto& uart = sysKernel.getUART();
	uart.println("[SENSOR READ] Read sensor info");
	sysKernel.releaseUART(); // <-- ����� ���� ������ �����, ���������� �� ;

	// �������� ������ �� 500 ��
	SimpleRTOS::scheduler.sleep(500);
}

int main(void)
{
	// SystemKernel system; // ������� ��������� ����������, �.�. ������ ��� ����������
	sysKernel.initialize();

	// �������������� ����������� (����������� ������)
	SimpleRTOS::scheduler.init();

	// ������������ ������ � ������������
	SimpleRTOS::scheduler.addTask(heartbeatTask, SimpleRTOS::TaskPriority::NORMAL);
	SimpleRTOS::scheduler.addTask(sensorReadTask, SimpleRTOS::TaskPriority::HIGH); // ������ ������� ������

	// ����������� ���� ���� - ��������� �������� ���������
	while(true) {
		SimpleRTOS::scheduler.dispatch();
	}

	return 1;
}