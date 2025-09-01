#include "Application.hpp"

Application::Application(SystemKernel& system) : system_(&system) // ����� ����� ������
{
}

void Application::run()
{
	// ���������� ����������
	SharedBuffer buffer;

	auto& uart = system_->getUART();	// ����������� ������
	uart.println("Hello");				// ���������� ���
	system_->releaseUART();				// ���������� ������ ����
	
	while(true)
	{
	}
}