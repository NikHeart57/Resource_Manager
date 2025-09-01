#include "SystemKernel.hpp"


SystemKernel::SystemKernel() : UART_(UART_Resource::Config{}) // �������������� ������ �����
{
	
}

void SystemKernel::initialize() 
{
	// �������� �������������� ���� �������
	UART_.start(57600);
	UART_.println("System Kernel initialized");
}



UART_Resource& SystemKernel::getUART() 
{
	if (isUARTBusy_) 
	{
		// ����� ������� ������ �� "��������", ������� ����������,
		// ��� ��������� ������������ � �����
		// ���� ������ ������ �������� ������, �� �������� ������
		UART_.println("ERROR: UART is already busy!");
	}
	isUARTBusy_ = true;
	return UART_;
}





void SystemKernel::releaseUART() 
{
	isUARTBusy_ = false;
}