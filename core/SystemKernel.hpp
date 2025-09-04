/**
 * SystemKernel �� ������ �������� ���������� �������� (UART, SPI � �.�.), �� � �������� � ����������� �����.
 *
 */
#pragma once
#include "../resources/UART_Resource.hpp"
#include <stdlib.h>
#include <stdint.h>

enum class TaskPriority {
	LOW,
	NORMAL,
	HIGH,
	CRITICAL // ��� ����������
};


class SystemKernel 
{
private:
	// ��������: SystemKernel - ����������� �������� ���� ��������.
	UART_Resource UART_;			// ������ UART
	bool isUARTBusy_ = false;		// ���� ��������� �������
	
public:
	SystemKernel();
	void initialize();
	
	// ����� ��� ������������� ������� ������� ������������
	UART_Resource& getUART();
	void releaseUART();
};