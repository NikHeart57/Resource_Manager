#pragma once

#include "../core/SystemKernel.hpp"
class SystemKernel; // Forward declaration

class Application 
{
	private:
	SystemKernel* system_; // ��������� ������ ������
	long int counter_ = 0;
	union SharedBuffer 
	{
		char as_char[32];      // ��� ����� (itoa)
		uint8_t as_uint8[32];  // ��� ����� ����
		uint32_t as_uint32[8]; // ��� 32-������ ��������
		float as_float[8];     // ��� ����� � ��������� ������
	};

	public:
	explicit Application(SystemKernel& system);
	void run(); // ���� ��������� �������� ���� ����������
};