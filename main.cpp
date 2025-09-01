#include "main.hpp"

int main(void)
{
	SystemKernel system;		// ������� ����
	system.initialize();		// �������������� �������

	Application app(system);	// ������� ����������, �������� ��� ����
	app.run();					// ��������� ���������� ������

	return 1; 
}