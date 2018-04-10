#include "state_handler.h"
#include "serial.h"

unsigned char serialBuffer[5];
unsigned char bufferIndex;
static unsigned char newData;

/*
Commands implemented:
-> 1 = send single frame
-> 2 = send repetitive frame each 1 ms
-> 3 = send repetitive frame each 5 ms
-> 4 = send repetitive frame each 10 ms
-> 5 = stop repetitive frames
-> 6 = baudrate 125kbps
-> 7 = baudrate 250 kbps
-> 8 = baudrate 500kbps
-> 9 = baudrate 1Mbps
-> A = read bus for 5 seconds
*/

void processISR(void)
{
	serialBuffer[bufferIndex++] = huart4.Instance->DR;
	if(bufferIndex == 5)
	{
		bufferIndex = 0;
	}
	newData = 1;
}

void sendByte(unsigned char byte)
{
	huart4.Instance->DR = byte;
	do
	{
	} while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE) == 0);
}

void sendString(unsigned char *pnt, unsigned char numberBytes)
{
	unsigned char i;
	i = 0;
	for(i=0; i<numberBytes; i++)
	{
		sendByte(*(pnt+i));
	}
}

void sendInt(unsigned short data)
{
	unsigned char index, separateChars[10];
	index = 0;
	do
	{
		separateChars[index++] = data%10;
		data /= 10;
	}
	while(data > 0);
	for(;index>0;index--)
	{
		sendByte((unsigned char)(separateChars[index-1]+48));
	}
}

void serialInit()
{
	bufferIndex = 0;
	newData = 0;
}

void serialMain()
{
	if(newData == 1)    /*if new command is available, process it*/
	{
		switch (serialBuffer[bufferIndex-1])
		{
			case 49:
				if(STATE == Idle)
				{
					STATE = TransmitSingle;
				}
				break;
			case 50:
				if(STATE == Idle)
				{
					STATE = InitPeriodicFrame;
					PERIODICITY = 1;
				}
				break;
			case 51:
				if(STATE == Idle)
				{
					STATE = InitPeriodicFrame;
					PERIODICITY = 5;
				}
				break;
			case 52:
				if(STATE == Idle)
				{
					STATE = InitPeriodicFrame;
					PERIODICITY = 10;
				}
				break;
			case 53:
				STATE = Idle;
				break;
			case 54:
				BAUDRATE = 12;
				STATE = SetBaud;
				break;
			case 55:
				BAUDRATE = 25;
				STATE = SetBaud;
				break;
			case 56:
				BAUDRATE = 50;
				STATE = SetBaud;
				break;
			case 57:
				BAUDRATE = 100;
				STATE = SetBaud;
				break;
			case 65:
				STATE = ReceiveLimitedTime;
				break;
			default:
				break;
		}
		newData = 0;
	}
}
