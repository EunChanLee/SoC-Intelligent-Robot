/************************************************************************
  Title     : Robot Body Protocol Source File
  File name : robot_protocol.c    

  Author    : adc inc. (oxyang@adc.co.kr)
  History
		+ v0.0  2007/2/14
		+ v1.0  2008/8/6
************************************************************************/
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "robot_protocol.h"
#include "uart_api.h"
//////////////////////////////////////////////////// Protocol Test

void DelayLoop(int delay_time)
{
	struct timeval start, end;
	long msec;
	gettimeofday(&start, NULL);

	for (;;) {
		gettimeofday(&end, NULL);
		msec = ((end.tv_sec - start.tv_sec) * 1000) + ((end.tv_usec - start.tv_usec) / 1000);
		if (msec > delay_time)
		{
			break;
		}
	}
}

void Send_Command(unsigned char Ldata, unsigned char Ldata1)
{
	unsigned char packet[6] = { 0 };

	packet[0] = START_CODE;	// Start Byte -> 0xff
	packet[1] = START_CODE1; // Start Byte1 -> 0x55
	packet[2] = Ldata;
	packet[3] = Ldata1;
	packet[4] = Hdata;  // 0x00
	packet[5] = Hdata1; // 0xff

	uart1_buffer_write(packet, 6);
}

#define ERROR	0
#define OK	1
void Motion(unsigned char num)
{
	unsigned char tnum = (unsigned char)(255 - (int)num);
	Send_Command(num, tnum); 
}

int Motion_ack(unsigned char num)
{
	
	/*
	// old_version

	unsigned char tnum = (unsigned char)(255 - (int)num);
	Send_Command(num, tnum);
	return Recieve();
	*/

	unsigned int ackNum;
	unsigned char tnum = (unsigned char)(255 - (int)num);
	Send_Command(num, tnum);
	Clear_Buf();
	ackNum = Recieve();
	return ackNum;
}

int Recieve()
{
	/*
	// old_version
	unsigned char buff[6];
	unsigned char data;
	uart1_buffer_read(buff, 6);
	if (buff[0] == 0xff)
	{
		if (buff[1] == 0x55)
		{
			data = buff[2];
		}
		else return -1;
	}
	else return -1;

	//return data;
	*/
	
	unsigned char buff[5] = {0};
	unsigned char buffSize = 5;
	unsigned char cnt = 0; 
	unsigned char temp = 0;

	while (1) {
		while (uart1_rxbuf_exist() > 0)
		{
			temp = 0;
			uart1_buffer_read(&temp, 1);

			if (temp == 0x55)
				cnt = 0;

			if (cnt < buffSize)
				buff[cnt++] = temp;
		}

		if (cnt == buffSize)
			return buff[1];
	}
}

int exam_rxbuf(void) {
	int temp = uart1_rxbuf_exist();
	printf("uart_rx_level :%d\n", temp);
	return temp;
}

void Clear_Buf(void)
{
	uart1_rxbuf_clear();
}
