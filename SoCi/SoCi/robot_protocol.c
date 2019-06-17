#include <stdio.h>
#include <string.h>
#include <sys/time.h>
// #include "stdip_ex.h"
#include "robot_protocol.h"
#include "uart_api.h"
//////////////////////////////////////////////////// Protocol Test

static unsigned char pnum = 0;
//static unsigned char counter = 0;

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

void send_packet(unsigned char Ldata, unsigned char Hdata)
{
	unsigned char packet[6] = { 0 };

	packet[0] = START_CODE;	// Start Byte -> 0xff
	packet[1] = START_CODE1; // Start Byte1 -> 0x55
	packet[2] = Ldata;
	packet[3] = 255 - Ldata;
	packet[4] = Hdata;
	packet[5] = 255 - Hdata;

	uart1_buffer_write(packet, 6);
}

/*
void Send_Command(unsigned char Ldata, unsigned char Ldata1)
{
	unsigned char Command_Buffer[6] = {0,};

	Command_Buffer[0] = START_CODE;	// Start Byte -> 0xff
	Command_Buffer[1] = START_CODE1; // Start Byte1 -> 0x55
	Command_Buffer[2] = Ldata;
	Command_Buffer[3] = Ldata1;
	Command_Buffer[4] = Hdata;  // 0x00
	Command_Buffer[5] = Hdata1; // 0xff

	uart1_buffer_write(Command_Buffer, 6);
}*/

// int Motion_ack(unsigned char num)
// {
// 	/*
// 	Clear_Buf();
// 	int index = 0;
// 	unsigned char arr[4] = { 0 };
// 	unsigned char buff;
// 	while (1)
// 	{
// 		send_packet(num, counter);
// 		DelayLoop(5);

// 		while ((uart1_rxbuf_exist() > 0))
// 		{
// 			uart1_buffer_read(&buff, 1);
// 			if (buff == 0x55||buff == 0xff)
// 			{
// 				index = 0;
// 			}
// 			else if(index<4){
// 				arr[index++] = buff;
// 			}
// 		}
// 		if (arr[2] == counter && index!=0)
// 		{
// 			counter++;
// 			break;
// 		}

// 	}
// 	return arr[0];	
// 	*/
// 	/*
// 		while (success_flag == 0) {

// 			switch (state) {
// 			case idle:
// 				if (uart1_rxbuf_exist() > 0)
// 					state = start1;
// 				else
// 					toCnt++;
// 				break;
// 			case buff_check:
// 				if (uart1_rxbuf_exist() > 0)
// 					state = state_mem;
// 				else
// 					toCnt++;
// 				break;
// 			case start1:
// 				uart1_buffer_read(basket, 1);
// 				if (basket[0] == 0xff) {
// 					state_mem = start2;
// 					state = buff_check;
// 				}
// 				else {
// 					state = idle;
// 					toCnt++;
// 				}
// 				break;
// 			case start2:
// 				uart1_buffer_read(basket + 1, 1);
// 				if (basket[1] == 0x55) {
// 					state_mem = data1;
// 					state = buff_check;
// 				}
// 				else {
// 					state = idle;
// 					toCnt++;
// 				}
// 				break;
// 			case data1:
// 				uart1_buffer_read(basket + 2, 1);
// 				state_mem = data2;
// 				state = buff_check;
// 				break;
// 			case data2:
// 				uart1_buffer_read(basket + 3, 1);
// 				state_mem = data3;
// 				state = buff_check;
// 				break;
// 			case data3:
// 				uart1_buffer_read(basket + 4, 1);
// 				state_mem = data4;
// 				state = buff_check;
// 				break;
// 			case data4:
// 				uart1_buffer_read(basket + 5, 1);
// 				success_flag = 1;
// 				state = idle;
// 				break;
// 			default:
// 				state = idle;
// 				toCnt++;
// 				break;
// 			}

// 			gettimeofday(&end, NULL);
// 			msec = ((end.tv_sec - start.tv_sec) * 1000) + ((end.tv_usec - start.tv_usec) / 1000);

// 			if (msec > 5000 && state == idle){
// 				break;
// 			}
// 		}
// 		*/
	
// 	unsigned char basket[12] = { 0 };
// 	unsigned char acknum;
// 	unsigned char acksta;
// 	int success_flag;
// 	int tocnt;
// 	int flow_ctrl;

// 	fsm_state state;
// 	fsm_state state_mem;

// 	flow_ctrl = 1;

// 	while (1) {
// 		state = idle;
// 		state_mem = idle;
// 		success_flag = 0;
// 		tocnt = 0;

// 		if (flow_ctrl) {
// 			send_packet(num, pnum++);
// 			flow_ctrl = 0;
// 		}
// 		else
// 			send_packet(num, (pnum - 1));

// 		uart1_rxbuf_clear();

// 		while ((tocnt < 100000) && (success_flag == 0)) {
// 			switch (state) {
// 			case idle:
// 				if (uart1_rxbuf_exist() > 0)
// 					state = start1;
// 				else
// 					tocnt++;
// 				break;
// 			case buff_check:
// 				if (uart1_rxbuf_exist() > 0)
// 					state = state_mem;
// 				else
// 					tocnt++;
// 				break;
// 			case start1:
// 				uart1_buffer_read(basket, 1);
// 				if (basket[0] == 0xff) {
// 					state_mem = start2;
// 					state = buff_check;
// 				}
// 				else {
// 					state = idle;
// 					tocnt++;
// 				}
// 				break;
// 			case start2:
// 				uart1_buffer_read(basket + 1, 1);
// 				if (basket[1] == 0x55) {
// 					state_mem = data1;
// 					state = buff_check;
// 				}
// 				else {
// 					state = idle;
// 					tocnt++;
// 				}
// 				break;
// 			case data1:
// 				uart1_buffer_read(basket + 2, 1);
// 				state_mem = data2;
// 				state = buff_check;
// 				break;
// 			case data2:
// 				uart1_buffer_read(basket + 3, 1);
// 				state_mem = data3;
// 				state = buff_check;
// 				break;
// 			case data3:
// 				uart1_buffer_read(basket + 4, 1);
// 				state_mem = data4;
// 				state = buff_check;
// 				break;
// 			case data4:
// 				uart1_buffer_read(basket + 5, 1);
// 				success_flag = 1;
// 				state = idle;
// 				break;
// 			default:
// 				state = idle;
// 				tocnt++;
// 				break;
// 			}
// 		}
		
// 		if (success_flag) {
// 			acknum = basket[4];
// 			acksta = basket[2];

// 			if (acknum == pnum)
// 				break;
// 		}
// 	}

// 	return (int)acksta;
	
// 	/*
// 	unsigned char buff[6];
// 	unsigned char data;

// 	send_packet(num, 0);

// 	uart1_buffer_read(buff, 1);
// 	if (buff[0] == 0xff){
// 		uart1_buffer_read(buff, 1);
// 		if (buff[0] == 0x55){
// 			uart1_buffer_read(buff, 1);
// 			data = buff[0];
// 			uart1_buffer_read(buff, 1);
// 			uart1_buffer_read(buff, 1);
// 			uart1_buffer_read(buff, 1);
// 		}
// 		else return -1;
// 	}
// 	else return -1;

// 	return data;
// 	*/
// }
int Motion_ack(unsigned char num, int flg)
{
	if(flg == 1){
		unsigned char tnum = (unsigned char)(255 - (int)num);
		send_packet(num, tnum);
	}
	return Recieve();
}

int Recieve(void)
{
	unsigned char buff[1];
	unsigned char data;
	uart1_buffer_read(buff, 1);

	if (buff[0] == 0xff)
	{
		uart1_buffer_read(buff, 1);
		if (buff[0] == 0x55)
		{
			uart1_buffer_read(buff, 1);
			data = buff[0];
			uart1_buffer_read(buff, 1);
			uart1_buffer_read(buff, 1);
			uart1_buffer_read(buff, 1);
		}
		else return -1;
	}
	else return -1;
		

	return data;
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
