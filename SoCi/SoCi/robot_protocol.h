#ifndef __ROBOT_PROTOCOL_H__
#define __ROBOT_PROTOCOL_H__

#define START_CODE    0xFF
#define START_CODE1   0x55
// #define Hdata	      0x00
// #define Hdata1        0xFF

typedef enum tag_fsm_state {
	idle,
	start1,
	start2,
	data1,
	data2,
	data3,
	data4,
	buff_check
}fsm_state;

void DelayLoop(int delay_time);
void send_packet(unsigned char Ldata, unsigned char Hdata);
//void Send_Command(unsigned char Ldata, unsigned char Ldata1);
int Motion_ack(unsigned char num, int flg);
int Recieve(void);
int exam_rxbuf(void);
void Clear_Buf(void);

#endif

