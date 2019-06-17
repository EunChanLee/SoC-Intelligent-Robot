#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <math.h>
#include <termios.h>
//image progessing
#include "amazon2_sdk.h"
#include "graphic_api.h"
//robot motion
#include "uart_api.h"
#include "robot_protocol.h"
//user fnc
#include "stdip_ex.h"
#include "mt_driver_ex.h"


#define __SETUP
//----------------------------------------------------------------//
// #define __YELLOWBAR
// #define __REDSTAIR
// #define __MINELAND
// // // #define _N_MINDELAND
// #define __HUDDLE
// #define __BLUEGATE1
// #define __BRIDGE
#define __GOLF
// #define _N_GOLF
// #define __BLUEGATE2
// #define __TRAPHOLE_new
// #define __TRAPHOLE
// #define __CREVASSE
// #define __SPINNINGBAR
// // //----------------------------------------------------------------//
//#define __DEBUG_MODE
//----------------------------------------------------------------//
//#define __COLOR_PICK
//#define __FIN_TEST
//#define __COLOR_PICK
//#define __COLOR_CAL
//#define __YELLOWBAR_DBG
//#define __ARRMID_DBG
//#define __BRIDGE_DBG
//#define __GOLF_DBG
//#define __TRAP_DBG
//#define __FALLDOWN_TEST
//#define __COMM_TEST
//#define __BDG_GREEN_TEST
//#define __BDG_LINE_TEST
//----------------------------------------------------------------//

static struct termios inittio, newtio;

void init_console(void)
{
	tcgetattr(0, &inittio);
	newtio = inittio;
	newtio.c_lflag &= ~ICANON;
	newtio.c_lflag &= ~ECHO;
	newtio.c_lflag &= ~ISIG;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;

	cfsetispeed(&newtio, B115200);

	tcsetattr(0, TCSANOW, &newtio);
}

int main(int argc, char**argv) {
	int state, n, flg, fcnt;
	FILE* log;
	int walker_ver = 0;

	printf("welcome to hobot :D \n");

	if (open_graphic() < 0) {
		printf("Open Graphic Error. Check graphic module.\n");
		return -1;
	}

#ifdef __SETUP

	init_console();

	if (uart_open() < 0) {
		printf("Open Uart Error. Check uart line. \n");
		return -1;
	}

	uart_config(UART1, 57600, 8, UART_PARNONE, 1);

	printf("autonomous system ready to drive. \n");

	if (exam_rxbuf() > 0) {
		Clear_Buf();
		printf("buffer initialized.\n");
	}
	
	if (Motion_SOC(con_check) < 0) {
		printf("%d\n", Motion_SOC(con_check));
		printf("uart error occured.\n");
		return -1;
	}
	printf("autonomous system ready to drive. 3 \n");

	printf("program will be started within 5 sec.\n");
	printf("program is ready. plz disconnect the uart line in 5 sec.\n");
	for (n = 5; n > 0; n--) {
		printf("time : %d\n", n);
		_wait;
	}
	

	
#endif

#ifndef __DEBUG_MODE
	char fname[50] = { 0 };
	fcnt = 0;




#ifdef __YELLOWBAR
	walker_ver = 0;
	watch_waitGreenLight(fcnt++); // log0


	// mot_arrangeMid_Ver2(NULL, 3);
	// mot_gotoReadyLine_red(fcnt++, walker_ver); // log1
	// mot_arrangeMid_Ver2(NULL, 0);

	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);
	_wait_d;
	
	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);
	_wait_d;
	
	mot_arrangeMid(fcnt++);
	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);
	
	Motion_SOC(head_2cm);
	mot_arrangeMid(fcnt++);
	mot_gotoReadyLine_red(fcnt++, walker_ver); // log1

	mot_arrangeMid(fcnt++);
	Motion_SOC(smallwalk_2);
	// Motion_SOC_log(walk_3,1);
	// _wait;
	// _wait;
	// _wait;
	   //revised 0114 

	// Motion_SOC_log(walk_3,2);
	// mot_arrangeMid(fcnt++);
	// _wait;
	// _wait;
	// _wait;
////////////////////////////////////////////////////////////////

	// Motion_SOC_log(head_2cm,3);
	// _wait;
	// _wait;
	// Motion_walker(walk_3, walker_ver);    //2019 0108 WALKVERSION DO NOT USE
	// Motion_SOC_log(walk_3,4);
	// _wait;
	// _wait;
	// _wait;
	// Motion_SOC_log(head_2cm,5);
	// _wait;
	// _wait;
	// mot_arrangeMid(fcnt++);   //revised 0114 

	 // //revised 0117

	//Motion_walker(walk_1, walker_ver);  //2019 0108 WALKVERSION DO NOT USE
	// Motion_SOC_log(walk_1,6);
	// _wait;
	// _wait;

#endif

#ifdef __REDSTAIR
	// redstair
	// do bulldozer
	Motion_SOC(bulldozer);
	// _wait;
	// _wait;
	// _wait;// revised 0114
	// ¸ð¼Ç :ºÒµµÀú¿¡ ÁÂÇâÁÂ ¾ø¾Ù °Í.
	Motion_SOC(head_2cm);
	mot_arrangeMid(fcnt++); // log3
	// _wait;
#endif

#ifdef __MINELAND
							//MINE SCENARIO
							// minelands
	int i;
	int dist1, dist2;
	//int center_x1, center_x2;
	
	int direction = 0;
	Motion_SOC(mine_std);
	

	int left = 0;
	int right = 0;
	int side_dir = 0;
	int count = 0;


	//MINE SCENARIO
#if 1
	watch_dmz(fcnt++);

#endif

	direct_camera_display_on();
#endif
#ifdef _N_MINDELAND
	walker_ver = 0;
	
	for (n = 0; n < 3; n++) {
		Motion_SOC(walk_2);	// revised 190115
		// Motion_walker(walk_2, walker_ver);	// revised 190115
	}
	mot_arrangeMid(fcnt++);
	for (n = 0; n < 3; n++) {
		Motion_SOC(walk_2); // revised 190115
		// Motion_walker(walk_2, walker_ver); 	// revised 190115
	}
	mot_arrangeMid(fcnt++);
	for (n = 0; n < 3; n++) {
		Motion_SOC(walk_2);	// revised 190115
		// Motion_walker(walk_2, walker_ver);	// revised 190115
	}
	mot_arrangeMid(fcnt++);
	Motion_SOC(walk_2);	// revised 190115
	// Motion_walker(walk_2, walker_ver);	// revised 190115
	//walker_ver = 0;
	//for (n = 0; n < 2; n++) {
	//	Motion_walker(walk_4, walker_ver);
	//	Motion_SOC(head_2cm);
	//}
	//mot_arrangeMid(fcnt++);
	//for (n = 0; n < 2; n++) {
	//	Motion_walker(walk_4, walker_ver);
	//	Motion_SOC(head_2cm);
	//}
	//mot_arrangeMid(fcnt++);




#endif

	// mot_arrangeMid(fcnt++);
	// Motion_SOC(head_bar);    //test 0115


#ifdef __HUDDLE
	mot_gotoReadyLine_huddle(fcnt++, walker_ver);
	// Motion_while(huddle_dumbling);
	Motion_SOC(huddle_dumbling);
	// _wait;
	// Motion_while(head_std);
	Motion_SOC(head_2cm);
	//Motion_walker(walk_2, walker_ver); //20190108 WALKVERSION DO NOT USE
	// Motion_while(walk_2);
	Motion_SOC(walk_2);
	_wait; // _wait;
	// mot_arrangeMid(fcnt++); //revised 190115
	// Motion_while(tl_90);
	Motion_SOC(tl_90);		//After huddle
	_wait_d;
	Motion_SOC(head_2cm);
	// Motion_SOC(tl_2);	//revised 0118
	mot_arrangeMid(fcnt++); 
	// _wait;
	//Motion_walker(walk_3, walker_ver);//20190108 WALKVERSION DO NOT USE
	// Motion_while(walk_3);
	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);
	
	// Motion_while(head_2cm);
	Motion_SOC(head_2cm);
	// _wait;
	mot_arrangeMid(fcnt++);
#endif

#ifdef __BLUEGATE1
	for (n = 0; n < 2; n++) {
		//Motion_walker(walk_4, walker_ver);
		Motion_SOC(head_2cm);
		Motion_SOC(walk_3); //revised walk_3 --> walk_1		
		// Motion_SOC(tr_1);
		_wait_d;
	}
	mot_arrangeMid(NULL);   //revised
	Motion_SOC(head_2cm);
	Motion_SOC(walk_3); //revised walk_3 --> walk_1	
	mot_arrangeMid(NULL);	

#endif


#ifdef __BRIDGE
	walker_ver = 0;

	// // green bridge
	// // check line & rearrange
	mot_gotoReadyLine_bdg(fcnt++, walker_ver);

	mot_runBridge(fcnt++, walker_ver);

	mot_arrangeMid(NULL);

	_wait_250ms;
	Motion_SOC(_2cm_down);


#endif
	
	


#ifdef __GOLF
	U16* output = (U16*)malloc(pl * 2);
	int cnt = 0;
	int center_gy;				//°ñÇÁ°ø Áß½ÉÁÂÇ¥ x
	int center_gx;				//°ñÇÁ°ø Áß½ÉÁÂÇ¥ y
	int outline_hl;				//È¦ ¿ÞÂÊ ¿Ü°ûÁÂÇ¥ x
	int outline_hr;				//È¦ ¿À¸¥ÂÊ ¿Ü°ûÁÂÇ¥ x
	int center_hc;				//È¦ Áß½ÉÁÂÇ¥ x
	int center_hcy;				//È¦ Áß½ÉÁÂÇ¥ y
	int mod;

	int length;
	int xchk, ychk;

	float gradient,distL, distR;

	float limit_p = 60.0;
	float limit_n = -60.0;

	strcat(fname, "log_golf");
	concat_num(fname, fcnt++);
	log = fopen(fname, "w");
	


	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);

	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);
	mot_arrangeMid(NULL);


	//골프공의 거릴를 구해 일정거리 안으로 들어오도록 걸어간다.
	fprintf(log, "stage1\n");
	direct_camera_display_off();
	ychk = 0;
	xchk = 0;
	int fin_x = 0, fin_y = 0;

	ychk = 0;
	xchk = 0;
	fin_x = 0;
	fin_y = 0;
	while (cnt < 100)
	{
		_wait;
		_wait;
		Motion_SOC(head_down);
		center_gx = ORANGE_Golf_HS(log, 8, 2, output, &center_gy);
		clear_screen();
		draw_fpga_video_data_full(output);
		flip();
		ychk = MV_F_BigStep_golf(log, center_gy);
		_wait;
		_wait;
		xchk = MV_LR_BigStep_golf(log, center_gx);
		cnt++;
		if (ychk == 1 && xchk == 1)
			break;
	}
	direct_camera_display_on();


	direct_camera_display_off();
	while (1)
	{
		Motion_SOC(head_2cm);
		BLUE_Hole(output, &length, &outline_hl, &center_hcy);

		// hole 의 중심  x 좌표
		center_hc = outline_hl + (length / 2);
		outline_hr = outline_hl + length;

		fprintf(log, "center, y_pos, diameter, x_left, x_right=(%d, %d, %d, %d, %d)\n", center_hc, center_hcy, length, outline_hl, outline_hr);

		distL = outline_hl - 30;
		distR = 150 - outline_hr;


		clear_screen();
		draw_fpga_video_data_full(output);

		flip();
		
		if(distL > (distR - 5) || distL < (distR + 5)){
			// fitted
			Motion_SOC(sl_2);
			Motion_SOC(head_2cm);
			break;
		}
		else if(distL < (distR - 5)){
			Motion_SOC(tr_1);
			Motion_SOC(head_2cm);
		}
		else if(distL < (distR - 25)){
			Motion_SOC(tr_2);
			Motion_SOC(head_2cm);
		}
		else if(distL < (distR - 45)){
			Motion_SOC(tr_3);
			Motion_SOC(head_2cm);
		}
		else if(distL > (distR + 5)){
			Motion_SOC(tl_1);
			Motion_SOC(head_2cm);
		}
		else if(distL > (distR + 25)){
			Motion_SOC(tl_2);
			Motion_SOC(head_2cm);
		}
		else if(distL > (distR + 45)){
			Motion_SOC(tl_3);
			Motion_SOC(head_2cm);
		}
	}
	direct_camera_display_on();



	free(output);
	direct_camera_display_on();
	fclose(log);
	Motion_SOC(soccer);
	Motion_SOC(tl_90);
	//move left or right to face the center of golf ball

	//go close to golf ball with y-coordinate
	Motion_SOC(head_2cm);
	Motion_SOC(walk_4);
	mot_arrangeMid(fcnt++);
#endif
#ifdef _N_GOLF
	walker_ver = 0;

	for (n = 0; n < 3; n++) {
		Motion_walker(walk_4, walker_ver);
		Motion_SOC(head_2cm);
	}
	mot_arrangeMid(fcnt++);
	for (n = 0; n < 2; n++) {
		Motion_walker(walk_4, walker_ver);
		Motion_SOC(head_2cm);
	}
	mot_arrangeMid(fcnt++);
	_wait;
	Motion_SOC(tl_90);
	// Motion_SOC(tl_2);
	Motion_walker(walk_3, walker_ver);

#endif


#ifdef __BLUEGATE2
	walker_ver = 0;

	// mot_arrangeMid(fcnt++);
	for (n = 0; n < 2; n++) {
		Motion_SOC(head_2cm);
		Motion_SOC(walk_3);	// 20190513 walk_4 -> walk_2
	}
	mot_arrangeMid(fcnt++);
	// for (n = 0; n < 3; n++) {
	// 	Motion_walker(walk_4, walker_ver);
	// 	Motion_SOC(head_2cm);
	// }									// 20190513 주석처리

	//mot_arrangeMid(fcnt)++
	//Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);;
	//_wait_d; _wait_d;
	//Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);;
	//_wait_d; _wait_d;-
	//mot_arrangeMid();
	//Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);
	//_wait_d; _wait_d;
	//Motion_SOC(walk_3);
#endif



#ifdef __TRAPHOLE_new //최적화 필요, 실행 잘 되는듯

	mot_gotoReadyLine_bdg(fcnt++, walker_ver);

	Motion_SOC(head_2cm);
	Motion_SOC(smallwalk_2);

	Motion_SOC(_2cm_up);
	_wait; _wait; _wait; _wait; _wait;
	Motion_SOC(head_2cm);
	Motion_SOC(walk_2);
	_wait;
	mot_gotoReadyLine_trap_new(fcnt++, walker_ver);
	// Motion_SOC(smallwalk_2);
	_wait;
	Motion_SOC(trap_dumbling);
	for(n=0; n<8;n++){
		_wait;
	}
#endif


#ifdef __TRAPHOLE //이거 뒤엎장
	walker_ver = 0;

	int onBoard, px, py, loop_ctrl, flow_ctrl;
	int xdist, xcnt;
	int zdist, zcnt;
	
	int cnt_jh=0;


	/*
	int onBoard, px, py, flow_ctrl, loop_ctrl;
	int cntA, cntB, cntC, cntD, cntAbar, cntBbar, cntCbar, cntDbar;
	int ypt[18] = { 0 };
	float data[17] = { 0 };
	float tmp[500] = { 0 };
	int log_cnt = 100;
	*/

	


	onBoard = 0;

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////
	// for test!!!!!
	Motion_SOC(head_2cm);
	Motion_SOC(walk_2);
	mot_arrangeMid(fcnt++);
	// for (n = 0; n < 3; n++) {
	// 	Motion_SOC(walk_3);
	// }
	
	//mot_arrangeMid(fcnt++);
	

	///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////

	Motion_SOC(head_bar);
	
	U8 hue, sat, val;
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);
	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];
	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];
	

direct_camera_display_off();
	printf("11111111\n");
	while (1){	
		printf("while\n");
		Motion_SOC(head_bot);
		
		read_fpga_video_data(fvda);
		get_surf(surf, fvda);
		memset(pala_mem, 0, sizeof(U16)*pl);
		printf("22222222\n");
		for (py = 0; py < 120; py++) {
			for (px = 0; px < 180; px++) {
				hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
				sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
				val = (U8)(0x000f & mysurf[py][px]);

				if((hue < 70 && hue >= 32) && (sat > 2) && (val > 4)){

					pala[py][px] = 0xffff;
					cnt_jh++;
				}
				else{
					pala[py][px] = 0x0000;
				}
			}
		}
		printf("before if: %d \n", cnt_jh);
		if(cnt_jh > 6000){   //7000
			printf("if(cnt_jh > 10000) \n");
				break;
			}
		else if(cnt_jh > 4000){
			printf("else if(cnt_jh > 8000) \n");
			Motion_SOC(smallwalk_2);
			break;
		}
		else{

			Motion_SOC(head_2cm);
			Motion_SOC(smallwalk_2);
			printf("else\n");
		}

		clear_screen();
		draw_fpga_video_data_full(pala_mem);
		flip();

		printf("55555555555 \n");

		_wait_d;


		Motion_SOC(head_2cm);
		Motion_SOC(smallwalk_2);
		//mot_arrangeMid(fcnt++);

	}
		free(fvda);
		free(surf);
		free(pala_mem);
		free(pala);
		free(mysurf);

	direct_camera_display_on();

	Motion_SOC(head_bar);
	//mot_gotoReadyLine_red(fcnt++, walker_ver);
	// mot_gotoReadyLine_trap(onBoard, fcnt++, walker_ver);

	mot_arrangeMid(fcnt++);
	Motion_SOC(smallwalk_1);
	mot_arrangeMid(fcnt++);

	_wait_250ms;
	//Motion_SOC(walk_2);
	//Motion_SOC(head_2cm);
	
	//mot_arrangeMid(fcnt++);
	Motion_SOC(_2cm_up);
	Motion_SOC(head_2cm);
	// _wait_250ms;





////////////////////finish 0207///////////////
	onBoard = 0;
	mot_gotoReadyLine_trap(onBoard, fcnt++, walker_ver);     //190207에 뻄


	memset(fname, 0, 50);
	strcat(fname, "log_playTrap");
	concat_num(fname, fcnt++);
	log = fopen(fname, "w");

	fprintf(log, "********************************************\n");
	fprintf(log, "TRAP play from this on!!!\n");
	fprintf(log, "********************************************\n");

	fprintf(log, "[LOG] first phase.\n");
	fprintf(log, "================================================\n");
	fprintf(log, "================================================\n");
	fprintf(log, "================================================\n");
	fprintf(log, "================================================\n");

	Motion_SOC(smallwalk_2);

	_wait;
	Motion_SOC(head_blue_hole);
	/*    190207 
	loop_ctrl = 1;

	while (loop_ctrl) {
		direct_camera_display_off();
		state = peek_bndy(log);
		direct_camera_display_on();
		switch (state) {
		case skewed_left:
		case 7:
			fprintf(log, "[LOG] log_playTrap \n>> skewed_left.\n");
			Motion_SOC(tr_1);
			Motion_SOC(head_blue_hole);
			fprintf(log, ">> turn right.\n");
			break;
		case skewed_right:
		case 8:
			fprintf(log, "[LOG] log_playTrap \n>> skewed_right.\n");
			Motion_SOC(tl_1);
			Motion_SOC(head_blue_hole);
			fprintf(log, ">> turn left.\n");
			break;
		default:
			fprintf(log, "[LOG] log_playTrap \n>> N/A.\n");
			loop_ctrl = 0;
			break;
		}
		fprintf(log, "================================================\n");
		fprintf(log, "================================================\n");
	}
*/
	fprintf(log, "[LOG] second phase.\n");
	fprintf(log, "================================================\n");
	fprintf(log, "================================================\n");
	fprintf(log, "================================================\n");
	fprintf(log, "================================================\n");

	Motion_SOC(head_2cm);
	loop_ctrl = 10;
	flow_ctrl = 1;



	


	direct_camera_display_off();
	while (loop_ctrl > 0) {
		read_fpga_video_data(fvda);
		get_surf(surf, fvda);
		memset(pala_mem, 0, sizeof(U16)*pl);

		xcnt = 0; zcnt = 0;
		xdist = 0; zdist = 0;

		for (py = 40; py < 120; py++) {
			for (px = 40; px < 140; px++) {
				hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
				sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
				val = (U8)(0x000f & mysurf[py][px]);

				if (val > 3) {  //if ((hue < 80 && hue >= 25) && (sat > 2) && (val > 3)) {
					pala[py][px] = 0x0000;
				}
				else {
					//zcnt++;
					//zdist += px;
					xcnt++;
					xdist += px;
					pala[py][px] = 0x001f;
				}
				
				//if (val < 6) {
				//	xcnt++;
				//	xdist += px;
				//	pala[py][px] = 0x001f;
				//}
			}
		}

		Motion_SOC(head_bar);
		xdist = (int)(xdist / xcnt);
		//zdist = (int)(zdist / zcnt);

		fprintf(log, "[LOG] log_playTrap \n>> xdist = %5d.\n", xdist);
		//fprintf(log, "[LOG] log_playTrap \n>> zdist = %5d.\n", zdist);

		for (py = 0; py < 120; py++)
			pala[py][xdist] = 0xf800;

		//for (py = 0; py < 120; py++)
		//	pala[py][zdist] = 0x07e0;


		clear_screen();
		draw_fpga_video_data_full(pala_mem);
		flip();
		///////////////////////0207revised/////////////////////////////////

		if (xdist < 88) {
			Motion_SOC(sl_1);
			Motion_SOC(head_2cm);
			fprintf(log, ">> move left fast.\n");
		}
		else if (xdist >= 88 && xdist <= 92) {
			fprintf(log, "[LOG] mot_runBridge \n>> line is fitted.\n");
			Motion_SOC(head_2cm);
			loop_ctrl = 0;
		}
		else if (xdist > 92) {
			Motion_SOC(sr_1);
			Motion_SOC(head_2cm);
			fprintf(log, ">> move right fast.\n");
		}
		//loop_ctrl--;
		Motion_SOC(head_bar);
		fprintf(log, "================================================\n");
	}
	direct_camera_display_on();


	fprintf(log, "[LOG] ready to do dumbling\n");
	fprintf(log, "================================================\n");


	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);
	fclose(log);

	//Motion_SOC(walk_2);
	Motion_SOC(walk_2);
	Motion_SOC(walk_2);
//	Motion_SOC(smallwalk_1);
	_wait;
	_wait;
	Motion_SOC(trap_dumbling);
#endif

#ifdef __CREVASSE

	Motion_SOC(head_2cm);
	Motion_SOC(walk_2_ver2);
	mot_arrangeMid(fcnt++);
	mot_gotoReadyLine_crevasses(fcnt++, walker_ver);
	// mot_gotoReadyLine_bdg(fcnt++, walker_ver);	
	for (n = 0; n < 3; n++){
	if(n==0)mot_arrangeMid(fcnt++);
	Motion_SOC(head_2cm);
	if(n<1) Motion_SOC(walk_2);
	if(n>=1) Motion_SOC(walk_1);
	// if(n==2) Motion_SOC(smallwalk_2);
	}
	Motion_SOC(crevasse);
	for(n=0; n<8;n++){
		_wait;
	}
	Motion_SOC(head_2cm);
	Motion_SOC(walk_4);
	mot_arrangeMid(fcnt++);
#endif


#ifdef __SPINNINGBAR
	walker_ver = 0;

	Motion_SOC(head_2cm);
	Motion_SOC(walk_3);
	mot_arrangeMid(NULL);
	Motion_SOC(head_bar);
	watch_waitGreenLight(fcnt++);

	for(n = 0; n < 2; n++){
		Motion_SOC(head_2cm);
		Motion_SOC(walk_4);
		//Motion_SOC(walk_2);
		//_wait_d;
	}
#endif
#else
#ifdef __COLOR_CAL
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala = (U16*)malloc(sizeof(U16)*pl);
	U8 hue;
	U8 sat;
	U8 val;
	direct_camera_display_off();
	while (1) {
		printf("[any key] exam // [3] exit \n");
		scanf("%d", &flg);
		while (getc(stdin) != '\n');
		if (flg == 3)
			break;

		read_fpga_video_data(fvda);
		get_surf(surf, fvda);
		n = pure_red(pala, surf);
		colorCalib(surf, 60, &hue, &sat, &val);

		printf("calibration h / s / v : %5d / %5d / %5d \n", hue, sat, val);
		printf(">> num of pure_Red pixels : %d \n", n);

		clear_screen();
		draw_fpga_video_data_full(pala);
		flip();
	}
	direct_camera_display_on();
	free(fvda);
	free(surf);
	free(pala);
#endif
#ifdef __COLOR_PICK
	int flg, n, m, x, y;
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala = (U16*)malloc(sizeof(U16)*pl);

	int x_sum[ih] = { 0 };
	int x_cnt[ih] = { 0 };
	int x_max = 0;
	int xm_idx = 0;
	int x_avg = 0;
	int y_sum[iw] = { 0 };
	int y_cnt[iw] = { 0 };
	int y_max = 0;
	int ym_idx = 0;
	int y_avg = 0;

	while (1) {
		fprintf(stdout, "[any key] exam // [3] exit \n");
		fscanf(stdin, "%d", &flg);
		while (getc(stdin) != '\n');
		if (flg == 3)
			break;

		direct_camera_display_off();

		for (n = 0; n < 50; n++) {
			read_fpga_video_data(fvda);
			get_surf(surf, fvda);
			colorRegion(pala, surf, YELLOW);

			x_max = 0;
			x_avg = 0;
			y_max = 0;
			y_avg = 0;
			memset(x_cnt, 0, sizeof(int)*ih);
			memset(y_cnt, 0, sizeof(int)*iw);

			for (y = 0; y < ih; y++) {
				for (x = 0; x < iw; x++) {
					if (!pala[XY(x, y)]) {
						x_sum[y] += x;
						x_cnt[y]++;
						y_sum[x] += y;
						y_cnt[x]++;
					}
				}
			}

			for (m = 0; m < ih; m++)
				if (x_max < x_cnt[m]) {
					x_max = x_cnt[m];
					xm_idx = m;
				}

			for (m = 0; m < iw; m++)
				if (y_max < y_cnt[m]) {
					y_max = y_cnt[m];
					ym_idx = m;
				}

			x_avg = (int)(x_sum[xm_idx] / x_cnt[xm_idx]);
			y_avg = (int)(y_sum[ym_idx] / y_cnt[ym_idx]);
			printf("x_avg, y_avg : %5d / %5d \n", x_avg, y_avg);
			clear_screen();
			draw_fpga_video_data_full(pala);
			flip();
		}

		direct_camera_display_on();
		//fprintf(stdout, "[Message] state : %d\n", state);
	}

	free(fvda);
	free(surf);
	free(pala);
#endif
#ifdef __FIN_TEST
	mot_arrangeMid();
	Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);;
	// Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);				// 20190513 주석처리
	mot_arrangeMid();
	Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);;
	// Motion_SOC(walk_3);											// 20190513 주석처리

	U16* output = (U16*)malloc(sizeof(U16)*pl);


	// ³ë¶õ»ö ¹Ù°¡ º¸ÀÏ ¶§±îÁö ´ë±â!
	Motion_SOC(head_bar);
	while (1) {
		flg = YLW_final(output);
		//printf("max : %d\n", flg);

		clear_screen();
		draw_fpga_video_data_full(output);
		flip();

		if (flg > 60)
			break;
	}

	// ³ë¶õ»ö ¹Ù°¡ ¾Èº¸ÀÏ ¶§±îÁö ´ë±â!
	while (1) {
		flg = YLW_final(output);
		//printf("max : %d\n", flg);

		clear_screen();
		draw_fpga_video_data_full(output);
		flip();

		if (flg < 30)
			break;
	}

	free(output);

	for (n = 0; n < 3; n++) {
		Motion_SOC(walk_2); _wait_d; Motion_SOC(walk_2);;
	}
#endif
#ifdef __YELLOWBAR_DBG
	Motion_SOC(head_bar); //_wait;
	while (1) {
		printf("[any number] exam / [2] exit \n");
		scanf("%d", &flg);

		if (flg == 2)
			break;

		ylwbarpeek();
	}
#endif
#ifdef __ARRMID_DBG
	Motion_SOC(head_std);
	Motion_SOC(head_right);

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);

	while (1) {
		fprintf(stdout, "[any key] exam / [2] fpga image / [3] exit \n");
		fscanf(stdin, "%d", &flg);
		while (getc(stdin) != '\n');

		direct_camera_display_off();

		if (flg == 2) {
			clear_screen();
			read_fpga_video_data(fvda);
			memcpy(surf, fvda, sizeof(U16)*pl);
			draw_fpga_video_data_full(surf);
			flip();
			printf("press any key to continue\n");
			fscanf(stdin, "%d", &flg);
			while (getc(stdin) != '\n');
		}
		else if (flg == 3)
			break;

		state = bndypeek();
		direct_camera_display_on();
		fprintf(stdout, "[Message] state : %d\n", state);
	}

	free(surf);
	free(fvda);

#endif
#ifdef __BRIDGE_DBG
	int flg, state;
	//Motion_SOC(_2cm_up);
	//Motion_SOC(head_std);
	Motion_SOC(head_2cm);

	while (1) {
		fprintf(stdout, "[any key] exam // [3] exit \n");
		fscanf(stdin, "%d", &flg);
		while (getc(stdin) != '\n');

		if (flg == 3)
			break;
		direct_camera_display_off();
		//state = line_target(90, fitting, bot_porking);
		state = bdgpeek(0);
		direct_camera_display_on();


		fprintf(stdout, "[Message] state : %d\n", state);
	}

#endif
#ifdef __GOLF_DBG
	int flg;
	int center_y;
	int center_x;
	int center_hole;
	int length;
	U16* output = (U16*)malloc(pl * 2);
	//int lookfornext;
	//int cnt = 0;
	flg = 0;

	while (1)
	{
		printf("[1] golf // [2] hole // [3] hole_new // [4] kick // [5] exit //  \n");
		scanf("%d", &flg);
		while (getchar() != '\n');

		if (flg == 1) {
			Motion_SOC(head_std);
			center_x = ORANGE_Golf_HS(7, 4, output, &center_y);
		}
		else if (flg == 2) {
			Motion_SOC(head_blue_hole);
			BLUE_Hole(output, &length, &center_x);
			center_hole = center_x + (length / 2);
			printf("center, length, x_init=(%d, %d, %d)\n", center_hole, length, center_x);
		}
		else if (flg == 3) {
			Motion_SOC(head_blue_hole_new);
			BLUE_Hole(output, &length, &center_x);
			center_hole = center_x + (length / 2);
			printf("center, length, x_init=(%d, %d, %d)\n", center_hole, length, center_x);
		}
		else if (flg == 4)
			Motion_SOC(soccer);
		else if (flg == 5)
			break;

		clear_screen();
		draw_fpga_video_data_full(output);
		flip();

		//lookfornext = GOLF_MOVE(center_x);
		//if (lookfornext == 1)
		//{
		//	pass1 = CHECK_CENTER(center_x);
		//	break;
		//}
	}

	free(output);
#endif
#ifdef __TRAP_DBG
	int state_hrizon, state_vert;
	//int n, state;
	int flg;

#ifdef MODE2
	Motion_SOC(head_std);
	Motion_SOC(head_right);
#else
#ifdef MODE1
	Motion_SOC(head_2cm);
#else
	//Motion_SOC(head_bot);
	//Motion_SOC(head_std);
#endif
#endif
	while (1) {
		printf("[any key] exam // [3] exit \n");
		scanf("%d", &flg);
		while (getchar() != '\n');

		if (flg == 3)
			break;
		direct_camera_display_off();
		state_hrizon = line_target(80, fitting, bot_porking, trap);
		state_vert = bdgpeek(1, trap_hole);
		direct_camera_display_on();
		printf("[Message] state_hrizon : %d\n", state_hrizon);
		printf("[Message] state_hrizon : %d\n", state_vert);

	}
#ifdef MODE2
	Motion_SOC(head_pre);
	Motion_SOC(head_std);
#else
	Motion_SOC(head_std);
#endif

#endif
#ifdef __FALLDOWN_TEST
	fcnt = 0;
	while (1) {
		printf("---------------------------\n");
		state = Motion_SOC(0);
		printf("gyro cnt : %5d\n", fcnt);
		switch (state) {
		case 1:
			break;
		case 0:
			printf("robot felt down\n");
			break;
		default:
			printf("uart error occured.\n");
			break;
		}
		fcnt++;
		printf("---------------------------\n");
		_wait_250ms;
	}
#endif
#ifdef __COMM_TEST
	direct_camera_display_off();
	while (1) {
		Motion_SOC(head_bot);
		Motion_SOC(head_std);
	}
#endif
#ifdef __BDG_GREEN_TEST
	int px, py, cntA, cntB, dstX;
	U8 hue, sat, val;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);

	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];

	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];

	Motion_SOC(head_2cm); //_wait;

	while (1) {
		printf("[1] exam / [2] display off / [3] display on / [4] exit  \n");
		scanf("%d", &flg);

		while (getchar() != '\n');

		if (flg == 4)
			break;
		else if (flg == 3)
			direct_camera_display_on();
		else if (flg == 2)
			direct_camera_display_off();
		else if (flg == 1) {
			read_fpga_video_data(fvda);
			get_surf(surf, fvda);

			memset(pala_mem, 0, sizeof(U16)*pl);

			cntA = 0; // cntA : sum
			cntB = 0; // cntB : number of pixels

					  // green pixels
			for (py = 60; py < 120; py++) {
				for (px = 0; px < 180; px++) {
					hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
					sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
					//val = (U8)(0x000f & mysurf[py][px]);

					if ((hue > 70 && hue <= 100) && (sat > 1)) {
						cntA += px;
						cntB++;
						pala[py][px] = 0x001f;
					}
				}
			}

			if (cntB < 900) {
				printf(">> exit condition satisfied.\n");
			}
			else {
				dstX = (int)(cntA / cntB);
				for (py = 0; py < 120; py++) {
					pala[py][dstX] = 0xf800;
				}
				printf(">> dstX = %5d.\n", dstX);
			}

			clear_screen();
			draw_fpga_video_data_full(pala_mem);
			flip();

			if (dstX < 70) {
				//Motion_SOC(sl_2);
				//Motion_SOC(head_2cm);
				printf(">> move left fast.\n");
			}
			else if (dstX >= 70 && dstX < 80) {
				//Motion_SOC(sl_1);
				//Motion_SOC(head_2cm);
				printf(">> move left.\n");
			}
			else if (dstX >= 80 && dstX <= 100) {
				printf("[LOG] mot_runBridge \n>> line is fitted.\n");
				//Motion_SOC(walk_4);
				//Motion_SOC(head_2cm);
				printf(">> move forward.\n");
			}
			else if (dstX > 100 && dstX <= 110) {
				//Motion_SOC(sr_1);
				//Motion_SOC(head_2cm);
				printf(">> move right.\n");
			}
			else if (dstX > 110) {
				//Motion_SOC(sr_2);
				//Motion_SOC(head_2cm);
				printf(">> move right fast.\n");
			}
		}		
	}
	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);
#endif
#endif

#ifdef __SETUP
	uart_close();
#endif
	close_graphic();
    printf("program terminated.\n");
	return 0;
}
