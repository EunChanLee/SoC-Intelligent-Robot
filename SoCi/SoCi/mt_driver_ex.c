#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "amazon2_sdk.h"
#include "graphic_api.h"
#include "uart_api.h"
#include "robot_protocol.h"
#include "stdip_ex.h"
#include "mt_driver_ex.h"

int mot_gotoReadyLine_red(int fcnt, int walk_ver) {
	int state, flg;

	FILE* logf = NULL;
	//U16* fvda;
	//U16* surf;
	//U16* o_surf;
	char fname[50] = { 0 };
	strcat(fname, "log_readyRed");

	printf("section111111111111111\n");
	
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	flg = 1;
	Motion_SOC(head_std);

	// first we search the line. keep move until we get a line
	while (flg) {
		direct_camera_display_off();
		state = line_target(logf, 0, peeking, bot_porking, default_obs);
		direct_camera_display_on();
		if (state == fitted)
			break;
		else
			Motion_SOC(walk_4);
		_wait_250ms;
	}

	// next we get closer to the line, if line is disapeared, we finish the movement.
	while (flg) {
		direct_camera_display_off();
		state = line_target(logf, 100, fitting, bot_porking, default_obs);
		direct_camera_display_on();
		printf("section3333333333333333333333\n");
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> skewed_left.\n");
				Motion_SOC(tr_1);
				printf("section_case_left\n");
				fprintf(logf, ">> turn right.\n");
				break;

			case skewed_right:
				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> skewed_right.\n");
				Motion_SOC(tl_1);
				fprintf(logf, ">> turn left.\n");
				printf("section_case_right\n");
				break;

			case far_far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is very far.\n");
				Motion_SOC(walk_2);
				fprintf(logf, ">> move forward.\n");
				printf("section_case_far_Far\n");
				break;

			case far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is far.\n");
				Motion_SOC(walk_2);
				fprintf(logf, ">> move forward.\n");
				printf("section_case_far\n");
				break;
			case bit_close:
			printf("section_case_close\n");

			case too_close:
				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is unexpectedly close.\n");
				flg = 0;
				printf("section_case_too_close\n");
				break;

			default:
				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is not detected.\n");
				printf("default\n");
				/*
				fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
				surf = (U16*)malloc(sizeof(U16)*pl);

				read_fpga_video_data(fvda);
				get_surf(surf, fvda);
				free(fvda);

				o_surf = (U16*)malloc(sizeof(U16)*pl);
				n = pure_red(o_surf, surf);
				free(surf);

				fprintf(logf, ">> num of Red pixels : %d \n", n);
				
				direct_camera_display_off();
				clear_screen();
				draw_fpga_video_data_full(o_surf);
				flip();
				direct_camera_display_on();
				
				free(o_surf);
				if (n > 15000)
					flg = 0;
				else
					Motion_SOC(walk_2);
				*/
				flg = 0;
				break;
			}
		}
		else {
			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is fitted.\n");
			break;
		}
	}

	//for (n = 0; n < 2; n++)
	//	Motion_SOC(smallwalk_3);

	fclose(logf);

	return 0;
}

int mot_gotoReadyLine_crevasses(int fcnt, int walk_ver) {
	int state, flg;

	FILE* logf = NULL;
	//U16* fvda;
	//U16* surf;
	//U16* o_surf;
	char fname[50] = { 0 };
	strcat(fname, "log_readyCrevasses");
	
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	flg = 1;
	Motion_SOC(head_std);
	int cnt_crevasses = 0;

		// next we get closer to the line, if line is disapeared, we finish the movement.
	while (flg) {
		direct_camera_display_off();
		state = line_target(logf, 200, fitting, top_porking, default_obs);
		direct_camera_display_on();

		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_left.\n");
				Motion_SOC(tr_1);
				// _wait_d;
				// Motion_SOC(tr_1);
				Motion_SOC(head_down);
				break;
			case skewed_right:   //기울어진..
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_right.\n");
				Motion_SOC(tl_1);
				// _wait_d;
				// Motion_SOC(tl_1);       //190130 revised
				Motion_SOC(head_down);
				break;
			case far_far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is very far.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				flg = 2;
				Motion_SOC(head_2cm);
				Motion_SOC(walk_3);
				Motion_SOC(head_down);
				fprintf(logf, ">> move forward.\n");
				break;
			case far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is far.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				flg = 2;
				Motion_SOC(head_2cm);
				Motion_SOC(walk_1);
				Motion_SOC(head_down);
				fprintf(logf, ">> move forward.\n");
				break;
			case bit_close:

			case too_close:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is unexpectedly close.\n");
				flg = 0;
				break;
			
			default:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is not detected.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				Motion_SOC(head_2cm);
				Motion_SOC(walk_2);
				// mot_arrangeMid(fcnt++);
				Motion_SOC(head_down);
				fprintf(logf, ">> move forward.\n");
				break;
			}
		}
		else {
			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is fitted.\n");
			break;
		}
	}

	// Motion_SOC(walk_2);
	// _wait_d;
	Motion_SOC(head_2cm);
	Motion_SOC(walk_2);
	fclose(logf);
	return 0;
	// while (flg) {
	// 	direct_camera_display_off();
	// 	state = line_target(logf, 100, fitting, bot_porking, default_obs);
	// 	direct_camera_display_on();
	// 	if (state != fitted) {
	// 		if(cnt_crevasses == 5){
	// 			flg = 0;
	// 			break;
	// 		}
	// 		switch (state) {
	// 		case skewed_left:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> skewed_left.\n");
	// 			Motion_SOC(tr_1);
	// 			printf("section_case_left\n");
	// 			fprintf(logf, ">> turn right.\n");
	// 			cnt_crevasses++;
	// 			break;

	// 		case skewed_right:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> skewed_right.\n");
	// 			Motion_SOC(tl_1);
	// 			fprintf(logf, ">> turn left.\n");
	// 			printf("section_case_right\n");
	// 			cnt_crevasses++;
	// 			break;

	// 		case far_far:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is very far.\n");
	// 			Motion_SOC(head_2cm);
	// 			Motion_SOC(walk_3);
	// 			fprintf(logf, ">> move forward.\n");
	// 			printf("section_case_far_Far\n");
	// 			break;

	// 		case far:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is far.\n");
	// 			Motion_SOC(head_2cm);
	// 			Motion_SOC(walk_2);
	// 			fprintf(logf, ">> move forward.\n");
	// 			printf("section_case_far\n");
	// 			break;
	// 		case bit_close:
	// 			printf("section_case_close\n");

	// 		case too_close:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is unexpectedly close.\n");
	// 			flg = 0;
	// 			printf("section_case_too_close\n");
	// 			break;

	// 		default:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is not detected.\n");
	// 			printf("default\n");
	// 			Motion_SOC(head_2cm);
	// 			Motion_SOC(walk_1);
	// 			break;
	// 		}
	// 	}
	// 	else {
	// 		fprintf(logf, "[LOG] mot_gotoReadyLine_crevasses \n>> line is fitted.\n");
	// 		flg = 0;
	// 		break;
	// 	}

	// }

	fclose(logf);

	return 0;
}

int mot_gotoReadyLine_huddle(int fcnt, int walk_ver) {
	int state, flg, false_cnt;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_readyHuddle");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	flg = 1;
	false_cnt = 0;
	Motion_SOC(head_std);

	fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> begin.\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");

	while (flg && (false_cnt < 2)) {
		direct_camera_display_off();
		state = line_target(logf, 0, peeking, bot_porking, default_obs);
		direct_camera_display_on();
		
		switch (state) {
		case skewed_left:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> skewed_left.\n");
			Motion_SOC(tr_1);
			fprintf(logf, ">> turn right.\n");
			break;
		case skewed_right:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> skewed_right.\n");
			Motion_SOC(tl_1);
			fprintf(logf, ">> turn left.\n");
			break;
		case fitted:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is detected.\n");
			flg = 0;
			break;
		default:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is not detected.\n");
			false_cnt++;
			Motion_SOC(head_2cm);
			Motion_SOC(walk_2);
			fprintf(logf, ">> false_cnt = %5d.\n", false_cnt);
			fprintf(logf, ">> move forward.\n");
			break;
		}
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
	}

	flg = 1;
	fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> ready get close to the huddle.\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");

	while (flg) {
		direct_camera_display_off();
		state = line_target(logf, 100, fitting, bot_porking, default_obs);
		direct_camera_display_on();
		switch (state) {
		case skewed_left:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> skewed_left.\n");
			Motion_SOC(tr_1);
			fprintf(logf, ">> turn right.\n");
			break;
		case skewed_right:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> skewed_right.\n");
			Motion_SOC(tl_1);
			fprintf(logf, ">> turn left.\n");
			break;
		case far_far:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is very far.\n");
			Motion_SOC(walk_3);
			fprintf(logf, ">> move forward.\n");
			break;
		case far:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is far.\n");
			Motion_SOC(walk_2);
			fprintf(logf, ">> move forward.\n");
			break;
		case fitted:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is fitted.\n");
			flg = 0;
			break;
		case bit_close:
		case too_close:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is unexpectedly close.\n");
			flg = 0;
			break;
		default:
			fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> line is not detected.\n");
			flg = 0;
			break;
		}
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
	}

	// Motion_SOC(smallwalk_3);

	fprintf(logf, "[LOG] mot_gotoReadyLine_huddle \n>> I'm ready to do dumbling.\n");

	fclose(logf);

	return 0;
}
int mot_arrangeMid(int fcnt) {
	int flg, state;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_arrMid");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");
	// rearrange
	// look side
	Motion_SOC(head_std);
	Motion_SOC(head_right);
	// check line & rearrange
	flg = 1;
	while (1) {
		direct_camera_display_off(); //주석처리 2019/01/08 -> 01/22 주석
		state = peek_bndy(logf);
		direct_camera_display_on(); //주석처리 2019/01/08 -> 01/22 주석
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_left.\n");
				Motion_SOC(trhead_2);
				fprintf(logf, ">> turn right.\n");
				break;
			case skewed_right:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_right.\n");
				Motion_SOC(tlhead_2);
				fprintf(logf, ">> turn left.\n");
				break;
			case far_far:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is very far at the right side.\n");
				Motion_SOC(srhead_3);
				fprintf(logf, ">> move right fast.\n");
				break;
			case far:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is far at the right side.\n");
				Motion_SOC(srhead_2);
				fprintf(logf, ">> move right.\n");
				break;
			case bit_close:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is close at the right side.\n");
				Motion_SOC(slhead_2);
				fprintf(logf, ">> move left.\n");
				break;
			case too_close:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is very close at the right side.\n");
				Motion_SOC(slhead_3);
				fprintf(logf, ">> move left fast.\n");
				break;
			case 7:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> slightly skewed_left.\n");
				Motion_SOC(trhead_1);
				fprintf(logf, ">> little turn right.\n");
				break;
			case 8:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> slightly skewed_right.\n");
				Motion_SOC(tlhead_1);
				fprintf(logf, ">> little turn left.\n");
				break;
			default: 
				// maybe it means too far, so any line was detected.
				// or really the cam captures meaningless pattern...
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is not detected.\n");
				Motion_SOC(slhead_3);
				fprintf(logf, ">> move left fast.\n");
				break;
			}
		}
		else
			break;
	}

	// head rewind
	Motion_SOC(head_pre);
	Motion_SOC(head_std);

	return 0;
}


int mot_arrangeMid_Ver2(int fcnt, int limit) {
	int flg, state;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_arrMid");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");
	int cnt;
	// rearrange
	// look side
	Motion_SOC(head_std);
	Motion_SOC(head_right);
	// check line & rearrange
	flg = 1;
	cnt = 0;
	while (1) {
		direct_camera_display_off(); //주석처리 2019/01/08 -> 01/22 주석
		state = peek_bndy(logf);
		direct_camera_display_on(); //주석처리 2019/01/08 -> 01/22 주석
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_left.\n");
				Motion_SOC(trhead_2);
				fprintf(logf, ">> turn right.\n");
				break;
			case skewed_right:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_right.\n");
				Motion_SOC(tlhead_2);
				fprintf(logf, ">> turn left.\n");
				break;
			case far_far:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is very far at the right side.\n");
				Motion_SOC(srhead_3);
				fprintf(logf, ">> move right fast.\n");
				break;
			case far:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is far at the right side.\n");
				Motion_SOC(srhead_2);
				fprintf(logf, ">> move right.\n");
				break;
			case bit_close:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is close at the right side.\n");
				Motion_SOC(slhead_2);
				fprintf(logf, ">> move left.\n");
				break;
			case too_close:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is very close at the right side.\n");
				Motion_SOC(slhead_3);
				fprintf(logf, ">> move left fast.\n");
				break;
			case 7:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> slightly skewed_left.\n");
				Motion_SOC(trhead_1);
				fprintf(logf, ">> little turn right.\n");
				break;
			case 8:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> slightly skewed_right.\n");
				Motion_SOC(tlhead_1);
				fprintf(logf, ">> little turn left.\n");
				break;
			default: 
				// maybe it means too far, so any line was detected.
				// or really the cam captures meaningless pattern...
				fprintf(logf, "[LOG] mot_arrangeMid \n>> line is not detected.\n");
				Motion_SOC(srhead_3);
				fprintf(logf, ">> move left fast.\n");
				break;
			}
		}
		else{
			if(cnt >= limit) break;
			Motion_SOC(head_pre);
		Motion_SOC(head_std);
			Motion_SOC(walk_2);
			cnt++;
		}
	}

	// head rewind
	Motion_SOC(head_pre);
	Motion_SOC(head_std);

	return 0;
}

//////////////////////
int mot_arrangeMid_jw(int fcnt) {
	int flg, state;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_arrMid_jh");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");
	// rearrange
	// look side

	Motion_SOC(head_right);
	// check line & rearrange
	flg = 1;
	while (1) {
		direct_camera_display_off(); //주석처리 2019/01/08 -> 01/22 주석
		state = peek_bndy(logf);
		direct_camera_display_on(); //주석처리 2019/01/08 -> 01/22 주석
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_left.\n");
				Motion_SOC(tr_2);
				fprintf(logf, ">> turn right.\n");
				break;
			case skewed_right:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_right.\n");
				Motion_SOC(tl_2);
				fprintf(logf, ">> turn left.\n");
				break;
			}
		}
		else
			break;
	}

	// head rewind
	Motion_SOC(head_pre);
	Motion_SOC(head_std);

	return 0;
}



int mot_gotoReadyLine_bdg(int fcnt, int walk_ver) {
	int state, flg;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_readyBdg");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	flg = 1;
	Motion_SOC(head_down);
	while (flg) {
		direct_camera_display_off();
		state = line_target(logf, 100, fitting, bot_porking, default_obs);
		direct_camera_display_on();

		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_left.\n");
				Motion_SOC(tr_1);
				// _wait_d;
				// Motion_SOC(tr_1);
				Motion_SOC(head_down);
				break;
			case skewed_right:   //기울어진..
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_right.\n");
				Motion_SOC(tl_1);
				// _wait_d;
				// Motion_SOC(tl_1);       //190130 revised
				Motion_SOC(head_down);
				break;
			case far_far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is very far.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				flg = 2;
				Motion_SOC(head_2cm);
				Motion_SOC(walk_3);
				Motion_SOC(head_down);
				fprintf(logf, ">> move forward.\n");
				break;
			case far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is far.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				flg = 2;
				Motion_SOC(head_2cm);
				Motion_SOC(walk_1);
				Motion_SOC(head_down);
				fprintf(logf, ">> move forward.\n");
				break;
			case bit_close:

			case too_close:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is unexpectedly close.\n");
				flg = 0;
				break;
			
			default:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is not detected.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				Motion_SOC(head_2cm);
				Motion_SOC(walk_2);
				// mot_arrangeMid(fcnt++);
				Motion_SOC(head_down);
				fprintf(logf, ">> move forward.\n");
				break;
			}
		}
		else {
			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is fitted.\n");
			break;
		}
	}

	// Motion_SOC(walk_2);
	// _wait_d;
	Motion_SOC(head_2cm);
	Motion_SOC(walk_1);
	fclose(logf);
	return 0;
}

int mot_gotoReadyLine_trap_new(int fcnt, int walk_ver) {

	int px, py, cntA, cntB, dstX;
	int cntA1, cntA2, cntA3, cntB1, cntB2, cntB3, dstX1, dstX2, dstX3;
	U8 hue, sat, val;

	int n, state, flg, loop_ctrl;
	
	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_gotoReadyLine_trap_new");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");


	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);
	
	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];

	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];

	//멀리있는 선 보고 기울기 조정(mot_gotoReadyLine_bdg랑 다른 점은 영상 읽을 때 motion이 head_down --> head_2cm로 바뀐거)
	// flg = 1;
	// while (flg) {
	// 	direct_camera_display_off();
	// 	state = line_target(logf, 100, fitting, bot_porking, default_obs);
	// 	direct_camera_display_on();
	// 
	// 	if (state != fitted) {
	// 		switch (state) {
	// 		case skewed_left:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_left.\n");
	// 			Motion_SOC(tr_1);
	// 			// _wait_d;
	// 			// Motion_SOC(tr_1);
	// 			Motion_SOC(head_2cm);
	// 			break;
	// 		case skewed_right:   //기울어진..
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_right.\n");
	// 			Motion_SOC(tl_1);
	// 			// _wait_d;
	// 			// Motion_SOC(tl_1);       //190130 revised
	// 			Motion_SOC(head_2cm);
	// 			break;
	// 		case far_far:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is very far.\n");
	// 			if (flg == 2) {
	// 				flg = 0;
	// 				break;
	// 			}
	// 			flg = 2;
	// 			Motion_SOC(head_2cm);
	// 			Motion_SOC(walk_3);
	// 			Motion_SOC(head_2cm);
	// 			fprintf(logf, ">> move forward.\n");
	// 			break;
	// 		case far:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is far.\n");
	// 			if (flg == 2) {
	// 				flg = 0;
	// 				break;
	// 			}
	// 			flg = 2;
	// 			Motion_SOC(head_2cm);
	// 			Motion_SOC(walk_1);
	// 			Motion_SOC(head_2cm);
	// 			fprintf(logf, ">> move forward.\n");
	// 			break;
	// 		case bit_close:

	// 		case too_close:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is unexpectedly close.\n");
	// 			flg = 0;
	// 			break;
			
	// 		default:
	// 			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is not detected.\n");
	// 			if (flg == 2) {
	// 				flg = 0;
	// 				break;
	// 			}
	// 			Motion_SOC(head_2cm);
	// 			Motion_SOC(walk_2);
	// 			// mot_arrangeMid(fcnt++);
	// 			Motion_SOC(head_2cm);
	// 			fprintf(logf, ">> move forward.\n");
	// 			break;
	// 		}
	// 	}
	// 	else {
	// 		fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is fitted.\n");
	// 		break;
	// 	}
	// }
	Motion_SOC(head_2cm);
	Motion_SOC(walk_2);

	//이거 지금 실행이 안되는거 같다 (까만색 라인 보고 greedbrg처럼 치우침 조정하는거 , walk모션 뺐음!)

	loop_ctrl = 1;

	while(loop_ctrl){
		Motion_SOC(head_down);
		fprintf(logf, "[LOG] mot_runBridge \n>> extra.\n");
		read_fpga_video_data(fvda);
		get_surf(surf, fvda);

		memset(pala_mem, 0, sizeof(U16)*pl);

				
		cntA = 0; 
		cntB = 0; // cntB : number of pixels


		for (py = 60; py < 120; py++) {
			for (px = 0; px < 180; px++) {
				hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
				sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
				val = (U8)(0x000f & mysurf[py][px]);
					//이거 까만색 맞는지 확인 필요!!(hsv값) -> 이거 노란색인데?
				if ((sat < 7 ) && (val < 4)){ // (hue < 70 && hue >= 32) && (sat > 2) && (val > 4)){ --> (sat < 7 ) && (val < 4)){ 
					cntA += px;
					cntB++;					
					pala[py][px] = 0x001f;
				}
			}
		}



		fprintf(logf, ">> num Black = %5d.\n", cntB);
				
				
		dstX = (int)(cntA / cntB);
		for (py = 0; py < 120; py++) {
			pala[py][dstX] = 0xf800;
		}
		fprintf(logf, ">> dstX = %5d.\n", dstX);
	

		clear_screen();
		draw_fpga_video_data_full(pala_mem);
		flip();

		if (dstX < 77) {
			Motion_SOC(sl_3);
			fprintf(logf, ">> move left fast.\n");
		}
		else if (dstX >= 77 && dstX < 87) {
			Motion_SOC(sl_2);
			fprintf(logf, ">> move left.\n");
		}
		else if (dstX >= 87 && dstX <= 97) {
			fprintf(logf, "[LOG] mot_runBridge \n>> line is fitted.\n");
			loop_ctrl = 0;
			fprintf(logf, ">> move forward.\n");
		}
		else if (dstX > 97 && dstX <= 107) {
			Motion_SOC(sr_2);
			fprintf(logf, ">> move right.\n");
		}
		else if (dstX > 107) {
			Motion_SOC(sr_3);
			fprintf(logf, ">> move right fast.\n");
		}
	}
	
	loop_ctrl = 1;
	while (loop_ctrl) {
		Motion_SOC(head_down);
		fprintf(logf, "[LOG] mot_runBridge \n>> extra.\n");
		read_fpga_video_data(fvda);
		get_surf(surf, fvda);

		memset(pala_mem, 0, sizeof(U16)*pl);

				
		// cntA = 0; 
		// cntB = 0; // cntB : number of pixels
		cntA1 = 0; // cntA1 : py가 60이상~80미만
		cntA2 = 0; // cntA2 : py가 80이상~100미만
		cntA3 = 0; // cntA3 : py가 100이상~120미만
		// cntB : number of pixels
		cntB1 = 0;
		cntB2 = 0;
		cntB3 = 0; 

		// black pixels
		for (py = 60; py < 120; py++) {
			for (px = 0; px < 180; px++) {
				hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
				sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
					val = (U8)(0x000f & mysurf[py][px]);
					//이거 까만색 맞는지 확인 필요!!(hsv값) -> 이거 노란색인데?
				if ((sat < 7 ) && (val < 4)){ // (hue < 70 && hue >= 32) && (sat > 2) && (val > 4)){ --> (sat < 7 ) && (val < 4)){ 
			
						
						if(py >= 60 && py <80){
							cntA1 += px;
							cntB1++;
						}
						else if(py >=80 && py <100){
							cntA2 += px;
							cntB2++;
						}
						else if(py > 100 && py <120){
							cntA3 += px;
							cntB3++;
						}
				}
			}
		}
		
		
					// int cntL = 0;
					// int cntR = 0;		
				clear_screen();
				draw_fpga_video_data_full(pala_mem);
				flip();

			
				fprintf(logf, ">> num Green = %5d.\n", cntB);
				dstX1 = (int)(cntA1 / cntB1);
				dstX2 = (int)(cntA2 / cntB2);
				dstX3 = (int)(cntA3 / cntB3);
				for (py = 60; py < 80; py++) {
					pala[py][dstX1] = 0xf800;
				}
				for (py = 80; py < 100; py++) {
					pala[py][dstX2] = 0xf800;
				}
				for (py = 100; py < 120; py++) {
					pala[py][dstX3] = 0xf800;
				}
				fprintf(logf, ">> dstX1 = %5d.\n", dstX1);
				fprintf(logf, ">> dstX2 = %5d.\n", dstX2);
				fprintf(logf, ">> dstX3 = %5d.\n", dstX3);
				dstX = dstX1 + dstX2 + dstX3;
				// clear_screen();
				// draw_fpga_video_data_full(pala_mem);
				// flip();
				int dst = 0; // dstX1과 dstX3중 dstX2보다 더 먼경우 선택
				int tmp1, tmp2;
				tmp1 = dstX1-dstX2; //dstX1
				tmp2 = dstX3-dstX2;	//dstX3


				if(tmp1<0)
					tmp1 = -tmp1;
				if(tmp2<0)
					tmp2 = -tmp2;
				if(cntB1 < 10) tmp1 = 0; 
				if(tmp1>tmp2){
					if(tmp1 < 8)
						loop_ctrl = 0; //무시할 정도가 되었을 때
					else 
						dst = 1;	//dstX1 선택
					// else
					// 	dst = 3;
				}
				else{
					if(tmp2 < 8)
						loop_ctrl = 0; //무시할 정도가 되었을 때
					else 
						dst = 2;	//dstX2선택
					
				}
				switch(dst){
					case 1://dstX1이 더 먼경우
						if(tmp1 > 8)
							Motion_SOC(tr_1);
						
						else if(tmp1 < 0)
							Motion_SOC(tl_1);

						break;
					case 2:
						if(tmp2 > 0)
							Motion_SOC(tl_1);
							
						else if(tmp2 < 0)
							Motion_SOC(tr_1);						
				}
	}	
	
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
			
			// direct_camera_display_on();

	fclose(logf);
	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);
	return 0;
}

int mot_gotoReadyLine_bdg_jw0130(int fcnt, int walk_ver) {
	int state, flg;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_readyBdg");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	flg = 1;
	Motion_SOC(head_bot);

	while (flg) {
		direct_camera_display_off();
		state = peek_vert(logf, 1, default_obs);
		direct_camera_display_on();

		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_left.\n");
				Motion_SOC(tr_1);
				_wait_d;
				Motion_SOC(tr_1);
				Motion_SOC(head_bot);
				break;
			case skewed_right:   //기울어진..
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_right.\n");
				Motion_SOC(tl_1);
				_wait_d;
				Motion_SOC(tl_1);       //190130 revised
				Motion_SOC(head_bot);
				break;

			
			default:

				Motion_SOC(walk_1);
				Motion_SOC(head_bot);
				fprintf(logf, ">> move forward.\n");
				break;
			}
		}
		else {
			
			break;
		}
	}

	Motion_SOC(walk_1);
	_wait_d;
	Motion_SOC(walk_1);
	fclose(logf);
	return 0;
}


int mot_gotoReadyLine_bdg_2(int fcnt, int walk_ver) {
	int state, flg;

	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_readyBdg");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	flg = 1;
	Motion_SOC(walk_1);
	_wait_d;
	Motion_SOC(head_down_2);
	while (flg) {
		direct_camera_display_off();
		state = line_target(logf, 100, fitting, top_porking, default_obs);
		direct_camera_display_on();
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_left.\n");
				Motion_SOC(tr_1);
				Motion_SOC(head_down_2);
				break;
			case skewed_right:   //기울어진..
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> skewed_right.\n");
				Motion_SOC(tl_1);
				Motion_SOC(head_down_2);
				break;
			case far_far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is very far.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				flg = 2;
				Motion_SOC(walk_2);
				Motion_SOC(head_down_2);
				fprintf(logf, ">> move forward.\n");
				break;
			case far:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is far.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				flg = 2;
				Motion_SOC(walk_1);
				Motion_SOC(smallwalk_0);
				Motion_SOC(head_down_2);
				fprintf(logf, ">> move forward.\n");
				break;
			case bit_close:
			case too_close:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is unexpectedly close.\n");
				flg = 0;
				break;
			default:
				fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is not detected.\n");
				if (flg == 2) {
					flg = 0;
					break;
				}
				// Motion_SOC(walk_3);
				Motion_SOC(head_down_2);
				fprintf(logf, ">> move forward.\n");
				break;
			}
		}
		else {
			fprintf(logf, "[LOG] mot_gotoReadyLine_bdg \n>> line is fitted.\n");
			break;
		}
	}

	Motion_SOC(walk_1);
	_wait_d;
	// Motion_SOC(walk_1);
	fclose(logf);
	return 0;
}
// int mot_gotoReadyLine_trap(int onboard, int fcnt, int walk_ver) {
// 	int state, loop_ctrl, flow_ctrl;
	
// 	FILE* logf = NULL;
// 	char fname[50] = { 0 };
// 	strcat(fname, "log_readyTrap");
// 	concat_num(fname, fcnt);
// 	logf = fopen(fname, "w");

// 	fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> begin.\n");

// 	fprintf(logf, "================================================\n");
// 	fprintf(logf, "================================================\n");
// 	fprintf(logf, "================================================\n");
// 	fprintf(logf, "================================================\n");

// 	if (!onboard) {

// 		loop_ctrl = 1;
// 		flow_ctrl = 0;
// 		//revised
// 		// while (loop_ctrl) {
// 		// 	direct_camera_display_off();
// 		// 	state = line_target(logf, 0, peeking, bot_porking, default_obs);
// 		// 	direct_camera_display_on();
// 		// 	switch (state) {
// 		// 	case skewed_left:
// 		// 		fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> skewed_left.\n");
// 		// 		Motion_SOC(tr_1);
// 		// 		Motion_SOC(head_2cm);
// 		// 		fprintf(logf, ">> turn right.\n");
// 		// 		break;
// 		// 	case skewed_right:
// 		// 		fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> skewed_right.\n");
// 		// 		Motion_SOC(tl_1);
// 		// 		Motion_SOC(head_2cm);
// 		// 		fprintf(logf, ">> turn left.\n");
// 		// 		break;
// 		// 	case fitted:
// 		// 		fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> line is detected. try to get closer.\n");
// 		// 		loop_ctrl = 0;
// 		// 		break;
// 		// 	default:
// 		// 		fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> line is not detected.\n");
// 		// 		Motion_SOC(walk_3);
// 		// 		Motion_SOC(head_2cm);
// 		// 		fprintf(logf, ">> move forward.\n");
// 		// 		break;
// 		// 	}
// 		// 	fprintf(logf, "================================================\n");
// 		// 	fprintf(logf, "================================================\n");
// 		// }

// 		// Motion_SOC(walk_3);
// 		fprintf(logf, ">> move forward.\n");
// 		fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> start second sequence.\n");

// 		fprintf(logf, "================================================\n");
// 		fprintf(logf, "================================================\n");
// 		fprintf(logf, "================================================\n");
// 		fprintf(logf, "================================================\n");

// 		loop_ctrl = 1;
// 		flow_ctrl = 1;

// 		while (loop_ctrl) {
// 			direct_camera_display_off();
// 			state = line_target(logf, 0, fitting, bot_porking, default_obs);
// 			direct_camera_display_on();

// 			switch (state) {
// 			case skewed_left:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> skewed_left.\n");
// 				Motion_SOC(tr_1);
// 				Motion_SOC(head_2cm);
// 				fprintf(logf, ">> turn right.\n");
// 				break;
// 			case skewed_right:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> skewed_right.\n");
// 				Motion_SOC(tl_1);
// 				Motion_SOC(head_2cm);
// 				fprintf(logf, ">> turn left.\n");
// 				break;
// 			case far_far:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> line is very far.\n");
// 				/*
// 				if (flow_ctrl < 1) {
// 					loop_ctrl = 0;
// 					break;
// 				}
// 				flow_ctrl--;
// 				*/
// 				Motion_SOC(walk_2);
// 				Motion_SOC(head_2cm);
// 				fprintf(logf, ">> move forward.\n");
// 				break;
// 			case far:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> line is far.\n");
// 				Motion_SOC(walk_2);
// 				//Motion_SOC(walk_2);
// 				//Motion_SOC(smallwalk_3);
// 				Motion_SOC(head_2cm);
// 				fprintf(logf, ">> move forward.\n");
// 				break;
// 			case fitted:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_red \n>> line is fitted.\n");
// 				loop_ctrl = 0;
// 				break;
// 			case bit_close:
// 			case too_close:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> line is unexpectedly close.\n");
// 				loop_ctrl = 0;
// 				break;
// 			default:
// 				fprintf(logf, "[LOG] mot_gotoReadyLine_trap \n>> line is not detected.\n");
// 				loop_ctrl = 0;
// 				break;
// 			}

// 			fprintf(logf, "================================================\n");
// 			fprintf(logf, "================================================\n");
// 		}
// 	}
// /*	else {
// 		Motion_SOC(walk_3);
// 		// Motion_SOC(smallwalk_3);
// 		// Motion_SOC(head_2cm);
// 		// Motion_SOC(walk_2);
// 		// Motion_SOC(smallwalk_3);
// 		// Motion_SOC(head_2cm);
// 		// Motion_SOC(smallwalk_2);
// 		// Motion_SOC(head_2cm);
// 		Motion_SOC(trap_dumbling);
// 	}
// */
// 	fclose(logf);

// 	return 0;
// }
int mot_tarArrMid(int tar) {
	int flg, state;
	// rearrange
	// look side
	Motion_SOC(head_std);
	Motion_SOC(head_right);
	// check line & rearrange
	flg = 1;
	while (1) {
		direct_camera_display_off();
		state = line_target(NULL, tar, fitting, top_porking, default_obs);
		direct_camera_display_on();
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				Motion_SOC(trhead_2);
				break;
			case skewed_right:
				Motion_SOC(tlhead_2);
				break;
			case far_far:
				Motion_SOC(srhead_3);
				break;
			case far:
				Motion_SOC(srhead_2);
				break;
			case bit_close:
				Motion_SOC(slhead_2);
				break;
			case too_close:
				Motion_SOC(slhead_3);
				break;
			default: // maybe it means too far, so any line was detected.
				if (flg) {
					Motion_SOC(srhead_3);
					flg = 0;
				}
				break;
			}
			printf("[Message] state : %d\n", state);
		}
		else
			break;
	}

	// head rewind
	Motion_SOC(head_pre);
	Motion_SOC(head_std);

	return 0;
}
int mot_runBridge(int fcnt, int walk_ver) {
	int px, py, cntA1, cntA2, cntA3,cntA, cntB1, cntB2, cntB3,cntB, dstX1, dstX2, dstX3, dstX;
	U8 hue, sat, val;

	int n, state, flg, loop_ctrl, loop2_ctrl, loop3_ctrl;
	//int flow_ctrl;
	int stateA;//, stateB;
	FILE* logf = NULL;
	char fname[50] = { 0 };
	int cnt_jh=0;
	strcat(fname, "log_runBdg");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);
	
	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];

	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];
	
	mot_arrangeMid(fcnt++);
	Motion_SOC(head_2cm);
	Motion_SOC(walk_1);
	Motion_SOC(smallwalk_2);
	mot_arrangeMid(fcnt++);
	Motion_SOC(head_2cm);	
	Motion_SOC(smallwalk_2);
	_wait_d;

	// Motion_SOC(walk_1);   ///0150 revised
	// _wait_d;
	Motion_SOC(_2cm_up);

	// Motion_SOC(head_std);
	// _wait_d;
	// _wait_d;
	// Motion_SOC(walk_1);
	// _wait_d;
	for(n = 0; n < 2; n++){
		_wait;
		_wait;
	}
	n = 0;
	fprintf(logf, "[LOG] mot_runBridge \n>> start to run on bridge.\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");

	// check line & rearrange
	flg = 0;
	loop_ctrl = 1;

	Motion_SOC(head_2cm);
	// mot_arrangeMid(fcnt++);
	Motion_SOC(walk_4_ver2);
	
	fprintf(logf, "[LOG] mot_runBridge \n>> Now I'm on Bridge.\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");
	fprintf(logf, "================================================\n");

	flg = 1;
	loop_ctrl = 1;
	loop2_ctrl = 1;
	loop3_ctrl = 1;
	//flow_ctrl = 0;
	n = 0;

		direct_camera_display_off();

		// Motion_SOC(head_bar);	

		while(loop3_ctrl) {
			// direct_camera_display_off();

			while (loop_ctrl) { //뒤틀림 맞추는 loop

					Motion_SOC(head_down);
					_wait; 
					fprintf(logf, "[LOG] mot_runBridge \n>> extra.\n");
					read_fpga_video_data(fvda);
					get_surf(surf, fvda);

					memset(pala_mem, 0, sizeof(U16)*pl);

					/*dstX = (int)(cntA / cntB) 에서, cntA += px; 넣을때 py에 따라서 기울어진 정도판단*/


					cntA1 = 0; // cntA1 : py가 60이상~80미만
					cntA2 = 0; // cntA2 : py가 80이상~100미만
					cntA3 = 0; // cntA3 : py가 100이상~120미만
					// cntB : number of pixels
					cntB1 = 0;
					cntB2 = 0;
					cntB3 = 0; 

					// int cntL = 0;
					// int cntR = 0;


							  // green--> black pixels
					for (py = 60; py < 120; py++) {
						for (px = 0; px < 180; px++) {
							hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
							sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
							val = (U8)(0x000f & mysurf[py][px]);		

						if ((hue > 65 && hue <= 110) && (sat > 1) && (val > 1)) {            //  (sat < 7 ) && (val < 3)
							if(py >= 60 && py <80){
								cntA1 += px;
								cntB1++;
							}
							else if(py >=80 && py <100){
								cntA2 += px;
								cntB2++;
							}
							else if(py > 100 && py <120){
								cntA3 += px;
								cntB3++;
							}
							pala[py][px] = 0x001f;
							// if(px <= 35){
							// 	cntL++;
							// }
							// else if(px >=145) cntR++;
						}
					}
				}
						
				cntB = cntB1 + cntB2 + cntB3;
				if(cntB2 < 500) { //원래는 450 
					fprintf(logf, ">> exit condition satisfied.\n");
					loop_ctrl = 0;
					loop2_ctrl = 0;
					loop3_ctrl = 0;
					break;
				}
				fprintf(logf, ">> num Green = %5d.\n", cntB);
				dstX1 = (int)(cntA1 / cntB1);
				dstX2 = (int)(cntA2 / cntB2);
				dstX3 = (int)(cntA3 / cntB3);



				for (py = 60; py < 80; py++) {
					pala[py][dstX1] = 0xf800;
				}
				for (py = 80; py < 100; py++) {
					pala[py][dstX2] = 0xf800;
				}
				for (py = 100; py < 120; py++) {
					pala[py][dstX3] = 0xf800;
				}

				clear_screen();
				draw_fpga_video_data_full(pala_mem);
				flip();

				fprintf(logf, ">> dstX1 = %5d.\n", dstX1);
				fprintf(logf, ">> dstX2 = %5d.\n", dstX2);
				fprintf(logf, ">> dstX3 = %5d.\n", dstX3);
				dstX = dstX1 + dstX2 + dstX3;
				// clear_screen();
				// draw_fpga_video_data_full(pala_mem);
				// flip();
				int dst = 0; // dstX1과 dstX3중 dstX2보다 더 먼경우 선택
				int tmp1, tmp2;
				tmp1 = dstX1-dstX2; //dstX1
				tmp2 = dstX3-dstX2;	//dstX3


				// if(tmp1<0)
				// 	tmp1 = -tmp1;
				// if(tmp2<0)
				// 	tmp2 = -tmp2;
				if(cntB1 < 50) {
					if(dstX3 > dstX2){
						Motion_SOC(tl_1);
					}
					else if(dstX2 > dstX3){
						Motion_SOC(tr_1);
					}
					else{
						loop_ctrl = 0;
						break;
					}
				} 
				if(tmp1>tmp2-5){
					if(tmp1 < 1)
						loop_ctrl = 0; //무시할 정도가 되었을 때
					else 
						dst = 1;	//dstX1 선택
					// else
					// 	dst = 3;
				}
				else{
					if(tmp2 < 1)
						loop_ctrl = 0; //무시할 정도가 되었을 때
					else 
						dst = 2;	//dstX2선택
					
				}
				switch(dst){
					case 1://dstX1이 더 먼경우
						if(tmp1 > 5)
							Motion_SOC(tr_2);
						
						else if(tmp1 <= 5)
							Motion_SOC(tr_1);

						break;
					case 2:
						if(tmp2 > 6)
							Motion_SOC(tl_2);
							
						else if(tmp2 <= 6)
							Motion_SOC(tl_1);
					// case 3:
					// 	if(tmp1 > 0)
					// 		Motion_SOC(tr_2);
							
					// 	else if(tmp1 < 0)
					// 		Motion_SOC(tl_2);
					// case 4:
					// 	if(tmp2 > 0)
					// 		Motion_SOC(tl_2);
							
					// 	else if(tmp2 < 0)
					// 		Motion_SOC(tr_2);
						
				}
	
				dstX1 = 0;
				dstX2 = 0;
				dstX3 = 0;
			}




			while (loop2_ctrl) {
				Motion_SOC(head_down);
				fprintf(logf, "[LOG] mot_runBridge \n>> extra.\n");
				read_fpga_video_data(fvda);
				get_surf(surf, fvda);

				memset(pala_mem, 0, sizeof(U16)*pl);

				
				cntA = 0; 
				cntB = 0; // cntB : number of pixels

						  // green pixels
				for (py = 60; py < 120; py++) {
					for (px = 0; px < 180; px++) {
						hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
						sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
						val = (U8)(0x000f & mysurf[py][px]);

						if ((hue > 65 && hue <= 110) && (sat > 1) && (val > 1)){
							cntA += px;
							cntB++;					
							pala[py][px] = 0x001f;
						}
					}
				}
				fprintf(logf, ">> num Green = %5d.\n", cntB);
				
				
				dstX = (int)(cntA / cntB);
				for (py = 0; py < 120; py++) {
					pala[py][dstX] = 0xfff0;
				}
				fprintf(logf, ">> dstX = %5d.\n", dstX);
				

				clear_screen();
				draw_fpga_video_data_full(pala_mem);
				flip();

				if(n < 3){
					if (dstX < 70) {
						//Motion_SOC(sr_2);
						//fprintf(logf, ">> move right fast.\n");
						Motion_SOC(sl_2);

						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move left fast.\n");
					}
					// else if (dstX >= 70 && dstX < 80) {
					else if (dstX >= 70 && dstX < 90) {
						//Motion_SOC(sr_1);
						//fprintf(logf, ">> move right.\n");
						Motion_SOC(sl_1);
						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move left.\n");
					}
					// //else if (dstX >= 80 && dstX <= 100) {
					else if (dstX >= 90 && dstX <= 100) {
						fprintf(logf, "[LOG] mot_runBridge \n>> line is fitted.\n");
						n++;
						// if(n < 3) {
						Motion_SOC(head_2cm); Motion_SOC(walk_3_ver2);
						// else if (n < 5){
						// 	// Motion_SOC(walk_2);
						// 	// Motion_SOC(walk_2);
						// 	Motion_SOC(head_2cm);
						// 	Motion_SOC(walk_4);
						// 	}
						// else {
						// 	Motion_SOC(head_2cm);
						// 	Motion_SOC(walk_2);
						// }
						loop2_ctrl = 0;
						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move forward.\n");
					}
					else if (dstX > 100 && dstX <= 110) {
						Motion_SOC(sr_1);

						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move right.\n");
					}
					else if (dstX > 110) {
						Motion_SOC(sr_2);
						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move right fast.\n");
					}
					// break;
				}
				else {
					if (dstX < 70) {
						//Motion_SOC(sr_2);
						//fprintf(logf, ">> move right fast.\n");
						Motion_SOC(sl_2);

						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move left fast.\n");
					}
					// else if (dstX >= 70 && dstX < 80) {
					else if (dstX >= 70 && dstX < 80) {
						//Motion_SOC(sr_1);
						//fprintf(logf, ">> move right.\n");
						Motion_SOC(sl_1);
						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move left.\n");
					}
					// //else if (dstX >= 80 && dstX <= 100) {
					else if (dstX >= 80 && dstX <= 90) {
						fprintf(logf, "[LOG] mot_runBridge \n>> line is fitted.\n");
						n++;
						// if(n < 3) {
						// Motion_SOC(head_2cm); Motion_SOC(walk_2_ver2);
						// else if (n < 5){
						// 	// Motion_SOC(walk_2);
						// 	// Motion_SOC(walk_2);
							Motion_SOC(head_2cm);
							Motion_SOC(walk_4);
						// 	}
						// else {
							// Motion_SOC(head_2cm);
							// Motion_SOC(walk_2);
						// }
						loop2_ctrl = 0;
						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move forward.\n");
					}
					else if (dstX > 90 && dstX <= 100) {
						Motion_SOC(sr_1);

						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move right.\n");
					}
					else if (dstX > 100) {
						Motion_SOC(sr_2);
						// Motion_SOC(head_2cm);
						fprintf(logf, ">> move right fast.\n");
					}
				}
			}
			fprintf(logf, "================================================\n");
			fprintf(logf, "================================================\n");
			fprintf(logf, "================================================\n");
			fprintf(logf, "================================================\n");
			
			// direct_camera_display_on();
			loop_ctrl = 1;
			loop2_ctrl = 1;
		}
		
		
			
		//Motion_SOC(walk_3);
		Motion_SOC(head_2cm);

		fprintf(logf, "[LOG] mot_runBridge \n>> Ready to go down the bridge.\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");
		fprintf(logf, "================================================\n");

		flg = 0;
		loop_ctrl = 1;
		loop2_ctrl = 1;
		loop3_ctrl = 1;
		// flow_ctrl = 0;
		int count;
		int loopcnt = 0;

		Motion_SOC(head_bar);
		_wait_d;


		direct_camera_display_off();


		Motion_SOC(head_2cm);
		Motion_SOC(walk_2);


		while(1){
			
			count = 0;

			if(loopcnt %3 == 2) mot_arrangeMid(fcnt++);
			Motion_SOC(head_down_2);
			_wait;

			read_fpga_video_data(fvda);
			get_surf(surf, fvda);
			memset(pala_mem, 0, sizeof(U16)*pl);
			

			for (py = 60; py < 120; py++) {
				for (px = 0; px < 180; px++) {
					//hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
					sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
					val = (U8)(0x000f & mysurf[py][px]);

					if (val < 3) {
						pala[py][px] = 0x00ff;

						// if((py>80&&py<120)&&(px>15&&px<165)) count++;
					}
					else
						pala[py][px] = 0x0000;
				}
			}
			for (py = 70; py < 120; py++) {
				for (px = 15; px < 165; px++) {
					if(pala[py][px] == 0x00ff){
						count++;
					}
				}
			}
			

			clear_screen();
			draw_fpga_video_data_full(pala_mem);
			flip();

			if(count > 2000){ Motion_SOC(head_2cm); Motion_SOC(walk_1); _wait;} // 10 * 180 pixel = 1800 8줄 이하로 보일 때 out
			else if(count > 1500) {Motion_SOC(head_2cm); Motion_SOC(smallwalk_2); break;}
			else break;
			loopcnt++;

		}

	direct_camera_display_on();


	clear_screen();
	draw_fpga_video_data_full(pala_mem);
	flip();



	//Motion_SOC(smallwalk_0);
	//_wait_250ms;
	//Motion_SOC(_2cm_down);


	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);

	fclose(logf);

	return 0;
}


int watch_waitGreenLight(int fcnt) {
	int state;
	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_yellowbar");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");

	fprintf(logf, "[LOG] watch_waitGreenLight \n>> func Entered.\n");

	Motion_SOC(head_bar);
	direct_camera_display_off();
	while (1) {
		state = peek_start(logf);
		if (state)
			break;
	}
	
	fprintf(logf, "[LOG] watch_waitGreenLight \n>> yellow bar detected.\n");

	while (1) {
		state = peek_start(logf);
		if (!state)
			break;
	}
	direct_camera_display_on();

	fprintf(logf, "[LOG] watch_waitGreenLight \n>> yellow bar removed.\n");
	fclose(logf);
	return 0;
}





///////////////////////////////////new 0115

int watch_dmz(int fcnt) {
	int state;
	FILE* logf = NULL;
	char fname[50] = { 0 };
	strcat(fname, "log_dmz");
	concat_num(fname, fcnt);
	logf = fopen(fname, "w");
	int cnt = 0;
	fprintf(logf, "[LOG] watch_dmz \n>> func Entered.\n");

	direct_camera_display_off();
	while (1) {
	state = peek_start_dmz(logf, cnt);
	cnt++;
		 if (state)
		 	break;
	 }
	direct_camera_display_on();

	fclose(logf);
	return 0;
}

//////////////////////////////




int tester_trap(void) {
	int n, t, m;
	int xpt[9] = { 0 };
	int done[9] = { 0 };
	// 화면에 노란 색상 이진화

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pallate = (U16*)malloc(sizeof(U16)*pl);

	read_fpga_video_data(fvda);
	get_surf(surf, fvda);
	colorRegion(pallate, surf, YELLOW);

	for (n = 10; n < ih; n++) {
		for (t = 0; t < 9; t++) {
			m = 18 * (t + 1);
			if (pallate[XY(m, n)] && !done[t]) {
				xpt[t] = n;
				done[t] = 1;
			}
			else {
				if (done[t])
					pallate[XY(m, n)] = 0xf800;
				else
					pallate[XY(m, n)] = 0x07e0;
			}
		}
	}

	printf("=================================\n");
	printf("xptr|");
	for (t = 0; t < 9; t++) {
		m = 18 * (t + 1);
		printf(" [%3d] |", m);
	}
	printf("\n");
	printf("capt|");
	for (t = 0; t < 9; t++) {
		printf(" %4d  |", xpt[t]);
	}
	printf("\n");
	printf("=================================\n");

	clear_screen();
	draw_fpga_video_data_full(pallate);
	flip();
	free(fvda);
	free(surf);
	free(pallate);

	return 0;
}

int make_flat(void) {
	int flg, state;
	Motion_SOC(head_std);
	Motion_SOC(head_right);

	flg = 1;
	while (flg) {
		direct_camera_display_off();
		state = peek_bndy(NULL);
		direct_camera_display_on();
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				Motion_SOC(trhead_2);
				break;
			case skewed_right:
				Motion_SOC(tlhead_2);
				break;
			case far_far:
			case far:
			case bit_close:
			case too_close:
			default:
				flg = 0;
				break;
			}
		}
		else
			break;
		printf("[Message] state : %d\n", state);
	}
	Motion_SOC(head_pre);
	Motion_SOC(head_std);
	return 0;
}
