#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "amazon2_sdk.h"
#include "graphic_api.h"
#include "uart_api.h"
#include "robot_protocol.h"
#include "stdip_ex.h"


void morphology_Erode(U16 **input_Image,U16 **output_Image,int size_Height, int size_Width)
{
	int hit;
	int y, x, i, j;

	//image cloning
	for (y = 0; y < HEIGHT; y++)
		for ( x = 0; x < WIDTH; x++)
		{
			output_Image[y][x] = input_Image[y][x];
		}

	//5*5 erode
	for ( y = 0; y < HEIGHT - size_Height; y++)
		for ( x = 0; x < WIDTH - size_Width; x++)
		{
			hit = 0;
			for ( i = 0; i < size_Height; i++)
				for ( j = 0; j < size_Width; j++)
					if (input_Image[y + i][x + j] == 0xffff)
						hit++;

			if (hit < size_Height*size_Width) {
				for ( i = 0; i < size_Height; i++)
					for ( j = 0; j < size_Width; j++)
						output_Image[y + i][x + j] = 0;
			}

		}
}
//팽창
void morphology_Dilate(U16 **input_Image, U16 **output_Image, int size_Height, int size_Width)
{
	
	int hit;
	int x, y, i, j;
	//image cloning
	for ( y = 0; y < HEIGHT; y++)
		for ( x = 0; x < WIDTH; x++)
		{
			output_Image[y][x] = input_Image[y][x];
		}

	for ( y = 0; y < HEIGHT - size_Height; y++)
		for ( x = 0; x < WIDTH - size_Width; x++)
		{
			hit = 0;
			for ( i = 0; i < size_Height; i++)
				for ( j = 0; j < size_Width; j++)
					if (input_Image[y + i][x + j] == 0xffff)
						hit++;

			if (hit > 2) {
				for ( i = 0; i < size_Height; i++)
					for ( j = 0; j < size_Width; j++)
						output_Image[y][x] = 0xffff;
			}

		}
	
}


int mot_arrangeMid_2(int fcnt) {
	int flg, state;
	int cnt = 0;

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
		// direct_camera_display_off(); //주석처리 2019/01/08 -> 01/22 주석
		state = peek_bndy_2(logf);
		if (cnt == 4 && (state == 0 || state == 7)) state = 1;
		else if(cnt == 4 && (state == 1 && state == 8)) state = 0;
		// if (cnt >= 7 && (state >= -1 && state < 2)) state = far_far;
		// direct_camera_display_on(); //주석처리 2019/01/08 -> 01/22 주석
		if (state != fitted) {
			switch (state) {
			case skewed_left:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_left.\n");
				Motion_SOC(trhead_2);
				cnt++;
				fprintf(logf, ">> turn right.\n");
				break;
			case skewed_right:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> skewed_right.\n");
				Motion_SOC(tlhead_2);
				cnt++;
				fprintf(logf, ">> turn left.\n");
				break;
			// case far_far:
			// 	fprintf(logf, "[LOG] mot_arrangeMid \n>> line is very far at the right side.\n");
			// 	Motion_SOC(srhead_3);
			// 	fprintf(logf, ">> move right fast.\n");
			// 	break;
			// case far:
			// 	fprintf(logf, "[LOG] mot_arrangeMid \n>> line is far at the right side.\n");
			// 	Motion_SOC(srhead_2);
			// 	fprintf(logf, ">> move right.\n");
			// 	break;
			// case bit_close:
			// 	fprintf(logf, "[LOG] mot_arrangeMid \n>> line is close at the right side.\n");
			// 	Motion_SOC(slhead_2);
			// 	fprintf(logf, ">> move left.\n");
			// 	break;
			// case too_close:
			// 	fprintf(logf, "[LOG] mot_arrangeMid \n>> line is very close at the right side.\n");
			// 	Motion_SOC(slhead_3);
			// 	fprintf(logf, ">> move left fast.\n");
			// 	break;
			case 7:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> slightly skewed_left.\n");
				Motion_SOC(trhead_1);
				cnt++;
				fprintf(logf, ">> little turn right.\n");
				break;
			case 8:
				fprintf(logf, "[LOG] mot_arrangeMid \n>> slightly skewed_right.\n");
				Motion_SOC(tlhead_1);
				cnt++;
				fprintf(logf, ">> little turn left.\n");
				break;
			case far_far:
			case far:
					Motion_SOC(srhead_2);
					cnt++;
					break;
			case bit_close:
			case too_close:					
					Motion_SOC(slhead_2);
					cnt++;
					break;
			default: 
				// maybe it means too far, so any line was detected.
				// or really the cam captures meaningless pattern...
				// fprintf(logf, "[LOG] mot_arrangeMid \n>> line is not detected.\n");
				// Motion_SOC(trhead_3);
				cnt++;
				Motion_SOC(slhead_3);
				// fprintf(logf, ">> move left fast.\n");
				break;
			}
		}
		else
			break;

		if (cnt > 4) break;
	}

	// head rewind
	Motion_SOC(head_pre);
	Motion_SOC(head_std);

	return 0;
}

// primative functions
int Motion_walker(MOTION motion, int ver) {
	int flg;
	unsigned char motion_c = 0;
	unsigned char ack = 0;

	motion_c = (unsigned char)(0x000000ff & motion);

	switch (ver) {
	case 1:
		break;
	case 2:
		motion_c += 50;
		break;
	case 3:
		motion_c += 60;
		break;
	default:
		break;
	}

	ack = Motion_ack(motion_c, 1);

	if ((ack == 44) ||
		(ack == 100) ||
		(ack == 120)) {
		flg = 1;
	}
	else {
		printf("UART communication failed\n");
		flg = 0; //flg = -1; 2019.01.08
	}
	return flg;
}

void Motion_while(MOTION motion){
	int c;
	while(1){
		c = Motion_SOC(motion);
		if(c == 1) break;
	}
}

int Motion_SOC(MOTION motion) {
	FILE* logf = NULL;//20190108/
	int flg;
	int *cursor;
	long starttime = 0, totalpassedtime = 0;
	unsigned char motion_c = 0;
	unsigned char ack = 0;
	char fname[50] = { 0 };
	////////////20190108/////////////////
   	strcat(fname, "log_readyRed");
   	logf = fopen(fname, "w");
//////////////////////////////////
   	int check = 1;
   	int count = 0;
	motion_c = (unsigned char)(0x000000ff & motion);
	starttime = clock();
	while(1){
		ack = Motion_ack(motion_c, check);
		totalpassedtime = (clock()-starttime)/CLOCKS_PER_SEC;
		printf("printf -> actnum : %d \n", ack);
		if(motion == con_check) break;
		else if(ack == 200 || ack == 44 || ack == 144) break;
		else if(motion == bulldozer || motion == trap_dumbling || motion == huddle_dumbling || motion == _2cm_up || motion == _2cm_down || motion == tl_90 || motion == tr_90 || motion == crevasse) 	{_wait; _wait;}	
		if(totalpassedtime > 0.2){
			if(motion != bulldozer && motion != trap_dumbling && motion != huddle_dumbling && motion != _2cm_up && motion != _2cm_down && motion != tl_90 && motion != tr_90 || motion == crevasse) 
				break;
			else if(totalpassedtime > 4) break;
		}
		check = 0; _wait;
		// // else if(motion == bulldozer || motion == trap_dumbling) {check = 0; /*_wait; _wait; _wait; _wait; _wait; _wait; _wait; _wait;*/ if(ack == 200 || ack == 44) break;}
		// // else if(ack == 255) {check = 0;/* _wait; _wait_250ms;*/} //0123 ack == 44 || 제거
		// else if(ack == 200 || ack == 44) {/*_wait_250ms;*/ break;}
		// check = 0; _wait;

	}

	//////////////////
	fprintf(logf, ">> ack: %c actnum : %d \n", ack,ack);

	/////////////////



	// next_update
	// gyro ban list
	/*
	if (// Æ¯¼ö ¸ð¼Ç
		(motion != huddle_dumbling) &&a
		(motion != trap_dumbling) &&
		(motion != bulldozer) &&
		(motion != con_check) &&
		(motion != wakeup_front) &&
		(motion != wakeup_back) &&
		// °í°³ ¸ð¼Ç
		(motion != head_2cm) && 
		(motion != head_bar) && 
		(motion != head_right) && 
		(motion != head_std) && 
		(motion != head_green) && 
		(motion != head_bot) &&
		(motion != head_blue_hole) && 
		(motion != head_blue_hole_new) && 
		(motion != head_pre) &&
		// °í°³ µ¹¸° ÈÄ, ÁÂ¿ì ÀÌµ¿ ¸ð¼Ç
		(motion != tlhead_1) &&
		(motion != tlhead_2) &&
		(motion != tlhead_3) &&
		(motion != trhead_1) &&
		(motion != trhead_2) &&
		(motion != trhead_3) &&
		(motion != slhead_1) &&
		(motion != slhead_2) &&
		(motion != slhead_3) &&
		(motion != slhead_4) &&
		(motion != srhead_1) &&
		(motion != srhead_2) &&
		(motion != srhead_3) &&
		(motion != srhead_4)) {
		if (ack == 44)
			// Á¤»ó.
			flg = 1;
		else if (ack == 100) {
			// ¾ÕÀ¸·Î ³Ñ¾îÁ³À» ¶§,
			Motion_SOC(wakeup_front);
			flg = 0;
		}
		else if (ack == 200) {
			// µÚ·Î ³Ñ¾îÁ³À» ¶§.
			Motion_SOC(wakeup_back);
			flg = 0;
		}
		else {
			// ¿¹¿Ü.
			printf("UART communication failed\n");
			flg = -1;
		}
	}
	*/
	//else

	if (ack == 144) {
			// ¾ÕÀ¸·Î ³Ñ¾îÁ³À» ¶§,
		Motion_SOC(wakeup_front);
		flg = 0;
	}

	if (ack == 200) {
		flg = 1;
	}
	else {
		printf("UART communication failed\n");
		flg = 0; //flg = -1; 2019.01.08
	}

	///////////0118////////////
	// while(1){
	// 	if(flg == 1)
	// 		break;
	// }
	//////////////////////////
	/*
	// old_version

	if (ack == 44)
		// Á¤»ó.
		flg = 1;
	else if (ack == 100)
		// ¾ÕÀ¸·Î ³Ñ¾îÁ³À» ¶§,
		flg = 0;
	else {
		// ¿¹¿Ü.
		printf("UART communication failed\n");
		flg = -1;
	}
	*/
	return flg;
}

////////////////0118 Motion_log
int Motion_SOC_log(MOTION motion, int log){
	FILE* logf = NULL;//20190108/
	int flg;
	unsigned char motion_c = 0;
	unsigned char ack = 0;
	char fname[50] = { 0 };
	////////////20190108/////////////////
   	strcat(fname, "log_readyRed");
   	concat_num(fname, log);
   	logf = fopen(fname, "w");
//////////////////////////////////
   	int check = 1;
   	int count = 0;
	motion_c = (unsigned char)(0x000000ff & motion);
	while(1){
		ack = Motion_ack(motion_c, check);
		if(motion == con_check) break;
		else if(ack == 255) {check = 0;_wait; _wait_250ms;}
		else if(ack == 200 || ack == 44) {_wait; break;}
		else if(count == 3000) break;
		else count++;
	
	}

	//////////////////
	fprintf(logf, ">> ack: %c actnum : %d \n", ack,ack);

	/////////////////

	

	// next_update
	// gyro ban list
	/*
	if (// Æ¯¼ö ¸ð¼Ç
		(motion != huddle_dumbling) &&a
		(motion != trap_dumbling) &&
		(motion != bulldozer) &&
		(motion != con_check) &&
		(motion != wakeup_front) &&
		(motion != wakeup_back) &&
		// °í°³ ¸ð¼Ç
		(motion != head_2cm) && 
		(motion != head_bar) && 
		(motion != head_right) && 
		(motion != head_std) && 
		(motion != head_green) && 
		(motion != head_bot) &&
		(motion != head_blue_hole) && 
		(motion != head_blue_hole_new) && 
		(motion != head_pre) &&
		// °í°³ µ¹¸° ÈÄ, ÁÂ¿ì ÀÌµ¿ ¸ð¼Ç
		(motion != tlhead_1) &&
		(motion != tlhead_2) &&
		(motion != tlhead_3) &&
		(motion != trhead_1) &&
		(motion != trhead_2) &&
		(motion != trhead_3) &&
		(motion != slhead_1) &&
		(motion != slhead_2) &&
		(motion != slhead_3) &&
		(motion != slhead_4) &&
		(motion != srhead_1) &&
		(motion != srhead_2) &&
		(motion != srhead_3) &&
		(motion != srhead_4)) {
		if (ack == 44)
			// Á¤»ó.
			flg = 1;
		else if (ack == 100) {
			// ¾ÕÀ¸·Î ³Ñ¾îÁ³À» ¶§,
			Motion_SOC(wakeup_front);
			flg = 0;
		}
		else if (ack == 200) {
			// µÚ·Î ³Ñ¾îÁ³À» ¶§.
			Motion_SOC(wakeup_back);
			flg = 0;
		}
		else {
			// ¿¹¿Ü.
			printf("UART communication failed\n");
			flg = -1;
		}
	}
	*/
	//else
	if (ack == 200) {
		flg = 1;
	}
	else {
		printf("UART communication failed\n");
		flg = 0; //flg = -1; 2019.01.08
	}

	///////////0118////////////
	// while(1){
	// 	if(flg == 1)
	// 		break;
	// }
	//////////////////////////
	/*
	// old_version

	if (ack == 44)
		// Á¤»ó.
		flg = 1;
	else if (ack == 100)
		// ¾ÕÀ¸·Î ³Ñ¾îÁ³À» ¶§,
		flg = 0;
	else {
		// ¿¹¿Ü.
		printf("UART communication failed\n");
		flg = -1;
	}
	*/
	_wait_250ms;
	return flg;
}
// itoa func
// dst ¹®ÀÚ¿­ µÚ¿¡ [num].txt ¹®ÀÚ¿­À» »ðÀÔÇÑ´Ù. numÀº À½¼ö°¡ ¾Æ´Ñ Á¤¼öÀÌ¾î¾ß ÇÑ´Ù.
int concat_num(char* dst, int num) {
	int cptr, d_num, t_num, dec, n;
	char str[20] = { 0 };

	d_num = num;

	if (d_num < 0) {
		return -1;
	}

	cptr = 0;
	dec = 0;
	if (d_num == 0) {
		str[cptr] = 48;
	}
	else {
		// ÀÚ¸´¼ö ÆÄ¾Ç
		for (;;) {
			if (d_num == 0)
				break;
			d_num = (int)d_num / 10;
			dec++;
		}
	}

	// dec ÀÚ¸´¼ö ¼ýÀÚ.
	t_num = num;
	for (n = dec; n > 0; n--) {
		d_num = (int)(t_num / pow((double)10, (double)(n - 1)));
		str[cptr++] = d_num + 48;
		t_num = (int)(t_num % (int)pow((double)10, (double)(n - 1)));
	}

	strcat(dst, str);
	strcat(dst, ".txt");

	return 0;
}
// get htda from raw datas(fvda)
int get_htda(U16* htda, U16* fvda) {
	int n;
	int htda_num = 0;

	memcpy(htda, fvda + pl, hl * 2);

	for (n = 0; n < hl; n++) {
		if (htda[n] == 0xdcdc)
			break;
		else
			htda_num++;
	}

	return htda_num;
}
// get surf from raw datas(fvda)
int get_surf(U16* surf, U16* fvda) {
	memcpy(surf, fvda, pl * 2);
	return 0;
}
// drawing surf on lcd.
// panel slot : 
//	1, 2, 3, 4
// color param :
//	1-red, 2-green, 3-blue
void draw_surf(U16* surf, int slot, int color) {
	int n;
	U16 x;
	U16* palette = (U16*)malloc(sizeof(U16)*pl);

	for (n = 0; n < pl; n++) {
		switch (color) {
		case 1: x = 0xf800; break;
		case 2: x = 0x07e0; break;
		case 3: x = 0x001f; break;
		default: x = 0xffff; break;
		}

		if (surf[n])
			palette[n] = x;
		else
			palette[n] = 0x0000;
	}

	switch (slot) {
	case 1: draw_fpga_video_data(palette, 10, 10); break;
	case 2: draw_fpga_video_data(palette, 10, 130); break;
	case 3: draw_fpga_video_data(palette, 10, 250); break;
	case 4: draw_fpga_video_data(palette, 10, 370); break;
	default: break;
	}

	free(palette);

	return;
}
// hue, sat, val¿¡ µû¶ó »ö»ó °áÁ¤
COLOR colorpeek(U8 hue, U8 sat, U8 val) {
	COLOR flg;
	COLOR n;

	flg = NONE;

	for (n = WHITE; n < NONE; n++) {
		switch (n) {
		case RED:
			if ((hue < 20 || hue >= 210) && (sat > 1) && (val > 3))
				flg = RED;
			break;
		case ORANGE:
			if ((hue < 32 && hue >= 10) && (sat > 1) && (val > 3))
				flg = ORANGE;
			break;
		case YELLOW:
			if ((hue < 70 && hue >= 32) && (sat > 2) && (val > 3))
				flg = YELLOW;
			break;
		case GREEN:
			if ((hue < 125 && hue >= 78) && (sat > 3) && (val > 3))
				flg = GREEN;
			break;
		case BLUE:
			if ((hue < 220 && hue >= 150) && (sat > 3) && (val > 3))
				flg = BLUE;
			break;
		case WHITE:
			if ((sat < 4) && (val > 4))
				flg = WHITE;
			break;
		case BLACK:
			if(val<3)
				flg=BLACK;
		default:
			break;
		}
	}

	//printf("h/s/v : %4d / %4d / %4d \n", hue, sat, val);
	return flg;
}
void sideRegionCheck(U16* surf, int rho, int isleft, int tag) {
	int n, delta;
	U16 val = 0x0000;
	int local_rho = 0;

	local_rho = 90 + rho;

	if (local_rho < 0) {
		local_rho = 0;
	}
	else if (local_rho >= iw)
		local_rho = iw - 1;

	for (n = 0; n < 30; n++) { // 180 * 30 = 5400
							   // local_rho ·ÎºÎÅÍ ÁÂ·Î 30 ÇÈ¼¿ Á¶»ç 
		if (isleft) {
			delta = local_rho - n;
			if (delta < 0)
				break;
		}
		// local_rho ·ÎºÎÅÍ ¿ì·Î 30 ÇÈ¼¿ Á¶»ç 
		else {
			delta = local_rho + n;
			if (delta >= iw)
				break;
		}
		switch (tag) {
		case 0:
			val = 0xf800;
			break;
		case 1:
			val = 0x07e0;
			break;
		case 2:
			val = 0x001f;
			break;
		case 3:
			val = 0x7807;
			break;
		default:
			val = 0x0000;
		}
		surf[5220 + delta] = val;
		surf[5400 + delta] = val;
		surf[5580 + delta] = val;
	}
	return;
}

void sideRegionCheck2(U16* surf, int rho, int isleft, int tag) {
	int n, delta;
	U16 val = 0x0000;
	int local_rho = 0;

	local_rho = rho+90;

	if (local_rho < 0) {
		local_rho = 0;
	}
	else if (local_rho >= ih)
		local_rho = ih - 1;

	for (n = 60; n < 120; n++) { // 180 * 30 = 5400
							   // local_rho ·ÎºÎÅÍ ÁÂ·Î 30 ÇÈ¼¿ Á¶»ç 
		if (isleft) {
			delta = local_rho - n;
			if (delta < 0)
				break;
		}
		// local_rho ·ÎºÎÅÍ ¿ì·Î 30 ÇÈ¼¿ Á¶»ç 
		else {
			delta = local_rho + n;
			if (delta >= ih)
				break;
		}
		switch (tag) {
		case 0:
			val = 0xf800;
			break;
		case 1:
			val = 0x07e0;
			break;
		case 2:
			val = 0x001f;
			break;
		case 3:
			val = 0x7807;
			break;
		default:
			val = 0x0000;
		}
		surf[5220 + delta] = val;
		surf[5400 + delta] = val;
		surf[5580 + delta] = val;
	}
	return;
}

COLOR sideLookupColor(U16* surf, int rho, int isleft) {
	int n, delta;
	int local_cnt = 0;
	int local_rho = 0;
	int cal_h = 0;
	int cal_s = 0;
	int cal_v = 0;

	local_rho = 90 + rho;

	if (local_rho < 0) {
		local_rho = 0;
	}
	else if (local_rho >= iw)
		local_rho = iw - 1;

	// y = 30 ±âÁØ
	for (n = 10; n < 40; n++) { // 180 * 30 = 5400
								// local_rho ·ÎºÎÅÍ ÁÂ·Î 30 ÇÈ¼¿ Á¶»ç 
		if (isleft) {
			delta = local_rho - n;
			if (delta < 0)
				break;
		}
		// local_rho ·ÎºÎÅÍ ¿ì·Î 30 ÇÈ¼¿ Á¶»ç 
		else {
			delta = local_rho + n;
			if (delta >= iw)
				break;
		}
		cal_h += (U8)((0xff00 & surf[5220 + delta]) >> 8);
		cal_h += (U8)((0xff00 & surf[5400 + delta]) >> 8);
		cal_h += (U8)((0xff00 & surf[5580 + delta]) >> 8);
		cal_s += (U8)((0x00f0 & surf[5220 + delta]) >> 4);
		cal_s += (U8)((0x00f0 & surf[5400 + delta]) >> 4);
		cal_s += (U8)((0x00f0 & surf[5580 + delta]) >> 4);
		cal_v += (U8)(0x000f & surf[5220 + delta]);
		cal_v += (U8)(0x000f & surf[5400 + delta]);
		cal_v += (U8)(0x000f & surf[5580 + delta]);
		local_cnt += 3;
	}

	cal_h = (int)(cal_h / local_cnt);
	cal_s = (int)(cal_s / local_cnt);
	cal_v = (int)(cal_v / local_cnt);

#ifdef lcd_dbg
	printf("local_cnt / 3 : %5d \n ", (int)(local_cnt / 3));
	printf("h / s / v : %5d / %5d / %5d \n", cal_h, cal_s, cal_v);
#endif

	return colorpeek((U8)cal_h, (U8)cal_s, (U8)cal_v);
}
int sideLookupCRatio(U16* surf, COLOR user_color, int rho, int ypeek, int isleft) {
	int n, m, delta;
	int local_cnt = 0;
	int local_rho = 0;
	int cal_h[3] = { 0 };
	int cal_s[3] = { 0 };
	int cal_v[3] = { 0 };
	int bndy_ok[3] = { 0 };
	int y0 = 0;
	int y1 = 0;
	int y2 = 0;

	local_rho = 90 + rho;

	if (local_rho < 0) {
		local_rho = 0;
	}
	else if (local_rho >= iw)
		local_rho = iw - 1;

	// y = 30 ±âÁØ
	for (n = 10; n < 40; n++) { // 180 * 30 = 5400
								// local_rho ·ÎºÎÅÍ ÁÂ·Î 30 ÇÈ¼¿ Á¶»ç 
		if (isleft) {
			delta = local_rho - n;
			if (delta < 0)
				break;
		}
		// local_rho ·ÎºÎÅÍ ¿ì·Î 30 ÇÈ¼¿ Á¶»ç 
		else {
			delta = local_rho + n;
			if (delta >= iw)
				break;
		}
		y0 = iw*(ypeek - 1);
		y1 = y0 + iw;
		y2 = y1 + iw;

		// ¹Ù¿î´õ¸® º¸È£
		if (y0 >= 0 && y0 <= (pl - iw)) {
			cal_h[0] = (U8)((0xff00 & surf[y0 + delta]) >> 8);
			cal_s[0] = (U8)((0x00f0 & surf[y0 + delta]) >> 4);
			cal_v[0] = (U8)(0x000f & surf[y0 + delta]);
			bndy_ok[0] = 1;
		}
		if (y1 >= 0 && y1 <= (pl - iw)) {
			cal_h[1] = (U8)((0xff00 & surf[y1 + delta]) >> 8);
			cal_s[1] = (U8)((0x00f0 & surf[y1 + delta]) >> 4);
			cal_v[1] = (U8)(0x000f & surf[y1 + delta]);
			bndy_ok[1] = 1;
		}
		if (y2 >= 0 && y2 <= (pl - iw)) {
			cal_h[2] = (U8)((0xff00 & surf[y2 + delta]) >> 8);
			cal_s[2] = (U8)((0x00f0 & surf[y2 + delta]) >> 4);
			cal_v[2] = (U8)(0x000f & surf[y2 + delta]);
			bndy_ok[2] = 1;
		}

		for (m = 0; m < 3; m++)
			if (bndy_ok[m]) {
				if ((colorpeek((U8)cal_h[m], (U8)cal_s[m], (U8)cal_v[m]) == user_color))

					local_cnt++;
				bndy_ok[m] = 0;
			}
	}

	return local_cnt;
}

int sideLookupCRatio_jh(U16* surf, COLOR user_color, int rho, int xpeek, int isleft) {
	int n, m, delta;
	int local_cnt = 0;
	int local_rho = 0;
	int cal_h[3] = { 0 };
	int cal_s[3] = { 0 };
	int cal_v[3] = { 0 };
	int bndy_ok[3] = { 0 };
	int x0 = 0;
	int x1 = 0;
	int x2 = 0;

	local_rho = rho+90;

	if (local_rho < 0) {
		local_rho = 0;
	}
	else if (local_rho >= ih)
		local_rho = ih - 1;

	// y = 30 ±âÁØ
	for (n = 60; n < 120; n++) { // 180 * 30 = 5400
								// local_rho ·ÎºÎÅÍ ÁÂ·Î 30 ÇÈ¼¿ Á¶»ç 
		if (isleft) {
			delta = local_rho - n;
			if (delta < 0)
				break;
		}
		// local_rho ·ÎºÎÅÍ ¿ì·Î 30 ÇÈ¼¿ Á¶»ç 
		else {
			delta = local_rho + n;
			if (delta >= ih)
				break;
		}
		x0 = ih*(xpeek - 1);
		x1 = x0 + ih;
		x2 = x1 + ih;

		// ¹Ù¿î´õ¸® º¸È£
		if (x0 >= 0 && x0 <= (pl - ih)) {
			cal_h[0] = (U8)((0xff00 & surf[x0 + delta]) >> 8);
			cal_s[0] = (U8)((0x00f0 & surf[x0 + delta]) >> 4);
			cal_v[0] = (U8)(0x000f & surf[x0 + delta]);
			bndy_ok[0] = 1;
		}
		if (x1 >= 0 && x1 <= (pl - ih)) {
			cal_h[1] = (U8)((0xff00 & surf[x1 + delta]) >> 8);
			cal_s[1] = (U8)((0x00f0 & surf[x1 + delta]) >> 4);
			cal_v[1] = (U8)(0x000f & surf[x1 + delta]);
			bndy_ok[1] = 1;
		}
		if (x2 >= 0 && x2 <= (pl - ih)) {
			cal_h[2] = (U8)((0xff00 & surf[x2 + delta]) >> 8);
			cal_s[2] = (U8)((0x00f0 & surf[x2 + delta]) >> 4);
			cal_v[2] = (U8)(0x000f & surf[x2 + delta]);
			bndy_ok[2] = 1;
		}

		for (m = 0; m < 3; m++)
			if (bndy_ok[m]) {
				if ((colorpeek((U8)cal_h[m], (U8)cal_s[m], (U8)cal_v[m]) == user_color))

					local_cnt++;
				bndy_ok[m] = 0;
			}
	}

	return local_cnt;
}





int colorCalib(U16* surf, int ycenter, U8* cal_h, U8* cal_s, U8* cal_v) {
	int n;
	int hue_sum = 0;
	int sat_sum = 0;
	int val_sum = 0;
	int cnt = 0;
	float hue_avg = 0;
	float sat_avg = 0;
	float val_avg = 0;

	for (n = 0; n < pl; n++) {
		int x = n%iw;
		int y = (int)n / iw;

		U8 hue = (U8)((0xff00 & surf[n]) >> 8);
		U8 sat = (U8)((0x00f0 & surf[n]) >> 4);
		U8 val = (U8)(0x000f & surf[n]);

		if (x > 75 && x < 105 && y < ycenter + 10 && y > ycenter - 10) {
			//	printf("data // hsv : %8x // %4x / %4x / %4x \n", surf[n], hue, sat, val);
			hue_sum += hue;
			sat_sum += sat;
			val_sum += val;
			cnt++;
		}
	}

	hue_avg = hue_sum / cnt;
	sat_avg = sat_sum / cnt;
	val_avg = val_sum / cnt;

	//	printf("ha / sa / va : %5.2f / %5.2f / %5.2f \n", hue_avg, sat_avg, val_avg);

	*cal_h = (U8)hue_avg;
	*cal_s = (U8)sat_avg;
	*cal_v = (U8)val_avg;

	return 0;
}
// °ü½É »ö»óÀ¸·Î ÀÌÁøÈ­ ½ÃÅ´.
// outsurf : Ãâ·Â plane
// hsvsurf : ÀÔ·Â hsv plane
// usr_color : °ü½É »ö»ó
// return value - °ü½É »ö»óÀ¸·Î ÀÌÁøÈ­µÈ ÇÈ¼¿ °¹¼ö.
int colorRegion(U16* outsurf, U16* hsvsurf, COLOR usr_color) {
	int n, cnt;
	COLOR pcolor;

	cnt = 0;
	for (n = 0; n < pl; n++) {
		U8 hue = (U8)((0xff00 & hsvsurf[n]) >> 8);
		U8 sat = (U8)((0x00f0 & hsvsurf[n]) >> 4);
		U8 val = (U8)(0x000f & hsvsurf[n]);

		pcolor = colorpeek(hue, sat, val);

		if (pcolor == usr_color) {
			outsurf[n] = 0xffff;
			cnt++;
		}
		else
			outsurf[n] = 0x0000;
	}

	return cnt;
}
//
int pure_red(U16* outsurf, U16* hsvsurf) {
	int n, cnt;
	U8 hue;
	U8 val;
	cnt = 0;
	for (n = 0; n < pl; n++) {
		hue = (U8)((0xff00 & hsvsurf[n]) >> 8);
		val = (U8)(0x000f & hsvsurf[n]);

		if ((hue < 160 && hue >= 45) && (val > 3)) {
			outsurf[n] = 0xffff;
			cnt++;
		}
		else
			outsurf[n] = 0x0000;	
	}

	return cnt;
}
// °æ±âÀå °æ°è °¡·Î¼± È¹µæ
// peek_bndy ÇÔ¼ö¿¡ ¾²ÀÓ. °¡·Î¼± È¹µæÀÌ¶ó´Â Á¡¿¡¼­ chop_hrizon ÇÔ¼ö¿Í µ¿ÀÏ.
int chop_boundary(U16* htda, int htda_num, int* st, int* score) {
	int n, m;

	/*
	idx group
	0		: skewed-right lines (\)
	1		: flat lines (-)
	2		: skewed-left lines (/)
	*/

	/*
	return -1 : ¾î¶² ÆÐÅÏµµ °ËÃâµÇÁö ¾Ê¾ÒÀ» ¶§. htda°¡ ¾ø´Ù¸é.
	return 0  : ÆÐÅÏ °ËÃâµÊ.

	*st : idx groupÀÇ ³Ñ¹ö Àü´Þ.
	*score : idx 1 ÀÏ ¶§, Æò±Õ rho °ª. idx 1ÀÏ ¶§¸¸ ÀÇ¹ÌÀÖÀ½.

	*/

	int vot[3] = { 0 };
	int idx[3];

	for (n = 0; n < 3; n++)
		idx[n] = n;

	int t_vot;
	int t_idx;

	int sum;
	int r_tSum, l_tSum;

#ifdef show_htda_dat
	printf("=================================\n");
	printf("	show_htda_dat dbg message\n");
	printf("=================================\n");
#endif

	sum = 0;
	r_tSum = 0;
	l_tSum = 0;
	for (n = 0; n < htda_num; n++) {
		char rho = ((char)(0x00ff & htda[n])) * 2;
		U8 theta = (U8)((0xff00 & htda[n]) >> 8);

		if (rho < 0)
			rho = -rho;

#ifdef show_htda_dat
		printf("[%3d] rho/theta : %5d / %5d\n", n, rho, theta);
#endif

		if (theta >= 125 && theta < 195) {//(theta >= 125 && theta < 188) 
			l_tSum += theta;
			vot[0]++;
		}
		else if (theta >= 6 && theta < 80) {
			r_tSum += theta;
			vot[2]++;
		}
		else if (theta >= 195 || theta < 6) {//(theta >= 188 || theta < 6)
			sum += rho;
			vot[1]++;
		}
	}

#ifdef show_hzt_idx2
	printf("=================================\n");
	printf("	show_hzt_idx dbg message\n");
	printf("=================================\n");
	printf("idx |");
	for (n = 0; n < hzt_stage; n++)
		printf(" [%3d] |", idx[n]);
	printf("\n");
	printf("vot |");
	for (n = 0; n < hzt_stage; n++)
		printf(" %4d  |", vot[n]);
	printf("\n");
	printf("=================================\n");
#endif

	/* sort */

	for (n = (hzt_stage - 1); n > 0; n--) {
		for (m = 0; m < n; m++) {
			if (vot[m] < vot[m + 1]) {
				t_vot = vot[m];
				t_idx = idx[m];

				vot[m] = vot[m + 1];
				idx[m] = idx[m + 1];

				vot[m + 1] = t_vot;
				idx[m + 1] = t_idx;
			}
		}
	}

	if (vot[0] == 0)
		return -1;	// no pattern

	// calc score

	if (idx[0] == 0) {
		*score = (int)l_tSum / vot[0];
	}
	else if (idx[0] == 1) {
		*score = (int)sum / vot[0];
	}
	else if (idx[0] == 2) {
		*score = (int)r_tSum / vot[0];
	}
	else
		return -1;

	// output pattern

	*st = idx[0];

#ifdef show_htda_dat
	printf("=================================\n");
#endif

#ifdef show_hzt_idx
	printf("=================================\n");
	printf("	show_hzt_idx dbg message\n");
	printf("=================================\n");
	printf("no. |");
	for (n = 0; n < hzt_stage; n++)
		printf(" [%3d] |", n);
	printf("\n");
	printf("idx |");
	for (n = 0; n < hzt_stage; n++)
		printf(" [%3d] |", idx[n]);
	printf("\n");
	printf("vot |");
	for (n = 0; n < hzt_stage; n++)
		printf(" %4d  |", vot[n]);
	printf("\n");
	printf("=================================\n");
#endif

	return 0;
}
// generalÇÑ °¡·Î¼± È¹µæ
// line_target, peek_bdgland, peek_trap µîÀÇ ÇÔ¼ö¿¡ ¾²ÀÓ
// ÃÖÀûÈ­ÇÒ ¼ö ÀÖ´Â Àå¾Ö¹° ÆÄ¶ó¹ÌÅÍ :
//	bridge_up, bridge_down, trap_pre, trap_hole, default_obs


//int chop_vert(U16* htda, int htda_num, int* grp, int* go_rho1, int* go_rho2, vObs usr_obstacle)
int chop_hrizon(U16* htda, int htda_num, int* grp, int* go_rho, vOpt usr_option, vObs usr_obstacle) {
	int n, m, max, max_grp;
	// vot[0] : ¿ÞÂÊÀ¸·Î ±â¿î Á÷¼± (\)
	// vot[1] : ¹Ù¸¥ Á÷¼±
	// vot[2] : ¿À¸¥ÂÊÀ¸·Î ±â¿î Á÷¼± (/)
	int vot[3] = { 0 };
	// a_flat : ¹Ù¸¥ Á÷¼± ÁýÇÕ ¹è¿­
	U16* a_flat;
	// a_ptr : ¹Ù¸¥ Á÷¼± ÁýÇÕ À¯È¿ ¹üÀ§ °ª
	int a_ptr = 0;

	if (htda_num < 1) {
		printf("no htda datas\n");
		return -1;
	}

	a_flat = (U16*)malloc(sizeof(U16)*htda_num);
	memset(a_flat, 0, sizeof(U16)*htda_num);

	// ¹Ù¸¥ Á÷¼± ºÐ·ù
	for (n = 0; n < htda_num; n++) {
		// À¯È¿ÇÑ °¡·Î¼± °Ë»ç
		short rho = (short)(((char)(0x00ff & htda[n])) * 2);
		U8 theta = (U8)((0xff00 & htda[n]) >> 8);
		// Àý´ë°ª ÃëÇÔ
		if (rho < 0)
			rho = -rho;

		// Àå¾Ö¹° Á¾·ù¿¡ µû¶ó ÃÖÀûÈ­
		if (usr_obstacle == bridge_up) {
			;
		}
		else if (usr_obstacle == bridge_down) {
			;
		}
		else if (usr_obstacle == trap_pre) {
			if (theta >= 150 && theta < 185) {
				vot[0]++;
			}
			else if (theta >= 15 && theta < 55) {
				vot[2]++;

			}
			else if (theta >= 185 || theta < 15) {
				if (rho <= 110) {
					a_flat[a_ptr++] = htda[n];
					vot[1]++;
				}
			}
		}
		else if (usr_obstacle == trap_hole) {
			if (theta >= 150 && theta < 196) {
				vot[0]++;
			}
			else if (theta >= 4 && theta < 55) {
				vot[2]++;

			}
			else if (theta >= 196 || theta < 4) {
				if (rho <= 110) {
					a_flat[a_ptr++] = htda[n];
					vot[1]++;
				}
			}
		}
		else if (usr_obstacle == default_obs) {
			if (theta >= 125 && theta < 195) {
				vot[0]++;
			}
			else if (theta >= 5 && theta < 80) {
				vot[2]++;

			}
			else if (theta >= 195 || theta < 5) {
				if (rho <= 110) {
					a_flat[a_ptr++] = htda[n];
					vot[1]++;
				}
			}
		}
		else {
			printf("undefined obstacle \n");
			free(a_flat);
			return -1;
		}
	}

	// Á÷¼± ºÐÆ÷
	max = vot[0];
	max_grp = 0;

	if (max < vot[2]) {
		max = vot[2];
		max_grp = 2;
	}
	
	if (max < vot[1]) {
		max = vot[1];
		max_grp = 1;
	}

	// max°¡ 0ÀÓ. -> Á÷¼± Á¤º¸´Â Á¸ÀçÇÏÁö¸¸ À¯È¿ÇÑ Á¤º¸´Â ¾øÀ½À» ÀÇ¹Ì
	if (max == 0) {
		free(a_flat);
		return -1;
	}

	// 1Â÷ ±×·ì °áÁ¤ : vot¿¡ µû¶ó
	if (max_grp == 0)
		*grp = 0;
	else if (max_grp == 2)
		*grp = 2;
	else {
		// 2Â÷ ±×·ì °áÁ¤
		// flatÇÏ´Ù. flatÇÑ °¡·Î¼±ÀÌ ¿©·¯°³ Á¸Àç ÇÒ ¼ö ÀÖÀ¸¹Ç·Î, rho¿¡ µû¶ó ±×·ìÀ» Áþ´Â´Ù.
		// ±×·ìÀº 4°³ ±îÁö.
		int grho[4] = { 0 };
		int gamt[4] = { 0 };
		int grp_n = 0;
		int select = 0;
		int gptr, gcnt, flg;
		U16 tmp;
		short rho1, rho2;
		float gavg, gsum, delta;

		// flatÇÑ µ¥ÀÌÅÍµéÀ» rho¿¡ ´ëÇÏ¿© sortÇÑ´Ù.
		// option¿¡ µû¶ó ¿À¸§Â÷¼ø, ³»¸²Â÷¼øÀ¸·Î sort
		// Ã¹ Á¶°Ç¹®Àº È¤½Ã¶óµµ...
		if (a_ptr) {
			for (n = (a_ptr - 1); n > 0; n--) {
				for (m = 0; m < n; m++) {
					// flat µ¥ÀÌÅÍ Áß, rho ÃßÃâ
					rho1 = (short)(((char)(0x00ff & a_flat[m])) * 2);
					rho2 = (short)(((char)(0x00ff & a_flat[m + 1])) * 2);
					// ¿É¼Ç Àû¿ë
					switch (usr_option) {
					case top_porking:
						if (rho1 > rho2) {
							tmp = a_flat[m];
							a_flat[m] = a_flat[m + 1];
							a_flat[m + 1] = tmp;
						}
						break;
					case bot_porking:
						if (rho1 < rho2) {
							tmp = a_flat[m];
							a_flat[m] = a_flat[m + 1];
							a_flat[m + 1] = tmp;
						}
					default:
						// undefined
						break;
					}
				}
			}
		}
		else {
			printf("unexpected \n");
			free(a_flat);
			return -1;
		}

		// ±×·ìÁþ±â¸¦ ½ÃÀÛÇÑ´Ù.
		gptr = 0;
		// ÃÖ´ë 4°³ÀÇ ±×·ìÁþ±â
		for (n = 0; n < 4; n++) {
			// ´ÙÀ½ µ¥ÀÌÅÍ°¡ ¾øÀ¸¹Ç·Î, ±×·ìÁþ±â¸¦ ³¡³¿
			if (gptr == a_ptr)
				break;
			// gptr Á÷¼± µ¥ÀÌÅÍ rho ÃßÃâ
			rho1 = (short)(((char)(0x00ff & a_flat[gptr])) * 2);
			// ÃÊ±â ±×·ìÀº ÀÚ±â ÀÚ½ÅÀ» sum°ú avg·Î °¡Áø´Ù.
			gsum = (float)rho1;
			gavg = (float)rho1;
			gcnt = 1;
			while (1) {
				gptr++;
				// ´ÙÀ½ µ¥ÀÌÅÍ°¡ ¾øÀ¸¹Ç·Î, ±×·ìÁþ±â¸¦ ³¡³¿
				if (gptr == a_ptr) {
					grho[n] = (int)gavg;
					gamt[n] = gcnt;
					grp_n++;
					break;
				}
				
				// ±×·ì Æò±Õ(gavg)°ú ´ÙÀ½ °ªÀÇ Â÷(delta)¸¦ Á¶»çÇÑ´Ù.
				rho1 = (short)(((char)(0x00ff & a_flat[gptr])) * 2);
				delta = (gavg - (float)rho1);

				// delta°¡ ÀûÀýÇÏ¸é ±×·ì¿¡ Æ÷ÇÔÇÑ´Ù.
				if (delta < 10 && delta > -10) {
					gsum += (float)rho1;
					gcnt++;
					// ±×·ì Æò±Õ °»½Å
					gavg = gsum / gcnt;
				}
				// delta°¡ ±×·ì¿¡ Æ÷ÇÔµÇÁö ¾ÊÀ» ¸¸Å­ Å©¸é, ±×·ìÁþ±â¸¦ ³¡³½´Ù.
				else {
					grho[n] = (int)gavg;
					gamt[n] = gcnt;
					grp_n++;
					break;
				}
			}
		}

		// ±×·ìÁþ±â°¡ ³¡³ª°í, ±×·ì¿¡ Æ÷ÇÔµÈ µ¥ÀÌÅÍ·®À» Á¶»çÇÑ´Ù.
		// ±×·ìÅ©±â°¡ ÃæºÐÇÏÁö ¾Æ´ÏÇÏ¸é, ³ëÀÌÁî·Î °£ÁÖÇÏ¿© ±×·ìÀ» Å»¶ô½ÃÅ°°í ´ÙÀ½ ±×·ìÀ» ¼±ÅÃÇÑ´Ù.
		flg = 0;
		for (n = 0; n < grp_n; n++) {
			if (gamt[n] > 2) {
				select = n;
				flg = 1;
				break;
			}
		}

		// ¸¸¾à, ¸ðµç ±×·ìÅ©±â°¡ ÃæºÐÇÏÁö ¾Æ´ÏÇÏ¸é, °¡Àå Å« ±×·ìÀ» ¼±ÅÃÇÑ´Ù.

		if (!flg) {
			max = gamt[0];
			select = 0;
			for (n = 0; n < grp_n; n++) {
				if (max < gamt[n]) {
					max = gamt[n];
					select = n;
				}
			}
		}

		*go_rho = grho[select];
		*grp = 1;
	}

	free(a_flat);

	return 0;
}
// generalÇÑ ¼¼·Î¼± È¹µæ
// ÃÖÀûÈ­ÇÒ ¼ö ÀÖ´Â Àå¾Ö¹° ÆÄ¶ó¹ÌÅÍ
//	bridge_on, trap_hole, default_obs
// majorÇÑ µÎ °³ÀÇ Á÷¼± Á¤º¸¸¦ È¹µæÇÒ ¼ö ÀÖÀ½.
int chop_vert(U16* htda, int htda_num, int* grp, int* go_rho1, int* go_rho2, vObs usr_obstacle) {
	int n, m, max, max_grp;
	// vot[0] : ¿ÞÂÊ ±â¿î ¼±
	// vot[1] : ¹Ù¸¥ ¼±
	// vot[2] : ¿À¸¥ÂÊ ±â¿î¼±
	int vot[3] = { 0 };

	// a_flat : ¹Ù¸¥ Á÷¼± ÁýÇÕ ¹è¿­
	U16* a_flat;
	// a_ptr : ¹Ù¸¥ Á÷¼± ÁýÇÕ À¯È¿ ¹üÀ§ °ª
	int a_ptr = 0;

	if (htda_num < 1) {
		printf("no htda datas\n");
		return -1;
	}

	a_flat = (U16*)malloc(sizeof(U16)*htda_num);
	memset(a_flat, 0, sizeof(U16)*htda_num);

	// ¹Ù¸¥ ¼¼·Î Á÷¼± ºÐ·ù
	for (n = 0; n < htda_num; n++) {
		// Á÷¼± Á¤º¸¿¡¼­ rho¿Í theta¸¦ ÃßÃâ
		U8 theta = (U8)((0xff00 & htda[n]) >> 8);

		// Àå¾Ö¹° Á¾·ù¿¡ µû¶ó À¯È¿ Á÷¼± ºÐ·ù ÃÖÀûÈ­
		if (usr_obstacle == bridge_on) {
			if (theta > 111 && theta < 150) {
				vot[0]++;
			}
			else if (theta > 50 && theta < 95) {
				vot[2]++;
			}
			else if (theta >= 95 && theta <= 111) {
				a_flat[a_ptr++] = htda[n];
				vot[1]++;
			}
		}
		else if (usr_obstacle == trap_hole) {
			if (theta > 120 && theta < 150) {
				vot[0]++;
			}
			else if (theta > 50 && theta < 80) {
				vot[2]++;
			}
			else if (theta >= 80 && theta <= 120) {
				a_flat[a_ptr++] = htda[n];
				vot[1]++;
			}
		}
		else if (usr_obstacle == default_obs) {
			if (theta > 115 && theta < 150) {
				vot[0]++;
			}
			else if (theta > 50 && theta < 90) {
				vot[2]++;
			}
			else if (theta >= 90 && theta <= 115) {
				a_flat[a_ptr++] = htda[n];
				vot[1]++;
			}
		}
		else {
			printf("undefined obstacle \n");
			free(a_flat);
			return -1;
		}
	}

	max = vot[0];
	max_grp = 0;

	if (max < vot[2]) {
		max = vot[2];
		max_grp = 2;
	}

	if (max < vot[1]) {
		max = vot[1];
		max_grp = 1;
	}

	// max°¡ 0ÀÓ. -> Á÷¼± Á¤º¸´Â Á¸ÀçÇÏÁö¸¸ À¯È¿ÇÑ Á¤º¸´Â ¾øÀ½À» ÀÇ¹Ì
	if (max == 0) {
		free(a_flat);
		return -1;
	}

	// 1Â÷ ±×·ì °áÁ¤ : vot¿¡ µû¶ó
	if (max_grp == 0)
		*grp = 0;
	else if (max_grp == 2)
		*grp = 2;
	else {
		// 2Â÷ ±×·ì °áÁ¤
		// flatÇÏ´Ù. flatÇÑ °¡·Î¼±ÀÌ ¿©·¯°³ Á¸Àç ÇÒ ¼ö ÀÖÀ¸¹Ç·Î, rho¿¡ µû¶ó ±×·ìÀ» Áþ´Â´Ù.
		// ±×·ìÀº 4°³ ±îÁö.
		int grho[4] = { 0 };
		int gamt[4] = { 0 };
		int grp_n = 0;

		int gptr, gcnt;
		U16 tmp; int gtmp;
		short rho1, rho2;
		float gavg, gsum, delta;

		// flatÇÑ µ¥ÀÌÅÍµéÀ» rho¿¡ ´ëÇÏ¿© sortÇÑ´Ù.
		// option¿¡ µû¶ó ¿À¸§Â÷¼ø, ³»¸²Â÷¼øÀ¸·Î sort
		if (a_ptr) {
			for (n = (a_ptr - 1); n > 0; n--) {
				for (m = 0; m < n; m++) {
					// flat µ¥ÀÌÅÍ Áß, rho ÃßÃâ
					rho1 = (short)(((char)(0x00ff & a_flat[m])) * 2);
					rho2 = (short)(((char)(0x00ff & a_flat[m + 1])) * 2);
					if (rho1 > rho2) {
						tmp = a_flat[m];
						a_flat[m] = a_flat[m + 1];
						a_flat[m + 1] = tmp;
					}
				}
			}
		}
		// Ã¹ Á¶°Ç¹®Àº È¤½Ã¶óµµ ¸ð¸¦ ¿¹¿ÜÃ³¸®.
		else {
			printf("unexpected \n");
			free(a_flat);
			return -1;
		}

		// ±×·ìÁþ±â¸¦ ½ÃÀÛÇÑ´Ù.
		gptr = 0;
		// ÃÖ´ë 4°³ÀÇ ±×·ìÁþ±â
		for (n = 0; n < 4; n++) {
			// ´ÙÀ½ µ¥ÀÌÅÍ°¡ ¾øÀ¸¹Ç·Î, ±×·ìÁþ±â¸¦ ³¡³¿
			if (gptr == a_ptr)
				break;
			// gptr Á÷¼± µ¥ÀÌÅÍ rho ÃßÃâ
			rho1 = (short)(((char)(0x00ff & a_flat[gptr])) * 2);
			// ÃÊ±â ±×·ìÀº ÀÚ±â ÀÚ½ÅÀ» sum°ú avg·Î °¡Áø´Ù.
			gsum = (float)rho1;
			gavg = (float)rho1;
			gcnt = 1;
			while (1) {
				gptr++;
				// ´ÙÀ½ µ¥ÀÌÅÍ°¡ ¾øÀ¸¹Ç·Î, ±×·ìÁþ±â¸¦ ³¡³¿
				if (gptr == a_ptr) {
					grho[n] = (int)gavg;
					gamt[n] = gcnt;
					grp_n++;
					break;
				}

				// ±×·ì Æò±Õ(gavg)°ú ´ÙÀ½ °ªÀÇ Â÷(delta)¸¦ Á¶»çÇÑ´Ù.
				rho1 = (short)(((char)(0x00ff & a_flat[gptr])) * 2);
				delta = (gavg - (float)rho1);

				// delta°¡ ÀûÀýÇÏ¸é ±×·ì¿¡ Æ÷ÇÔÇÑ´Ù.
				if (delta < 10 && delta > -10) {
					gsum += (float)rho1;
					gcnt++;
					// ±×·ì Æò±Õ °»½Å
					gavg = gsum / gcnt;
				}
				// delta°¡ ±×·ì¿¡ Æ÷ÇÔµÇÁö ¾ÊÀ» ¸¸Å­ Å©¸é, ±×·ìÁþ±â¸¦ ³¡³½´Ù.
				else {
					grho[n] = (int)gavg;
					gamt[n] = gcnt;
					grp_n++;
					break;
				}
			}
		}

		// ±×·ìÁþ±â°¡ ³¡³ª°í, °¡´ÉÇÑ majorÇÑ µÎ °³ÀÇ ±×·ìÀ» ¼±ÅÃÇØ¾ß ÇÑ´Ù.

		// °¡Àå Å« Å©±âÀÇ µÎ ±×·ìÀ» ¼±ÅÃÇÏ±â À§ÇØ¼­, °ËÃâµÈ ±×·ì¿¡ ´ëÇÏ¿© sortingÀ» ¼öÇàÇÑ´Ù.
		for (n = (grp_n - 1); n > 0; n--) {
			for (m = 0; m < n; m++) {
				// ±×·ì Å©±â ºñ±³ ³»¸²Â÷¼ø Á¤·Ä
				if (gamt[m] < gamt[m+1]) {
					gtmp = gamt[m];
					gamt[m] = gamt[m + 1];
					gamt[m + 1] = gtmp;

					gtmp = grho[m];
					grho[m] = grho[m + 1];
					grho[m + 1] = gtmp;
				}
			}
		}

		// ±×·ìÀÌ ÇÏ³ª »ÓÀÎ °æ¿ì, ±× ±×·ìÀ» ¼±ÅÃÇÔ.
		// -1234, n/a Á¸ÀçÇÏÁö ¾ÊÀ½À» ³ªÅ¸³¿.
		if (grp_n < 2) {
			*go_rho1 = grho[0];
			*go_rho2 = -1234;
		}
		else {
			// sortingÇÑ ÀÌÈÄÀÌ¹Ç·Î, grho[0]¿Í grho[1]°¡ ±×·ì Å©±â ¼øÀ¸·Î Ã¹¹øÂ°, µÎ¹øÂ°ÀÌ´Ù.
			*go_rho1 = grho[0];
			*go_rho2 = grho[1];
		}

		*grp = 1;
	}

	free(a_flat);

	return 0;
}
// ÀÌÀü version
int vert_map_for_bdgpeek(U16* htda, int htda_num, int* st, int* score_left, int*score_right, vObs obstacle) {
	int n;
	int cmp_max, cmp_max_idx;
	/*
	idx
	0: skewed right (\)
	1: skewed left (/)
	2: left
	3: right
	*/

	// first check line set
	int cnt[4] = { 0 };
	int cmp[3] = { 0 };
	int lft_sum = 0;
	int rgt_sum = 0;

#ifdef show_htda_dat
	printf("=================================\n");
	printf("	show_htda_dat dbg message\n");
	printf("=================================\n");
#endif
	printf("htda num : %5d \n", htda_num);
	for (n = 0; n < htda_num; n++) {
		// htda ¹Þ±â
		short rho = (short)(((char)(0x00ff & htda[n])) * 2);
		U8 theta = (U8)((0xff00 & htda[n]) >> 8);

#ifdef show_htda_dat
		printf("[%3d] rho/theta : %5d / %5d\n", n, rho, theta);
#endif
		// votingÀ» ½ÃÀÛÇÕ´Ï´Ù.
		// ¼öÁ÷¼± Á¶°ÇÀÔ´Ï´Ù.
		if (obstacle == default_obs) {
			if (theta >= 95 && theta <= 111) {
				if (rho < 0) {
					lft_sum += (int)rho;
					cnt[2]++;
				}
				else {
					rgt_sum += (int)rho;
					cnt[3]++;
				}
				// ÁÂ¿ì·Î ±â¿ï¾îÁø ¼±ÀÇ Á¶°ÇÀÔ´Ï´Ù.
			}
			else if (theta > 111 && theta < 150) {
				cnt[0]++;
			}
			else if (theta > 50 && theta < 95) {
				cnt[1]++;
			}
		}
		else if (obstacle == trap_hole) {
			if (theta >= 95 && theta <= 111) {
				if (rho < 0) {
					lft_sum += (int)rho;
					cnt[2]++;
				}
				else {
					rgt_sum += (int)rho;
					cnt[3]++;
				}
			}
			// ÁÂ¿ì·Î ±â¿ï¾îÁø ¼±ÀÇ Á¶°ÇÀÔ´Ï´Ù.
			else if (theta > 111 && theta < 150) {
				cnt[0]++;
			}
			else if (theta > 50 && theta < 95) {
				cnt[1]++;
			}
		}
	}

#ifdef show_htda_dat
	printf("=================================\n");
#endif

	cmp[0] = cnt[0];
	cmp[1] = cnt[2] + cnt[3];
	cmp[2] = cnt[1];

	cmp_max = 0;
	cmp_max_idx = 0;

	for (n = 0; n < 3; n++)
		if (cmp[n] > cmp_max) {
			cmp_max = cmp[n];
			cmp_max_idx = n;
		}

	if (cmp_max == 0)
		return -1; // no pattern

	switch (cmp_max_idx) {
	case 0:
		*st = 0;
		break;
	case 1:
		if (cnt[2])
			*score_left = lft_sum / cnt[2];
		else
			*score_left = -1234;
		if (cnt[3])
			*score_right = rgt_sum / cnt[3];
		else
			*score_right = -1234;
		*st = 1;
		break;
	case 2:
		*st = 2;
		break;
	default:
		printf("undefined cmp idx \n");
		return -1;
	}

	return 0;
}

// application functions
// define¿¡ µû¶ó samp_frame¸¸Å­ ÇÁ·¹ÀÓÀ» Á¶»ç. ±âº» 21ÇÁ·¹ÀÓ

// °æ±âÀå °æ°è¸¦ È®ÀÎÇÏ´Âµ¥ ¾´´Ù.
// ¡Ø ¸ð¼Ç drive ÇÔ¼ö : mot_arrangeMid
int peek_bndy(FILE* log) {
	int n, m, htda_num;
	int rho, pat, flg, tmp, state = 0;

	// ÁÖ¿ä ÆÐÅÏ ºÐ·ù¿¡ ¾²ÀÌ´Â º¯¼ö, °¡Àå majorÇÑ ÆÐÅÏÀ» °¡·Á³¾ ¶§ »ç¿ëÇÔ.
	int max, max_pat;
	// cnt[4] : pattern ±×·ì ¹è¿­. smp_frame¸¸Å­ ÇÁ·¹ÀÓ Á¶»ç¸¦ ÇßÀ» ¶§, ³ª¿Â ÆÐÅÏÀÇ ¼ö¸¦ Á¶»çÇÑ´Ù.
	//	cnt[0] : ¾î¶² ÆÐÅÏµµ ¾øÀ½. ¿µ»ó¿¡ ¾î¶°ÇÑ À¯È¿ Á÷¼± Á¤º¸µµ ¾øÀ» ¶§.
	//	cnt[1], cnt[3] : ±â¿ï¾îÁø Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	//	cnt[2] : ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	int cnt[4] = { 0 };
	// arr : °¢ frameÀÇ Á÷¼±Á¤º¸ Á¶»ç °á°ú¸¦ ÀúÀåÇÏ´Â ¹è¿­. 
	//		arr[x]Àº À¯È¿ÇÑ frameÀÇ x¹øÂ° Á÷¼± Á¤º¸ÀÌ´Ù.
	int arr[smp_frame] = { 0 };
	int r_arr[smp_frame] = { 0 };
	int l_arr[smp_frame] = { 0 };
	// n_arr : À¯È¿ÇÑ frameÀÇ °¹¼ö¸¦ indicateÇÑ´Ù.
	int n_arr = 0;
	int r_n_arr = 0;
	int l_n_arr = 0;
	// À¯È¿ÇÑ °á°úµé Áß, Áß¾Ó°ªÀ» ÃëÇÒ ¶§, ÀÌ º¯¼ö¿¡ °á°ú¸¦ ÀúÀåÇÔ.
	int mid_rho = -65538;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	// smp_frame¸¸Å­ ÇÁ·¹ÀÓÀ» Á¶»ç.
	for (n = 0; n < smp_frame; n++) {

		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);

		flg = chop_boundary(htda, htda_num, &pat, &rho);
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (pat) {
			case 0:
				l_arr[l_n_arr++] = rho;
				cnt[1]++;
				break;
			case 1:
				// ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ» ¶§, À¯È¿ Á÷¼± Á¤º¸¸¦ ±â·ÏÇÑ´Ù.
				arr[n_arr++] = rho;
				cnt[2]++;
				break;
			case 2:
				r_arr[r_n_arr++] = rho;
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	// cnt ¹è¿­À» Á¶»çÇÏ¿© °¡Àå ÁÖ¿äÇÑ ÆÐÅÏÀº ¾î¶² °ÍÀÎ°¡ °áÁ¤
	max_pat = 0;
	max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > max) {
			max_pat = n;
			max = cnt[n];
		}

	// ÁÖ¿äÇÑ ÆÐÅÏÀ» °á°ú·Î ÃëÇÔ.
	if (max_pat == 0) {
		// ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
		state = -1;
	}
	else if (max_pat == 1) {
		// skewed left
		// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
		for (n = (l_n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (l_arr[m] < l_arr[m + 1]) {
					tmp = l_arr[m];
					l_arr[m] = l_arr[m + 1];
					l_arr[m + 1] = tmp;
				}
		// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
		mid_rho = l_arr[(int)(l_n_arr / 2)];
		state = 0;
		// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
		if (mid_rho < 155)
			state = 0;
		else
			state = 7; // Á¶±Ý ¿ìÇâ¿ì
	}
	else if (max_pat == 2) {
		// flat line
		// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
		for (n = (n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (arr[m] < arr[m + 1]) {
					tmp = arr[m];
					arr[m] = arr[m + 1];
					arr[m + 1] = tmp;
				}
		// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
		mid_rho = arr[(int)(n_arr / 2)];
		// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
		if (mid_rho < 38)
			state = 2;
		else if (mid_rho < 50)
			state = 3;
		else if (mid_rho < 58)	//58 -> 56 -> 58
			state = 4;
		else if (mid_rho < 68)
			state = 5;
		else
			state = 6;
	}
	else if (max_pat == 3) {
		// skewed right
		// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
		for (n = (r_n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (r_arr[m] < r_arr[m + 1]) {
					tmp = r_arr[m];
					r_arr[m] = r_arr[m + 1];
					r_arr[m + 1] = tmp;
				}
		// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
		mid_rho = r_arr[(int)(r_n_arr / 2)];
		// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
		if (mid_rho > 30)
			state = 1;
		else
			state = 8; // Á¶±Ý ÁÂÇâÁÂ
	}


	free(htda);
	free(fvda);

	if (log) {
		fprintf(log, "[LOG] peek_bndy \n>> result table\n");
		fprintf(log, "pat |");
		for (n = 0; n < 4; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "vot |");
		for (n = 0; n < 4; n++)
			fprintf(log, " %4d  |", cnt[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "arr |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "rho |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " %4d  |", arr[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "rho of the detected line : %5d \n", mid_rho);
		fprintf(log, "=================================\n");
	}

	// return °ªÀ¸·Î, motion driver ÇÔ¼ö ³»ºÎ¿¡¼­ Çö Á÷¼± »óÅÂ¸¦ indicateÇÏ´Â °ª
	// state [-1] : pattern not appear
	// state [ 0] : skewed left (\) 
	// state [ 1] : skewed right (/)
	// state [ 2] : line too far
	// state [ 3] : line far
	// state [ 4] : line fitted
	// state [ 5] : line close
	// state [ 6] : line too close

	return state;
}
// È­¸é¿¡¼­ °¡·Î¼±À» Å¸°ÙÆÃÇÏ´Âµ¥ ¾´´Ù.
// ÀÌ ÇÔ¼öÀÇ °á°ú´Â È­¸é¿¡¼­ °¡·Î¼±ÀÌ ¾îµð¿¡ À§Ä¡ÇØ ÀÖ´ÂÁö ¾Ë·ÁÁÖ´Â °ÍÀÌ´Ù.
// tar_rho´Â °¡·Î¼± °Å¸®¸¦ Å¸°ÙÆÃÇÏ´Â °ÍÀÌ´Ù. tar_rho¿¡ ¸ÂÃç fitted °á°ú¸¦ Ãâ·Â.
// behavior´Â ´Ü¼øÈ÷ ¿µ»ó¿¡¼­ ¼±ÀÌ °ËÃâµÇ´Â °æ¿ì¸¦ Ã£´Â °ÍÀÎÁö tar_rho¿¡ ºñ±³ÇÏ¿© °á°ú¸¦ ¾òÀ¸·Á´Â °ÍÀÎÁö ¼±ÅÃÇÏ´Â ¿É¼ÇÀÌ´Ù.
//	peeking : ¿µ»ó¿¡¼­ ¼±ÀÌ ÀÖ´ÂÁö ¾ø´ÂÁö ÆÇ´ÜÇÑ´Ù. ÀÌ °æ¿ì, tar_rho ÀÔ·ÂÀº ÀÇ¹Ì¾ø´Ù.
//	fitting : ¿µ»ó¿¡¼­ ¼±À» tar_rho¿¡ ºñ±³ÇÏ¿© °á°ú¸¦ Ãâ·ÂÇÑ´Ù.
// tar_optionÀº ¿µ»ó¿¡ Á¸ÀçÇÏ´Â °¡·Î¼±µé Áß, °¡Àå À§ÂÊ ¼±°ú °¡Àå ¾Æ·¡ÂÊ ¼±Áß ¾î´À ¼±À» tar_rho¿¡ ºñ±³ÇÒ °ÍÀÎÁö ¼±ÅÃÇÏ´Â ¿É¼ÇÀÌ´Ù.
//	top_porking
//	bot_porking
// usr_obs´Â Àå¾Ö¹°¿¡ ´ëÇÏ¿© ÃÖÀûÈ­ÇÏ´Â ¿É¼ÇÀÌ´Ù. µû·Î ÁöÁ¤ÇÏ°í ½ÍÁö ¾ÊÀ¸¸é, default_obs¸¦ ÀÎ¼ö·Î ³Ö´Â´Ù.
//	»ç¿ë °¡´ÉÇÑ ÀÎ¼ö : bridge_up, bridge_down, trap_pre, trap_hole, default_obs
// ÀÌ ÇÔ¼ö´Â ÁÖ·Î, Àü¹æ Àå¾Ö¹° Á¢±Ù¿¡ ÀÌ¿ëÇÑ´Ù.
// ¡Ø ¸ð¼Ç drive ÇÔ¼ö : mot_gotoReadyLine_bdg, mot_gotoReadyLine_red, mot_gotoReadyLine_trap

int peek_bndy_2(FILE* log) {
	int n, m, htda_num;
	int rho, pat, flg, tmp, state = 0;

	// ÁÖ¿ä ÆÐÅÏ ºÐ·ù¿¡ ¾²ÀÌ´Â º¯¼ö, °¡Àå majorÇÑ ÆÐÅÏÀ» °¡·Á³¾ ¶§ »ç¿ëÇÔ.
	int max, max_pat;
	// cnt[4] : pattern ±×·ì ¹è¿­. smp_frame¸¸Å­ ÇÁ·¹ÀÓ Á¶»ç¸¦ ÇßÀ» ¶§, ³ª¿Â ÆÐÅÏÀÇ ¼ö¸¦ Á¶»çÇÑ´Ù.
	//	cnt[0] : ¾î¶² ÆÐÅÏµµ ¾øÀ½. ¿µ»ó¿¡ ¾î¶°ÇÑ À¯È¿ Á÷¼± Á¤º¸µµ ¾øÀ» ¶§.
	//	cnt[1], cnt[3] : ±â¿ï¾îÁø Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	//	cnt[2] : ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	int cnt[4] = { 0 };
	// arr : °¢ frameÀÇ Á÷¼±Á¤º¸ Á¶»ç °á°ú¸¦ ÀúÀåÇÏ´Â ¹è¿­. 
	//		arr[x]Àº À¯È¿ÇÑ frameÀÇ x¹øÂ° Á÷¼± Á¤º¸ÀÌ´Ù.
	int arr[smp_frame] = { 0 };
	int r_arr[smp_frame] = { 0 };
	int l_arr[smp_frame] = { 0 };
	// n_arr : À¯È¿ÇÑ frameÀÇ °¹¼ö¸¦ indicateÇÑ´Ù.
	int n_arr = 0;
	int r_n_arr = 0;
	int l_n_arr = 0;
	// À¯È¿ÇÑ °á°úµé Áß, Áß¾Ó°ªÀ» ÃëÇÒ ¶§, ÀÌ º¯¼ö¿¡ °á°ú¸¦ ÀúÀåÇÔ.
	int mid_rho = -65538;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	// smp_frame¸¸Å­ ÇÁ·¹ÀÓÀ» Á¶»ç.
	for (n = 0; n < smp_frame; n++) {

		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);

		flg = chop_boundary(htda, htda_num, &pat, &rho);
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (pat) {
			case 0:
				l_arr[l_n_arr++] = rho;
				cnt[1]++;
				break;
			case 1:
				// ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ» ¶§, À¯È¿ Á÷¼± Á¤º¸¸¦ ±â·ÏÇÑ´Ù.
				arr[n_arr++] = rho;
				cnt[2]++;
				break;
			case 2:
				r_arr[r_n_arr++] = rho;
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	// cnt ¹è¿­À» Á¶»çÇÏ¿© °¡Àå ÁÖ¿äÇÑ ÆÐÅÏÀº ¾î¶² °ÍÀÎ°¡ °áÁ¤
	max_pat = 0;
	max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > max) {
			max_pat = n;
			max = cnt[n];
		}

	// ÁÖ¿äÇÑ ÆÐÅÏÀ» °á°ú·Î ÃëÇÔ.
	if (max_pat == 0) {
		// ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
		state = -1;
	}
	else if (max_pat == 1) {
		// skewed left
		// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
		for (n = (l_n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (l_arr[m] < l_arr[m + 1]) {
					tmp = l_arr[m];
					l_arr[m] = l_arr[m + 1];
					l_arr[m + 1] = tmp;
				}
		// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
		mid_rho = l_arr[(int)(l_n_arr / 2)];
		// state = 0;
		// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
		// if (mid_rho < 155)
		// 	state = 0;				
		// else
		state = 7; // Á¶±Ý ¿ìÇâ¿ì
	}
	else if (max_pat == 2) {
		// flat line
		// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
		for (n = (n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (arr[m] < arr[m + 1]) {
					tmp = arr[m];
					arr[m] = arr[m + 1];
					arr[m + 1] = tmp;
				}
		// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
		mid_rho = arr[(int)(n_arr / 2)];
		// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
		if (mid_rho < 38)
			state = 6;
		// else if (mid_rho < 50)
		// 	state = 3;
		// else if (mid_rho < 58)
			state = 4;
		// else if (mid_rho < 68)
		// 	state = 5;
		// else
		// 	state = 6;
		// if (mid_rho >= 68)	state = 6;// 190521 만든코드
	}
	else if (max_pat == 3) {
		// skewed right
		// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
		for (n = (r_n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (r_arr[m] < r_arr[m + 1]) {
					tmp = r_arr[m];
					r_arr[m] = r_arr[m + 1];
					r_arr[m + 1] = tmp;
				}
		// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
		mid_rho = r_arr[(int)(r_n_arr / 2)];
		// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
		// if (mid_rho > 30)
		// 	state = 1;
		// else
			state = 8; // Á¶±Ý ÁÂÇâÁÂ
	}


	free(htda);
	free(fvda);

	if (log) {
		fprintf(log, "[LOG] peek_bndy \n>> result table\n");
		fprintf(log, "pat |");
		for (n = 0; n < 4; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "vot |");
		for (n = 0; n < 4; n++)
			fprintf(log, " %4d  |", cnt[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "arr |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "rho |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " %4d  |", arr[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "rho of the detected line : %5d \n", mid_rho);
		fprintf(log, "=================================\n");
	}

	// return °ªÀ¸·Î, motion driver ÇÔ¼ö ³»ºÎ¿¡¼­ Çö Á÷¼± »óÅÂ¸¦ indicateÇÏ´Â °ª
	// state [-1] : pattern not appear
	// state [ 0] : skewed left (\) 
	// state [ 1] : skewed right (/)
	// state [ 2] : line too far
	// state [ 3] : line far
	// state [ 4] : line fitted
	// state [ 5] : line close
	// state [ 6] : line too close

	return state;
}


int line_target(FILE* log, int tar_rho, vMode behavior, vOpt tar_option, vObs usr_obs) {
	int n, m, htda_num;
	int rho, pat, flg, tmp, state = 0;

	// ÁÖ¿ä ÆÐÅÏ ºÐ·ù¿¡ ¾²ÀÌ´Â º¯¼ö, °¡Àå majorÇÑ ÆÐÅÏÀ» °¡·Á³¾ ¶§ »ç¿ëÇÔ.
	int max, max_pat;
	// cnt[4] : pattern ±×·ì ¹è¿­. smp_frame¸¸Å­ ÇÁ·¹ÀÓ Á¶»ç¸¦ ÇßÀ» ¶§, ³ª¿Â ÆÐÅÏÀÇ ¼ö¸¦ Á¶»çÇÑ´Ù.
	//	cnt[0] : ¾î¶² ÆÐÅÏµµ ¾øÀ½. ¿µ»ó¿¡ ¾î¶°ÇÑ À¯È¿ Á÷¼± Á¤º¸µµ ¾øÀ» ¶§.
	//	cnt[1], cnt[3] : ±â¿ï¾îÁø Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	//	cnt[2] : ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	int cnt[4] = { 0 };
	// arr : °¢ frameÀÇ Á÷¼±Á¤º¸ Á¶»ç °á°ú¸¦ ÀúÀåÇÏ´Â ¹è¿­. 
	//		arr[x]Àº À¯È¿ÇÑ frameÀÇ x¹øÂ° Á÷¼± Á¤º¸ÀÌ´Ù.
	int arr[smp_frame] = { 0 };
	// n_arr : À¯È¿ÇÑ frameÀÇ °¹¼ö¸¦ indicateÇÑ´Ù.
	int n_arr = 0;
	// À¯È¿ÇÑ °á°úµé Áß, Áß¾Ó°ªÀ» ÃëÇÒ ¶§, ÀÌ º¯¼ö¿¡ °á°ú¸¦ ÀúÀåÇÔ.
	int mid_rho = 0;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	// smp_frame¸¸Å­ ÇÁ·¹ÀÓÀ» Á¶»ç.
	for (n = 0; n < smp_frame; n++) {

		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);

		flg = chop_hrizon(htda, htda_num, &pat, &rho, tar_option, usr_obs);
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (pat) {
			case 0:
				cnt[1]++;
				break;
			case 1:
				// ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ» ¶§, À¯È¿ Á÷¼± Á¤º¸¸¦ ±â·ÏÇÑ´Ù.
				arr[n_arr++] = rho;
				cnt[2]++;
				break;
			case 2:
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	// cnt ¹è¿­À» Á¶»çÇÏ¿© °¡Àå ÁÖ¿äÇÑ ÆÐÅÏÀº ¾î¶² °ÍÀÎ°¡ °áÁ¤
	max_pat = 0;
	max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > max) {
			max_pat = n;
			max = cnt[n];
		}

	// 1. behavior == peeking
	//	tar_rho ÀÇ¹Ì ¾øÀ½. °¡·Î¼±ÀÇ À¯¹«¸¸ ÆÇ´ÜÇÔ.
	if (behavior == peeking) {
		if (max_pat == 0) // 
			state = -1;
		else
			state = 4;
	}
	// 2. behavior == fitting
	//	tar_rho ¹Ýµå½Ã ÁöÁ¤µÇ¾î¾ß ÇÔ.
	else if (behavior == fitting) {
		// ÁÖ¿äÇÑ ÆÐÅÏÀ» °á°ú·Î ÃëÇÔ.
		if (max_pat == 0) {
			// ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
			state = -1;
		}
		else if (max_pat == 1) {
			// skewed left
			state = 0;
		}
		else if (max_pat == 2) { 
			// flat line
			// Áß¾Ó°ªÀ» ÃëÇÏ±â À§ÇØ °¢ ÇÁ·¹ÀÓÀÇ À¯È¿ Á÷¼± Á¤º¸µéÀ» Á¤·ÄÇÑ´Ù.
			if (n_arr) {
				for (n = (n_arr - 1); n > 0; n--) {
					for (m = 0; m < n; m++) {
						if (arr[m] < arr[m + 1]) {
							tmp = arr[m];
							arr[m] = arr[m + 1];
							arr[m + 1] = tmp;
						}
					}
				}
			}

			if (n_arr)
				// Á¤·Ä ÈÄ, mid_rho¿¡ Áß¾Ó°ª ÀúÀå
				mid_rho = arr[(int)(n_arr / 2)];
			else
				mid_rho = 0;

			// Áß¾Ó°ªÀ¸·Î state ÆÇ´Ü.
			if (mid_rho < tar_rho - 20)
				state = 2;
			else if (mid_rho < tar_rho - 5)
				state = 3;
			else if (mid_rho < tar_rho + 5)
				state = 4;
			else if (mid_rho < tar_rho + 20)
				state = 5;
			else
				state = 6;
		}
		else if (max_pat == 3) { // skewed right
			state = 1;
		}
	}
	else {
		printf("unexpected mode\n");
		state = -1;
	}

	if (log) {
		fprintf(log, "[LOG] line_target \n>> result table\n");
		fprintf(log, "pat |");
		for (n = 0; n < 4; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "vot |");
		for (n = 0; n < 4; n++)
			fprintf(log, " %4d  |", cnt[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "arr |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "rho |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " %4d  |", arr[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "rho of the detected line : %5d \n", mid_rho);
		fprintf(log, "=================================\n");
	}

	free(fvda);
	free(htda);

	// state definition
	// state [-1] : pattern not appear
	// state [ 0] : skewed left (\) 
	// state [ 1] : skewed right (/)
	// state [ 2] : line too far
	// state [ 3] : line far
	// state [ 4] : line fitted
	// state [ 5] : line close
	// state [ 6] : line too close

	return state;
}



// 
int peek_start(FILE* log) {
	int cnt, n, px, py;

	U8 hue, sat, val;
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);
	U16* pala_m_out = (U16*)malloc(sizeof(U16)*pl);

	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];
	U16** pala_out = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala_out[n] = &pala_m_out[n * 180];
	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];

	read_fpga_video_data(fvda);
	get_surf(surf, fvda);
	memset(pala_mem, 0, sizeof(U16)*pl);

	cnt = 0;

	for (py = 0; py < 120; py++) {
		for (px = 0; px < 180; px++) {
			hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
			sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
			val = (U8)(0x000f & mysurf[py][px]);

			if ((hue < 70 && hue >= 32) && (sat > 2) && (val > 3)) {
				// cnt++;
				pala[py][px] = 0xffff;
			}
		}
	}

	morphology_Erode(pala, pala_out, 4, 4); 
	morphology_Dilate(pala_out, pala , 2, 2);
	
	for (py = 0; py < 120; py++) {
		for (px = 0; px < 180; px++) {
			
			if(pala[py][px] == 0xffff)
				cnt++;
		}
	}

	clear_screen();
	draw_fpga_video_data_full(pala_mem);
	flip();

	if (log) {
		fprintf(log, "[LOG] peek_start \n");
		fprintf(log, ">> (cnt, %6d)\n", cnt);
	}

	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);

	if (cnt < 500)
		return 0;

	return 1;
}


int peek_start_dmz(FILE* log, int motcnt) {
	int cnt, n, px, py,jh;
	int count1, count2;
	int jh_r=0;
	int jh_l=0;    // move left
	int jw_r=0;    
	int jw_l=0;


	U8 hue, sat, val;
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));	// 22000 * 2 bytes
	U16* surf = (U16*)malloc(sizeof(U16)*pl);			// surf 21600 * 2 bytes
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);

	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];

	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];

	Motion_SOC(mine_std);
	_wait_250ms;
	read_fpga_video_data(fvda);
	get_surf(surf, fvda);
	memset(pala_mem, 0, sizeof(U16)*pl);
	Motion_SOC(head_2cm);

	// clear_screen();
	// draw_fpga_video_data_full(pala_mem);
	// flip();
	cnt = 0;
	int fcnt = 0, Rcnt=0, Lcnt=0;
	jh=0;
	count1 = 0; count2 = 0;
	int state = 2, swit = 0;
	int breakflg = 0, blackflg = 0;
	int savedpx[2] = {0, 0};


	for (py = 0; py < 120; py++) {
		for (px = 0; px < 180; px++) {
			hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
			sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
			val = (U8)(0x000f & mysurf[py][px]);

			if(py == 95) pala[py][px] = 0x00ff;
			if(py == 68) pala[py][px] = 0x00ff;
			if(py == 45) pala[py][px] = 0x00ff;
			if(px == 141) pala[py][px] = 0x00ff;
			if(px == 100) pala[py][px] = 0x00ff; // 105 -> 110 -> 100
			if(px == 85) pala[py][px] = 0x00ff;	// 70 -> 75 -> 85
			if(px == 44) pala[py][px] = 0x0fff; // 25 -> 38

			if ((sat < 7 ) && (val < 4)) {         //black ///5    1->4->3	// val 3 -> 5 좀더 잘 인식되고자
				pala[py][px] = 0xffff;
			}
			else if((hue > 150) && (hue < 190)		// blue
				&& (sat > 3) && (val > 3)){
				pala[py][px] = 0x001f;
				cnt++;
			}
			
		}
	}
	clear_screen();
	draw_fpga_video_data_full(pala_mem);
	flip();


// 지뢰가 앞에 있는지 인식한다.

	for(py = 95; py >= 68; py--){
		for(px = 141; px > 44; px--){
			if((pala[py+1][px+1] == 0xffff)&&(pala[py+1][px] == 0xffff)&&(pala[py+1][px+2] == 0xffff)&&(pala[py][px+1] == 0xffff)&&(pala[py+2][px+1] == 0xffff)&&
				(pala[py+2][px+2] == 0xffff)&&(pala[py][px] == 0xffff)&&(pala[py][px+2] == 0xffff)&&(pala[py+2][px] == 0xffff)){
				if(py >= 68){
					if(px > 100){state = 11; savedpx[0] = 152 - px;}//breakflg = 1; break;}			// 지로가 바로 앞 오른쪽
					if(px <= 100 && px > 85){if(state == 11) state = 10; else state = 1;}  //breakflg = 1; break;} // 지뢰가 바로 앞에 있다 
					if(px <= 75 && px > 38){
						if(state == 11 || state == 10) {
							savedpx[1] = px - 38;
							 if(savedpx[0] < savedpx[1]) 
							 	state = 11; 
							 else state = 111;
						} 
						// else if(state == 1) state = 111; 
						else state = 111; breakflg = 1; break;
					} // 지뢰가 바로 앞 왼쪽
				}	// case 11 10 1 110 100 111 1000
				// if(py >= 38){
				// 	state = 2; // walk_1 정도 가능
				// 	breakflg = 1;
				// 	break;
				// }
				// if(py < 38){
				// 	state = 3; // walk_2 하고 끝내자.
				// 	breakflg = 1;
				// 	break;
				// }
			}
		}
		if(breakflg == 1){
			breakflg = 0;
			break;
		}
	}
	if(state == 2){
		for(py= 0; py <= 68; py++){
			for(px = 145; px > 40; px--){
				if((pala[py+1][px+1] == 0xffff)&&(pala[py+1][px] == 0xffff)&&(pala[py+1][px+2] == 0xffff)&&(pala[py][px+1] == 0xffff)&&(pala[py+2][px+1] == 0xffff)&&
					(pala[py+2][px+2] == 0xffff)&&(pala[py][px] == 0xffff)&&(pala[py][px+2] == 0xffff)&&(pala[py+2][px] == 0xffff)){
					if(py >= 45){
						state = 2; // walk_1 정도 가능
						breakflg = 1;
						break;
					}
					if(py < 45){
						state = 3; // walk_2 하고 끝내자.
						breakflg = 1;
						break;
					}
				}
			}
			if(breakflg == 1){
				breakflg = 0;
				break;
			}
			else if(py == 68) state = 4;
		}
	}

if(state == 1 || state == 11 || state == 111 || state == 10){

	for(py=120; py >= 0; py--){
		for(px=38; px >= 0; px--){if(pala[py][px] == 0xffff) Lcnt++;}	 // 38*121 = 4598
		for(px=180; px >= 160; px--){if(pala[py][px] == 0xffff) Rcnt++;} // 21*121 = 2541 
	}

	if(Lcnt > 722) blackflg = 1;// 19*38 = 722 -> 왼쪽에 인식되는 것은 지뢰가 아니라 검은 선이다. 
	if(Rcnt > 252) blackflg = 2;// 21*12 = 252 -> 오른쪽 블라블라

	for(py=120; py > 95; py--){
		for(px=38; px >=0; px--){
			if((pala[py+1][px+1] == 0xffff)&&(pala[py+1][px] == 0xffff)&&(pala[py+1][px+2] == 0xffff)&&(pala[py][px+1] == 0xffff)&&(pala[py+2][px+1] == 0xffff)&&
				(pala[py+2][px+2] == 0xffff)&&(pala[py][px] == 0xffff)&&(pala[py][px+2] == 0xffff)&&(pala[py+2][px] == 0xffff) && blackflg != 1){
				if(state == 1){
				state = 12;
				breakflg = 1;
				break;}
				if(state == 11){state = 112; breakflg = 1; break;}
				if(state == 10){state = 102; breakflg = 1; break;}
				if(state == 111){state = 1112; breakflg = 1; break;}

			}
		}
		if(breakflg == 1){
			breakflg = 0;
			break;
		}
	}

	for(py=120; py > 95; py--){
		for(px=180; px >=155; px--){
			if((pala[py+1][px+1] == 0xffff)&&(pala[py+1][px] == 0xffff)&&(pala[py+1][px+2] == 0xffff)&&(pala[py][px+1] == 0xffff)&&(pala[py+2][px+1] == 0xffff)&&
				(pala[py+2][px+2] == 0xffff)&&(pala[py][px] == 0xffff)&&(pala[py][px+2] == 0xffff)&&(pala[py+2][px] == 0xffff) && blackflg != 2){
				if(state == 12){
					state = 123;
					breakflg = 1;
					break;
				}
				if(state == 112 || state == 102){state = 1123; breakflg = 1; break;}
				if(state == 1112){state = 11123; breakflg = 1; break;}
				if(state == 1 || state == 11 || state == 10){
					state = 13;
					breakflg = 1;
					break;
				}
				if(state == 111){
					state = 113;
					breakflg = 1;
					break;
				}
			}
		}
		if(breakflg == 1){
			breakflg = 0;
			break;
		}
	}
}
blackflg = 0;
// 지뢰 인식에 따른 행동 판단

	switch(state){
		case 10:
		case 11:
		case 13:
		Motion_SOC(sl_4);
		// _wait_d;
		// Motion_SOC(head_2cm);
		// Motion_SOC(walk_2);
		break;
		case 112:
		Motion_SOC(sl_3);
		Motion_SOC(head_2cm);
		Motion_SOC(walk_3);
		break;
		case 113:
		Motion_SOC(sr_3);
		Motion_SOC(head_2cm);
		Motion_SOC(walk_3);
		break;
		case 12:
		case 111:
		case 102:
		case 1112:
		Motion_SOC(sr_4);
		// _wait_d;
		// Motion_SOC(head_2cm);
		// Motion_SOC(walk_2);
		break;
		case 1:
		case 123:
		Motion_SOC(mine);
		motcnt = 3;
		break;
		case 2:
		Motion_SOC(walk_1);
		break;
		case 3:
		Motion_SOC(walk_2);
		break;
		case 4:
		Motion_SOC(walk_3);
		break;
		case 1123:
		Motion_SOC(tr_3);
		_wait_d;
		Motion_SOC(head_2cm);
		Motion_SOC(walk_6);
		motcnt=3;
		break;
		case 11123:
		Motion_SOC(tl_3);
		_wait_d;
		Motion_SOC(head_2cm);
		Motion_SOC(walk_6);
		motcnt = 3;
		break;
	}

	if(motcnt % 4 == 3){
		mot_arrangeMid_2(fcnt);
	}


	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);
	if (log) {
		fprintf(log, "[LOG] peek_start \n");
		fprintf(log, ">> (cnt, %6d)\n", cnt);
	}


	if (cnt > 100){
		return 1;
	}
	else return 0;

/*
//지뢰 인식 영역  display(노란색 선)
	
	for (py = 60; py < 110; py++) {
		for (px = 40; px < 140; px++) {
				if(py == 60 || py == 109 || px == 40 || px == 139)
					pala[py][px] = 0xff00;
			}
		}
//지뢰 인식후 state값 주기

	for (py = 60; py < 110; py++) {
		for (px = 40; px < 140; px++) {
			if((pala[py][px] == 0xffff)&&(pala[py][px-1] == 0xffff)&&(pala[py][px+1] == 0xffff)&&(pala[py-1][px] == 0xffff)&&(pala[py+1][px] == 0xffff)&&
				(pala[py+1][px+1] == 0xffff)&&(pala[py-1][px-1] == 0xffff)&&(pala[py-1][px+1] == 0xffff)&&(pala[py+1][px-1] == 0xffff))
			{
				if(px>=50 && px<70)
					state = 1; // 지뢰가 오른쪽 가까이에 있을 경우
				else if(px >=70 && px<95)	
					state = 2; //지뢰가 오른쪽 멀리에 있을 경우
				else if(px>=95 && px < 120)
					state = 3; // 지뢰가 왼쪽 가까이에 있을 경우
				else if(px >= 120 && px < 140)
					state = 4; // 지뢰가 왼쪽 멀리에 있을 경우
			}
			if(state != 0)
				break;
		}
	}

// state값에 따라 Motion 호출

		switch(state){
			case 1: 
				Motion_SOC(sr_3);
				break;
			case 2:
				Motion_SOC(sr_2);
				break;
			case 3: 
				Motion_SOC(sl_3);
				break;
			case 4:
				Motion_SOC(sl_2);
				break;
			default:
				Motion_SOC(walk_2);
				// Motion_SOC(tr_1); //땅 기울어져서 넣음
				_wait;
				break;
		}

		state = 0;
*/

	// clear_screen();
	// draw_fpga_video_data_full(pala_mem);
	// flip();


////////////////////////////////////////////
	
}
////////////////////////////////////////////////////////////////

int peek_start_yellow(FILE* log) {
	int cnt, n, px, py,jh;
	int count1, count2;
	int cnt_jh=0;

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

	read_fpga_video_data(fvda);
	get_surf(surf, fvda);
	memset(pala_mem, 0, sizeof(U16)*pl);

	//Motion_SOC(head_bot);



	for (py = 0; py < 120; py++) {
		for (px = 0; px < 180; px++) {
			hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
			sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
			val = (U8)(0x000f & mysurf[py][px]);

			if((hue < 70 && hue >= 32) && (sat > 2) && (val > 5)){

				pala[py][px] = 0xffff;
				cnt_jh++;
			}
			else{

			}
			
		}
	}


	clear_screen();
	draw_fpga_video_data_full(pala_mem);
	flip();



	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);


	int i;
	int dist1, dist2;
	U16* output = (U16*)malloc(pl * 2);
////////////////////////////////////////////

	if (cnt_jh > 15000)
		return 1;

	return 0;
}





// bridge¸¦ °Ç³Ê´Âµ¥ »óÅÂ¸¦ °áÁ¤ÇÏ´Â ÇÔ¼ö.
// obstacle
//	: bridge_on, trap_hole, default_obs
// obstacle == bridge_on ÀÏ¶§, onBridge ÆÄ¶ó¹ÌÅÍ´Â 0, 1À» °¡Áú ¼ö ÀÖ´Ù.
// obstacle == trap_hole, default_obsÀÏ ¶§, onBridge ÆÄ¶ó¹ÌÅÍ´Â ¹Ýµå½Ã 1ÀÌ¾î¾ß ÇÑ´Ù.
//	: peek_vert(1, bridge_on), peek_vert(0, bridge_on);
//	: peek_vert(1, trap_hole), peek_vert(1, default_obs);
int peek_vert(FILE* log, int onBridge, vObs obstacle) {
	int n, m, htda_num;
	int rho1, rho2, pat, flg, tmp, state = 0;
	// ÁÖ¿ä ÆÐÅÏ ºÐ·ù¿¡ ¾²ÀÌ´Â º¯¼ö, °¡Àå majorÇÑ ÆÐÅÏÀ» °¡·Á³¾ ¶§ »ç¿ëÇÔ.
	int max, max_pat;
	// cnt[4] : pattern ±×·ì ¹è¿­. smp_frame¸¸Å­ ÇÁ·¹ÀÓ Á¶»ç¸¦ ÇßÀ» ¶§, ³ª¿Â ÆÐÅÏÀÇ ¼ö¸¦ Á¶»çÇÑ´Ù.
	//	cnt[0] : ¾î¶² ÆÐÅÏµµ ¾øÀ½. ¿µ»ó¿¡ ¾î¶°ÇÑ À¯È¿ Á÷¼± Á¤º¸µµ ¾øÀ» ¶§.
	//	cnt[1], cnt[3] : ±â¿ï¾îÁø Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	//	cnt[2] : ¹Ù¸¥ Á÷¼± ÆÐÅÏÀÌ ³ªÅ¸³µÀ½.
	int cnt[4] = { 0 };
	// frame ³» À¯È¿ÇÑ Á÷¼± Á¤º¸ Ãâ·ÂÀ» ´ã´Â´Ù.
	int vlines[smp_frame * 2] = { 0 };
	// n_val : vlines ¹è¿­ÀÇ À¯È¿ÇÑ Á÷¼± Á¤º¸·®À» indicateÇÑ´Ù.
	int n_val = 0;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	// smp_frame¸¸Å­ Á÷¼± Á¤º¸ °Ë»ç.
	for (n = 0; n < smp_frame; n++) {
		// ÇÁ·¹ÀÓ Á÷¼± Á¤º¸ ÃßÃâ
		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);
		// ÇÑ ÇÁ·¹ÀÓ ³» Á÷¼± Á¤º¸ Áß À¯È¿ÇÑ Á¤º¸(¼¼·Î¼±) °Ë»ç.
		flg = chop_vert(htda, htda_num, &pat, &rho1, &rho2, obstacle);
		//printf("cp0\n");
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (pat) {
			case 0:
				cnt[1]++;
				break;
			case 1:
				// À¯È¿ÇÑ Á÷¼±¿¡ ´ëÇÏ¿© ¹è¿­¿¡ Ãß°¡.
				if (rho1 != -1234)
					vlines[n_val++] = rho1;
				if (rho2 != -1234)
					vlines[n_val++] = rho2;
				cnt[2]++;
				break;
			case 2:
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	//	printf("dbg ptr11 : %3d / %3d \n", n_idx_ptr_left, n_idx_ptr_right);

	// cnt¸¦ ºñ±³ÇÏ¿© majorÇÑ ÆÐÅÏ °áÁ¤.
	max_pat = 0;
	max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > max) {
			max_pat = n;
			max = cnt[n];
		}

	// second, arr Á¤·Ä
	// Áß°£°ªÀ¸·Î ÆÇ´Ü.
	if (max_pat == 0) {
		// ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
		if (onBridge) {
			int score_rl, score_lr;
			int var_left = -120;   //-60
			int var_right = 60;    //30
			// ÇÁ·¹ÀÓ hsv µ¥ÀÌÅÍ ÃßÃâ
			read_fpga_video_data(fvda);
			get_surf(surf, fvda);
			score_lr = sideLookupCRatio_jh(surf, BLACK, var_left, 60, 0); // region_lr
			score_rl = sideLookupCRatio_jh(surf, BLACK, var_right, 60, 1); // region_rl

			// ¿ÞÂÊ, ¿À¸¥ÂÊ ¸ðµÎ ¸¸Á·.
			if (score_lr >= 120 && score_rl >= 120 )
				state = 4;
			// ¿À¸¥ÂÊ¸¸ ¸¸Á·.
			else if (score_lr < 120 && score_rl >= 120)
				state = 5;
			// ¿ÞÂÊ¸¸ ¸¸Á·.
			else if (score_lr >= 120 && score_rl < 120)
				state = 3;
			// ¿¹¿Ü.
			else // ¶ì¿ë?
				state = -1;
		}
		else
			state = -1;
	}
	else if (max_pat == 1) {
		// skewed left
		state = 1;
	}
	else if (max_pat == 2) { 
		// ¹Ù¸¥ Á÷¼±ÀÌ ÁÖ¿äÇÑ °ÍÀ¸·Î ÆÐÅÏÀÌ °áÁ¤µÊ.
		// ÀÌÁ¦ ¾òÀº Á÷¼±µ¥ÀÌÅÍ¸¦ Á¤·ÄÇÑ ÈÄ, ±×·ìÁþ±â¸¦ ¼öÇàÇÏ¿©, ÁÖ¿äÇÑ µÎ Á÷¼± Á¤º¸¸¦ ¾ò¾î³¾ °ÍÀÓ.
		
		// grouping variables
		int grho[4] = { 0 };
		int gamt[4] = { 0 };
		int gptr, gcnt, flg, gtmp, grp_n = 0;
		float gavg, gsum, delta;

		//
		int rho_left, rho_right;

		// À¯È¿ Á÷¼±Á¤º¸¸¦ Á¤·ÄÇÔ. ¿À¸§Â÷¼ø. È­¸é»ó ¿ÞÂÊ Á÷¼±ÀÌ ¹è¿­ÀÇ ³·Àº ÀÎµ¦½º¿¡ ¸ð¿©µé °ÍÀÓ.
		if (n_val) {
			// bubble sort
			for (n = (n_val - 1); n > 0; n--) {
				for (m = 0; m < n; m++) {
					if (vlines[m] > vlines[m + 1]) {
						tmp = vlines[m];
						vlines[m] = vlines[m + 1];
						vlines[m + 1] = tmp;
					}
				}
			}
		}
		else {
			// ÁÖ¿ä ÆÐÅÏÀÌ ¹Ù¸¥ Á÷¼±À¸·Î °áÁ¤µÇ¾úÁö¸¸, ¾î¶°ÇÑ À¯È¿ Á÷¼±µµ Á¸ÀçÇÏÁö ¾Ê´Â °æ¿ì´Â
			// ÀÏ¾î³¯ ¼ö ¾ø´Â ¿¹¿ÜÀÌÁö¸¸, ¾ÈÀüÀåÄ¡·Î Ã³¸®.
			free(htda);
			free(surf);
			free(fvda);
			return -1;
		}
		
		// grouping
		// ±×·ìÀº 4°³ ±îÁö.
		// ±×·ìÁþ±â¸¦ ½ÃÀÛÇÑ´Ù.
		gptr = 0;
		// ÃÖ´ë 4°³ÀÇ ±×·ìÁþ±â
		for (n = 0; n < 4; n++) {
			// ´ÙÀ½ µ¥ÀÌÅÍ°¡ ¾øÀ¸¹Ç·Î, ±×·ìÁþ±â¸¦ ³¡³¿
			if (gptr == n_val)
				break;
			// ÃÊ±â ±×·ìÀº ÀÚ±â ÀÚ½ÅÀ» sum°ú avg·Î °¡Áø´Ù.
			gsum = (float)vlines[gptr];
			gavg = (float)vlines[gptr];
			gcnt = 1;
			while (1) {
				gptr++;
				// ´ÙÀ½ µ¥ÀÌÅÍ°¡ ¾øÀ¸¹Ç·Î, ±×·ìÁþ±â¸¦ ³¡³¿
				if (gptr == n_val) {
					grho[n] = (int)gavg;
					gamt[n] = gcnt;
					grp_n++;
					break;
				}
				// ±×·ì Æò±Õ(gavg)°ú ´ÙÀ½ °ªÀÇ Â÷(delta)¸¦ Á¶»çÇÑ´Ù.
				delta = (gavg - (float)vlines[gptr]);
				// delta°¡ ÀûÀýÇÏ¸é ±×·ì¿¡ Æ÷ÇÔÇÑ´Ù.
				if (delta < 10 && delta > -10) {
					gsum += (float)vlines[gptr];
					gcnt++;
					// ±×·ì Æò±Õ °»½Å
					gavg = gsum / gcnt;
				}
				// delta°¡ ±×·ì¿¡ Æ÷ÇÔµÇÁö ¾ÊÀ» ¸¸Å­ Å©¸é, ±×·ìÁþ±â¸¦ ³¡³½´Ù.
				else {
					grho[n] = (int)gavg;
					gamt[n] = gcnt;
					grp_n++;
					break;
				}
			}
		}

		// ±×·ìÁþ±â°¡ ³¡³ª°í, °¡´ÉÇÑ majorÇÑ µÎ °³ÀÇ ±×·ìÀ» ¼±ÅÃÇØ¾ß ÇÑ´Ù.
		// °¡Àå Å« Å©±âÀÇ µÎ ±×·ìÀ» ¼±ÅÃÇÏ±â À§ÇØ¼­, °ËÃâµÈ ±×·ì¿¡ ´ëÇÏ¿© sortingÀ» ¼öÇàÇÑ´Ù.
		for (n = (grp_n - 1); n > 0; n--) {
			for (m = 0; m < n; m++) {
				// ±×·ì Å©±â ºñ±³ ³»¸²Â÷¼ø Á¤·Ä
				if (gamt[m] < gamt[m + 1]) {
					gtmp = gamt[m];
					gamt[m] = gamt[m + 1];
					gamt[m + 1] = gtmp;

					gtmp = grho[m];
					grho[m] = grho[m + 1];
					grho[m + 1] = gtmp;
				}
			}
		}

		// ±×·ìÀÌ ÇÏ³ª »ÓÀÎ °æ¿ì, ±× ±×·ìÀ» ¼±ÅÃÇÔ. À½¼öÀÏ °æ¼ö ¿ÞÂÊ¼±, ¾ç¼öÀÏ °æ¿ì ¿À¸¥ÂÊ ¼±À¸·Î °£ÁÖ.
		// -1234, n/a Á¸ÀçÇÏÁö ¾ÊÀ½À» ³ªÅ¸³¿.
		if (grp_n < 2) {
			if (grho[0] < 0) {
				rho_left = grho[0];
				rho_right = -1234;
			}
			else {
				rho_left = -1234;
				rho_right = grho[0];
			}
			// °ËÃâµÈ ¼± ÇÏ³ª.
			flg = 0;
		}
		else {
			// sortingÇÑ ÀÌÈÄÀÌ¹Ç·Î, grho[0]¿Í grho[1]°¡ ±×·ì Å©±â ¼øÀ¸·Î Ã¹¹øÂ°, µÎ¹øÂ°ÀÌ´Ù.
			if (grho[1] > grho[0]) {
				rho_left = grho[0];
				rho_right = grho[1];
			}
			else {
				rho_left = grho[1];
				rho_right = grho[0];
			}
			// °ËÃâµÈ ¼± µÑ ÀÌ»ó.
			flg = 1;
		}

		rho1 = rho_left;
		rho2 = rho_right;

		// µÎ Á÷¼± Á¤º¸¸¦ ¾Ë¾Æ³ÂÀ¸¹Ç·Î, Á¸ÀçÇÏ´Â Á÷¼± Á¤º¸ÀÇ °¹¼ö(1°³ or 2°³), onBridge³Ä ¾Æ´Ï³Ä¿¡ µû¶ó Ã³¸®.
		if (!flg) {
			// Á÷¼± Á¤º¸°¡ 1°³ÀÏ °æ¿ì
			// Â¡°Ë´Ù¸® À§°¡ ¾Æ´Ò °æ¿ì. °ËÃâµÈ ¼±À» ±âÁØÀ¸·Î ÁÂ¿ì »ö»óÀ» Á¶»çÇÑ´Ù.
			if (!onBridge) {
				COLOR region_l = NONE;
				COLOR region_r = NONE;

				read_fpga_video_data(fvda);
				get_surf(surf, fvda);
#ifdef lcd_dbg
				U16* tmp_surf = (U16*)malloc(sizeof(U16)*pl);
				colorRegion(tmp_surf, surf, GREEN);
#endif
				// ¿ÞÂÊ¼± ±âÁØ
				if (rho_left != -1234 && rho_right == -1234) {
					region_l = sideLookupColor(surf, rho_left, 1);
					region_r = sideLookupColor(surf, rho_left, 0);
					flg = 0;
#ifdef lcd_dbg
					sideRegionCheck2(tmp_surf, rho_left, 1, 0);
					sideRegionCheck2(tmp_surf, rho_left, 0, 1);
#endif
				}
				// ¿À¸¥ÂÊ¼± ±âÁØ
				else if (rho_left == -1234 && rho_right != -1234) {
					region_l = sideLookupColor(surf, rho_right, 1);
					region_r = sideLookupColor(surf, rho_right, 0);
					flg = 1;
#ifdef lcd_dbg
					sideRegionCheck2(tmp_surf, right_rho, 1, 2);
					sideRegionCheck2(tmp_surf, right_rho, 0, 3);
#endif
				}
				// ÀÖÀ» ¼ö ¾ø´Â ¿¹¿Ü
				else {
#ifdef lcd_dbg
					free(tmp_surf);
#endif
					free(surf);
					free(htda);
					free(fvda);
					return -1;
				}

#ifdef lcd_dbg
				printf("region state (left / right) : %2d / %2d\n", region_l, region_r);
				clear_screen();
				draw_fpga_video_data_full(tmp_surf);
				flip();
				free(tmp_surf);
#endif

				// ÁÂ¿ì »ö»ó Á¶»ç °á°ú¸¦ Åä´ë·Î state¸¦ °áÁ¤ÇÑ´Ù.
				// GREEN »ö»óÀÌ ÀÖ´Â ÂÊÀ¸·Î ¿òÁ÷ÀÓ
				if (!flg) {
					if (region_l == GREEN)
						state = 2;
					else if (region_r == GREEN)
						state = 6;
					else
						state = 5;
				}
				else {
					if (region_l == GREEN)
						state = 2;
					else if (region_r == GREEN)
						state = 6;
					else
						state = 3;
				}
			}
			// Â¡°Ë´Ù¸® À§ÀÏ °æ¿ì.
			else {
				if (rho_left != -1234 && rho_right == -1234) {
					// ¿ÞÂÊ ¼±¿¡ ÀÇÇØ state °áÁ¤
					if (rho_left < -65)
						state = 5;
					else if (rho_left < -40)
						state = 4;
					else
						state = 3;
				}
				else if (rho_left == -1234 && rho_right != -1234) {
					// ¿À¸¥ÂÊ ¼±¿¡ ÀÇÇØ state °áÁ¤
					if (rho_right < 18)
						state = 3;
					else if (rho_right < 42)
						state = 4;
					else
						state = 5;
				}
				// Á¸ÀçÇÒ ¼ö ¾ø´Â ¿¹¿Ü.
				else {
					free(surf);
					free(htda);
					free(fvda);
					return -1;
				}
			}
		}
		// Á÷¼± Á¤º¸°¡ 2°³ÀÏ °æ¿ì
		else {
			// Â¡°Ë´Ù¸® À§°¡ ¾Æ´Ò °æ¿ì.
			if (!onBridge) {
				COLOR region_ll = NONE; // left_line_left_region;
				COLOR region_lr = NONE; // left_line_right_region;
				COLOR region_rl = NONE; // right_line_left_region;
				COLOR region_rr = NONE; // right_line_right_region;

				read_fpga_video_data(fvda);
				get_surf(surf, fvda);

#ifdef lcd_dbg
				U16* tmp_surf = (U16*)malloc(sizeof(U16)*pl);
				colorRegion(tmp_surf, surf, GREEN);
#endif
				// ¿ÞÂÊ ¿À¸¥ÂÊ ¼±¿¡ ´ëÇØ¼­ ÁÂ¿ì »ö»ó Á¶»ç.
				// ¿ÞÂÊ ¼±ÀÇ ¿ÞÂÊ Á¶»ç
				region_ll = sideLookupColor(surf, rho_left, 1);
				// ¿ÞÂÊ ¼±ÀÇ ¿À¸¥ÂÊ Á¶»ç
				region_lr = sideLookupColor(surf, rho_left, 0);
				// ¿À¸¥ÂÊ ¼±ÀÇ ¿ÞÂÊ Á¶»ç
				region_rl = sideLookupColor(surf, rho_right, 1);
				// ¿À¸¥ÂÊ ¼±ÀÇ ¿À¸¥ÂÊ Á¶»ç
				region_rr = sideLookupColor(surf, rho_right, 0);
#ifdef lcd_dbg
				sideRegionCheck2(tmp_surf, left_rho, 1, 0);
				sideRegionCheck2(tmp_surf, left_rho, 0, 1);
				sideRegionCheck2(tmp_surf, right_rho, 1, 2);
				sideRegionCheck2(tmp_surf, right_rho, 0, 3);

				printf("region state (ll / lr / rl / rr) : %2d / %2d / %2d / %2d \n", region_ll, region_lr, region_rl, region_rr);
				draw_fpga_video_data_full(tmp_surf);
				flip();
				free(tmp_surf);
#endif

				// GREEN »ö»óÀÌ ÀÖ´Â ÂÊÀ¸·Î ÀÌµ¿, »ö»ó¿¡ ÀÇÇØ state °áÁ¤!
				if (region_ll == BLACK)
					// ³­ ¿À¸¥ÂÊ ³¡¿¡ ÀÖÀ½
					state = 2;
				else if (region_lr == BLACK || region_rl == BLACK)
					// ¿©±ä Â¡°Ë´Ù¸® ¾Õ, ÁÂ¿ì ¼±ÀÇ inward·Î GREEN!
					// threshold¿¡ ¸ÂÃç ¸ð¼Ç °áÁ¤!
					if (rho_right < 18)
						state = 5;
					else if (rho_right < 42)
						state = 4;
					else
						state = 3;
				else if (region_rr == BLACK)
					// ³­ ¿ÞÂÊ ³¡¿¡ ÀÖÀ½!
					state = 6;
				else // undefined
					state = 3;
			}
			// Â¡°Ë´Ù¸® À§ÀÏ °æ¿ì
			else {
				// threshold¿¡ ¸ÂÃç ¸ð¼Ç °áÁ¤
				if (rho_right < 18)
					state = 3;
				else if (rho_right < 42)
					state = 4;
				else
					state = 5;
			}
		}
	}
	else if (max_pat == 3) { // skewed right
		state = 0;
	}

	free(htda);
	free(surf);
	free(fvda);


	
	// state [-1] : pattern not appear
	// state [ 0] : skewed left (\) 
	// state [ 1] : skewed right (/)
	// state [ 2] : rapid left move
	// state [ 3] : sided left
	// state [ 4] : fitted
	// state [ 5] : sided right
	// state [ 6] : rapid right move

	return state;
}
// ÀÌÀü version
int bdgpeek(FILE* log, int onBridge, vObs obstacle) {
	int n, m, htda_num;
	int scr_left, scr_right, grp, flg, tmp, state = 0;

	int cnt_max, cnt_max_idx;
	int cnt[4] = { 0 };
	int arr_left[smp_frame];
	int arr_right[smp_frame];
	int n_idx_ptr_left = 0;
	int n_idx_ptr_right = 0;
	int left_rho;
	int right_rho;

	memset(arr_left, -1234, smp_frame * sizeof(int));
	memset(arr_right, -1234, smp_frame * sizeof(int));

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	// this function will work properly when two lines(left, right) exist

	for (n = 0; n < smp_frame; n++) {

		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);

		// state [-1] : pattern not appear
		// state [ 0] : skewed left (\) 
		// state [ 1] : skewed right (/)
		// state [ 2] : rapid left move
		// state [ 3] : sided left
		// state [ 4] : fitted
		// state [ 5] : sided right
		// state [ 6] : rapid right move

		flg = vert_map_for_bdgpeek(htda, htda_num, &grp, &scr_left, &scr_right, obstacle);
		//printf("cp0\n");
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (grp) {
			case 0:
				cnt[1]++;
				break;
			case 1:	// insert score into arr.
					//printf("scr_left / scr_right : %5d / %5d \n", scr_left, scr_right);
				if (scr_left != -1234)
					arr_left[n_idx_ptr_left++] = scr_left;
				if (scr_right != -1234)
					arr_right[n_idx_ptr_right++] = scr_right;
				cnt[2]++;
				break;
			case 2:
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	//	printf("dbg ptr11 : %3d / %3d \n", n_idx_ptr_left, n_idx_ptr_right);

	// first, cnt Á¶»ç
	// °¡Àå Å« °ªÀ» °¡Áö´Â cnt?
	cnt_max_idx = 0;
	cnt_max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > cnt_max) {
			cnt_max_idx = n;
			cnt_max = cnt[n];
		}

	// second, arr Á¤·Ä
	// Áß°£°ªÀ¸·Î ÆÇ´Ü.
	if (cnt_max_idx == 0) {	// ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
		if (onBridge) {
			int score_rl, score_lr;
			get_surf(surf, fvda);
			// Á¤¸» ¾Æ¹«°Íµµ ¾ø´ÂÁö ÇÑ¹ø È®ÀÎÇØ º¼±î¿ä? ^^

			score_lr = sideLookupCRatio(surf, GREEN, -60, 30, 0); // region_lr
			score_rl = sideLookupCRatio(surf, GREEN, 30, 30, 1); // region_rl

			if (score_lr >= 60 && score_rl >= 60)
				state = 4;
			else if (score_lr < 60 && score_rl >= 60)
				state = 5;
			else if (score_lr >= 60 && score_rl < 60)
				state = 3;
			else // ¶ì¿ë?
				state = -1;
		}
		else
			state = -1;
	}
	else if (cnt_max_idx == 1) { // skewed left
		state = 1;
	}
	else if (cnt_max_idx == 2) { // flat line
								 // sorting
								 // first Á¤·ÄÇÒ µ¥ÀÌÅÍ°¡ ÀÖ´ÂÁö °Ë»ç
		if (n_idx_ptr_left) {
			// bubble sort
			for (n = (n_idx_ptr_left - 1); n > 0; n--)
				for (m = 0; m < n; m++)
					if (arr_left[m] < arr_left[m + 1]) {
						tmp = arr_left[m];
						arr_left[m] = arr_left[m + 1];
						arr_left[m + 1] = tmp;
					}
			left_rho = arr_left[(int)(n_idx_ptr_left / 2)];
		}
		else
			left_rho = -1234;

		if (n_idx_ptr_right) {
			for (n = (n_idx_ptr_right - 1); n > 0; n--)
				for (m = 0; m < n; m++)
					if (arr_right[m] < arr_right[m + 1]) {
						tmp = arr_right[m];
						arr_right[m] = arr_right[m + 1];
						arr_right[m + 1] = tmp;
					}
			right_rho = arr_right[(int)(n_idx_ptr_right / 2)];
		}
		else
			right_rho = -1234;

		// left_rho, right_rho¿¡ Áß°£°ª ÀúÀå

		//printf("n_idx_left / n_idx_right : %5d / %5d \n", n_idx_ptr_left, n_idx_ptr_right);
		//printf("left_rho / right_rho : %5d / %5d \n", left_rho, right_rho);
		// left_rho, right_rho check.
		/*
		// Â¡°Ë´Ù¸® À§°¡ ¾Æ´Ò °æ¿ì. ¿¹¿ÜÃ³¸®°¡ ¸¹½À´Ï´Ù. ¤Ð¤Ð
		if (!onBridge) {
			COLOR region_ll = NONE; // left_line_left_region;
			COLOR region_lr = NONE; // left_line_right_region;
			COLOR region_rl = NONE; // right_line_left_region;
			COLOR region_rr = NONE; // right_line_right_region;
			get_surf(surf, fvda);
#ifdef lcd_dbg
			U16* tmp_surf = (U16*)malloc(sizeof(U16)*pl);
			// ¼± ÁÂ¿ì·Î »ö»óÀ» ½ÃÇèÇØ ºÁ¾ß ÇÑ´Ù.
			// »ö»ó ÃßÃâ
			clear_screen();
			colorRegion(tmp_surf, surf, GREEN);
#endif

			// ¼±ÀÌ ¾çÂÊ´Ù ÀÖ´Ù.
			if (left_rho != -1234 && right_rho != -1234) {
				// ¿ÞÂÊ ¼±¿¡ ´ëÇØ¼­ »ö»óÀ» Á¶»çÇØ º¸ÀÚ.
				// ¿ÞÂÊ ¼±ÀÇ ¿ÞÂÊ Á¶»ç
				region_ll = sideLookupColor(surf, left_rho, 1);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, left_rho, 1, 0);
#endif
				// ¿ÞÂÊ ¼±ÀÇ ¿À¸¥ÂÊ Á¶»ç
				region_lr = sideLookupColor(surf, left_rho, 0);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, left_rho, 0, 1);
#endif

				// ¿À¸¥ÂÊ ¼±¿¡ ´ëÇØ¼­ »ö»ýÀ» Á¶»çÇØ º¸ÀÚ.
				// ¿À¸¥ÂÊ ¼±ÀÇ ¿ÞÂÊ Á¶»ç
				region_rl = sideLookupColor(surf, right_rho, 1);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, right_rho, 1, 2);
#endif
				// ¿À¸¥ÂÊ ¼±ÀÇ ¿À¸¥ÂÊ Á¶»ç
				region_rr = sideLookupColor(surf, right_rho, 0);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, right_rho, 0, 3);
#endif

				// GREEN »ö»óÀÌ ÀÖ´Â ÂÊÀ¸·Î ¿òÁ÷ÀÔ½Ã´Ù!
				if (region_ll == GREEN)	// ³­ ¿À¸¥ÂÊ ³¡¿¡ ÀÖ±º¿ä!
					state = 2;
				else if (region_lr == GREEN || region_rl == GREEN) // ¿©±ä Â¡°Ë´Ù¸® ¾ÕÀÌ±º¿ä!
																   // threshold¿¡ ¸ÂÃç ¸ð¼Ç °áÁ¤ this is what I just wanted!
					if (right_rho < 18)
						state = 5;
					else if (right_rho < 42)
						state = 4;
					else
						state = 3;
				else if (region_rr == GREEN) // ³­ ¿ÞÂÊ ³¡¿¡ ÀÖ½À´Ï´Ù!
					state = 6;
				else // ¾î... ¹¹ÁÒ? Àü ¾îµðÁÒ?
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ª ¹Û¿¡ ¾ø´Ù.
			else if (left_rho == -1234 && right_rho != -1234) { // ¿À¸¥ÂÊ ¼±
																//¼±ÀÇ ¿ÞÂÊ Á¶»ç
				region_rl = sideLookupColor(surf, right_rho, 1);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, right_rho, 1, 2);
#endif
				//¼±ÀÇ ¿À¸¥ÂÊ Á¶»ç
				region_rr = sideLookupColor(surf, right_rho, 0);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, right_rho, 0, 3);
#endif
				// GREEN »ö»óÀÌ ÀÖ´Â ÂÊÀ¸·Î ¿òÁ÷ÀÔ½Ã´Ù!
				if (region_rl == GREEN)
					state = 6;
				else if (region_rr == GREEN)
					state = 2;
				else // ÀÀ? ¿©±ä ¾îµð?
					state = 3;
			}
			else if (left_rho != -1234 && right_rho == -1234) { // ¿ÞÂÊ ¼±
																//¼±ÀÇ ¿ÞÂÊ Á¶»ç
				region_ll = sideLookupColor(surf, left_rho, 1);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, left_rho, 1, 0);
#endif
				//¼±ÀÇ ¿À¸¥ÂÊ Á¶»ç
				region_lr = sideLookupColor(surf, left_rho, 0);
#ifdef lcd_dbg
				sideRegionCheck(tmp_surf, left_rho, 0, 1);
#endif

				// GREEN »ö»óÀÌ ÀÖ´Â ÂÊÀ¸·Î ¿òÁ÷ÀÔ½Ã´Ù!
				if (region_ll == GREEN)
					state = 6;
				else if (region_lr == GREEN)
					state = 2;
				else // ³­ ´©±¸?¤Ð
					state = 5;
			}
			// ¼±ÀÌ ÇÏ³ªµµ ¾ø´Ù.
			else {
				// Á¤¸» ¹º°¡ Àß¸øµÇ¾ú´Ù. ¿©±â°¡ ¾îµòÁö ¸ð¸£°Ú´Ù¤Ð¤Ð
				state = -1;
			}

#ifdef lcd_dbg
			printf("region state (ll / lr / rl / rr) : %2d / %2d / %2d / %2d \n", region_ll, region_lr, region_rl, region_rr);
			draw_fpga_video_data_full(tmp_surf);
			flip();
			free(tmp_surf);
#endif
		}

		// Â¡°Ë´Ù¸® À§ÀÏ °æ¿ì
		else {
			// ¼±ÀÌ ¾çÂÊ´Ù ÀÖ´Ù.
			if (left_rho != -1234 && right_rho != -1234) {
				// threshold¿¡ ¸ÂÃç ¸ð¼Ç °áÁ¤
				if (right_rho < 22)
					state = 5;
				else if (right_rho < 52)
					state = 4;
				else
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ª ¹Û¿¡ ¾ø´Ù.
			else if (left_rho == -1234 && right_rho != -1234) { // ¿À¸¥ÂÊ ¼±
																// ¹º°¡ Àß¸øµÇ¾ú´Ù. ±Ùµ¥ ±×³É ÇØº¸ÀÚ ^^. ¿À¸¥ÂÊÀº ÀâÈ÷´Ï±î ...
				if (right_rho < 22)
					state = 5;
				else if (right_rho < 52)
					state = 4;
				else
					state = 3;
			}
			else if (left_rho != -1234 && right_rho == -1234) { // ¿ÞÂÊ ¼±
																// ¹º°¡ Àß¸ø‰ç?¤·¤·, ÀÏ´Ü ¿ìÇâ¿ì. ¿ÞÂÊÀ¸·Î ¸ÂÃçº¼±î?;;
																//state = 0;
				if (left_rho < -65)
					state = 5;
				else if (left_rho < -40)
					state = 4;
				else
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ªµµ ¾ø´Ù.
			else {
				// ¹º°¡ Àß¸øµÇ¾ú´Ù. ÀÏ´Ü ¿ìÇâ¿ì.
				state = 0;
			}
		}
		*/
		if (onBridge) {
			// ¼±ÀÌ ¾çÂÊ´Ù ÀÖ´Ù.
			if (left_rho != -1234 && right_rho != -1234) {
				// threshold¿¡ ¸ÂÃç ¸ð¼Ç °áÁ¤
				if (right_rho < 25)
					state = 5;
				else if (right_rho < 52)
					state = 4;
				else
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ª ¹Û¿¡ ¾ø´Ù.
			else if (left_rho == -1234 && right_rho != -1234) { // ¿À¸¥ÂÊ ¼±
				// ¹º°¡ Àß¸øµÇ¾ú´Ù. ±Ùµ¥ ±×³É ÇØº¸ÀÚ ^^. ¿À¸¥ÂÊÀº ÀâÈ÷´Ï±î ...
				if (right_rho < 35)
					state = 5;
				else if (right_rho < 65)
					state = 4;
				else
					state = 3;
			}
			else if (left_rho != -1234 && right_rho == -1234) { // ¿ÞÂÊ ¼±
																// ¹º°¡ Àß¸ø‰ç?¤·¤·, ÀÏ´Ü ¿ìÇâ¿ì. ¿ÞÂÊÀ¸·Î ¸ÂÃçº¼±î?;;
																//state = 0;
				if (left_rho < -62)
					state = 5;
				else if (left_rho < -22)
					state = 4;
				else
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ªµµ ¾ø´Ù.
			else {
				// ¹º°¡ Àß¸øµÇ¾ú´Ù. ÀÏ´Ü ¿ìÇâ¿ì.
				state = 0;
			}
		}
		else {
			// ¼±ÀÌ ¾çÂÊ´Ù ÀÖ´Ù.
			if (left_rho != -1234 && right_rho != -1234) {
				// threshold¿¡ ¸ÂÃç ¸ð¼Ç °áÁ¤
				if (right_rho < 25)
					state = 5;
				else if (right_rho < 52)
					state = 4;
				else
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ª ¹Û¿¡ ¾ø´Ù.
			else if (left_rho == -1234 && right_rho != -1234) { // ¿À¸¥ÂÊ ¼±
																// ¹º°¡ Àß¸øµÇ¾ú´Ù. ±Ùµ¥ ±×³É ÇØº¸ÀÚ ^^. ¿À¸¥ÂÊÀº ÀâÈ÷´Ï±î ...
				if (right_rho < 25)
					state = 5;
				else if (right_rho < 52)
					state = 4;
				else
					state = 3;
			}
			else if (left_rho != -1234 && right_rho == -1234) { // ¿ÞÂÊ ¼±
																// ¹º°¡ Àß¸ø‰ç?¤·¤·, ÀÏ´Ü ¿ìÇâ¿ì. ¿ÞÂÊÀ¸·Î ¸ÂÃçº¼±î?;;
																//state = 0;
				if (left_rho < -62)
					state = 5;
				else if (left_rho < -17)
					state = 4;
				else
					state = 3;
			}
			// ¼±ÀÌ ÇÏ³ªµµ ¾ø´Ù.
			else {
				// ¹º°¡ Àß¸øµÇ¾ú´Ù. ÀÏ´Ü ¿ìÇâ¿ì.
				state = 0;
			}
		}
	}
	else if (cnt_max_idx == 3) { // skewed right
		state = 0;
	}

	free(htda);
	free(surf);
	free(fvda);

	if (log) {
		fprintf(log, "[LOG] peek_vert \n>> result table\n");
		fprintf(log, "left|");
		for (n = 0; n < n_idx_ptr_left; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "rho |");
		for (n = 0; n < n_idx_ptr_left; n++)
			fprintf(log, " %4d  |", arr_left[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "rght|");
		for (n = 0; n < n_idx_ptr_right; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "rho |");
		for (n = 0; n < n_idx_ptr_right; n++)
			fprintf(log, " %4d  |", arr_right[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "cnt |");
		for (n = 0; n < 4; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "vot |");
		for (n = 0; n < 4; n++)
			fprintf(log, " %4d  |", cnt[n]);
		fprintf(log, "\n");
		fprintf(log, "=================================\n");
	}

	return state;
}

/*
int peek_green(FILE* log){ //초록색 카운트

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

		for(py=0;py<120;py++){
		for(px=0;px<180;px++){

		val = (U8)(0x000f & mysurf[py][px]);
		hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
		sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);

			if(val>3){
				if((hue > 70 && hue <= 100) && (sat > 1)){
					pala[py][px] = 0x0000;   //not green
				}
				else{

					pala[py][px] = 0xffff;
					cnt_jh++;   //white
			}
			}
		}
	}

}*/




// bridge¿¡¼­ ³»·Á¿Ã ¶§, »óÅÂ¸¦ °áÁ¤ÇÏ´Â ÇÔ¼ö.
int peek_bdgland(FILE* log) {
	int n, m, htda_num;
	int rho_top, pat, flg, tmp, state = 0;

	int max, max_pat;
	int cnt[4] = { 0 };
	int arr[smp_frame] = { 0 };
	int n_arr = 0;
	int midrho = -1234;

	int log_flag = 0;
	int cal_h = -1234;
	int cal_s = -1234;
	int cal_v = -1234;
	COLOR color_test = NONE;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	for (n = 0; n < smp_frame; n++) {

		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);

		flg = chop_hrizon(htda, htda_num, &pat, &rho_top, bot_porking, default_obs); // top ±×·ìÀÇ rho avg¸¦ ¾Ë¾Æ³½´Ù.
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (pat) {
			case 0:
				cnt[1]++;
				break;
			case 1:	// insert score into arr.
				if (rho_top < 0)
					rho_top = -rho_top;

				arr[n_arr++] = rho_top;
				cnt[2]++;
				break;
			case 2:
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	// first, cnt Á¶»ç
	// °¡Àå Å« °ªÀ» °¡Áö´Â cnt?
	max_pat = 0;
	max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > max) {
			max_pat = n;
			max = cnt[n];
		}
	// second, arr Á¤·Ä
	if (max_pat == 0) {
		// ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
		state = -1;
	}
	else if (max_pat == 1) {
		// skewed left
		state = 0;
	}
	else if (max_pat == 2) {
		// flat line
		// sort

		for (n = (n_arr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (arr[m] < arr[m + 1]) {
					tmp = arr[m];
					arr[m] = arr[m + 1];
					arr[m + 1] = tmp;
				}

		// midrho¿¡ Áß¾Ó°ª ÀúÀå
		midrho = arr[(int)(n_arr / 2)];

		if (midrho > 119)
			midrho = 119;

		if (midrho > 93) {
			log_flag = 1;
			cal_h = 0;
			cal_s = 0;
			cal_v = 0;
			get_surf(surf, fvda);

			state = 4;
		}
		else {
			state = 3;
		}
	}
	else if (max_pat == 3) { // skewed right
		state = 1;
	}

	free(htda);
	free(surf);
	free(fvda);

	if (log) {
		fprintf(log, "[LOG] peek_bdgland \n>> result table\n");
		fprintf(log, "pat |");
		for (n = 0; n < 4; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "vot |");
		for (n = 0; n < 4; n++)
			fprintf(log, " %4d  |", cnt[n]);
		fprintf(log, "\n");
		fprintf(log, "--------------------------------\n");
		fprintf(log, "arr |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " [%3d] |", n);
		fprintf(log, "\n");
		fprintf(log, "rho |");
		for (n = 0; n < n_arr; n++)
			fprintf(log, " %4d  |", arr[n]);
		fprintf(log, "\n");
		fprintf(log, "top set : %4d \n", midrho);
		if (log_flag) {
			fprintf(log, "cal_h = %4d / cal_s = %4d / cal_v = %4d \n", cal_h, cal_s, cal_v);
			fprintf(log, "color_test = %3d\n", color_test);
			fprintf(log, "RED   : 0\n");
			fprintf(log, "BLUE  : 1\n");
			fprintf(log, "YELLOW: 2\n");
			fprintf(log, "ORANGE: 3\n");
			fprintf(log, "WHILE : 4\n");
			fprintf(log, "NONE  : 5\n");
		}
		fprintf(log, "=================================\n");
	}

	// state [-1] : pattern not appear
	// state [ 0] : skewed left (\) 
	// state [ 1] : skewed right (/)
	// state [ 3] : far
	// state [ 4] : fitted

	return state;
}
//

// golf functions
int ORANGE_Golf_HS(FILE* log, int h, int s, U16* output_image, int *y_length)
{
	int i, j, k;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);

	U8* H = (U8*)malloc(pl);
	U8* S = (U8*)malloc(pl);
	U8* V = (U8*)malloc(pl);

	_wait_d;
	read_fpga_video_data(fvda);

	memset(HSV_data, 0, pl * 2);
	memset(H, 0, pl);
	memset(S, 0, pl);
	memset(V, 0, pl);

	//memory setting
	memcpy(HSV_data, fvda, pl * 2);

	HSV2EACH(HSV_data, H, S, V, pl);

	memset(output_image, 0, pl * 2);

	U8* HS_inter = (U8*)malloc(pl);
	U8* S_new = (U8*)malloc(pl);

	memset(HS_inter, 0, pl);
	memset(S_new, 0, pl);

	int x1[5] = { 0 }, y1[5] = { 0 };

	//fill S_new and HS_inter
	for (j = 1; j < 119; j++)
	{
		for (i = 1; i < 179; i++)
		{
			if (S[iw*j + i] >= s && abs(H[iw*j + i] - 8) <= h && V[iw*j + i] > 2)
			{
				S_new[iw*j + i] = 0xff;
				HS_inter[iw*j + i] = 0xff;
				output_image[iw*j + i] = 0xffff;
			}
		}
	}

	//find bottom points of golf ball
	for (j = 118; j >0; j--)
	{
		for (i = 1; i < 179; i++)
		{
			//find first bottom point of orange golf ball from HS_inter 
			if (x1[0] == 0)
			{
				if (j > 4)
				{
					if (((HS_inter[iw*j + i] == 0xff) && (HS_inter[iw*(j - 1) + i] == 0xff) && (HS_inter[iw*(j - 2) + i] == 0xff) && (HS_inter[iw*(j - 3) + i] == 0xff)) ||
						((HS_inter[iw*j + i] == 0xff) && (HS_inter[iw*(j - 1) + i] == 0xff) && (HS_inter[iw*(j - 2) + i] == 0xff) && (HS_inter[iw*(j - 4) + i] == 0xff)) ||
						((HS_inter[iw*j + i] == 0xff) && (HS_inter[iw*(j - 1) + i] == 0xff) && (HS_inter[iw*(j - 3) + i] == 0xff) && (HS_inter[iw*(j - 4) + i] == 0xff)) ||
						((HS_inter[iw*j + i] == 0xff) && (HS_inter[iw*(j - 2) + i] == 0xff) && (HS_inter[iw*(j - 3) + i] == 0xff) && (HS_inter[iw*(j - 4) + i] == 0xff)))
					{
						x1[0] = i;
						y1[0] = j;
					}
				}
				else
				{
					x1[0] = i;
					y1[0] = j;
				}
			}
			//find second bottom point of orange golf ball from S_new 
			if ((x1[0] != 0) && (j == y1[0]))
			{
				if (i <176)
				{
					if ((S_new[iw*j + i] == 0xff) && (S_new[iw*j + i + 1] == 0x00) && (S_new[iw*j + i + 2] == 0x00) && (S_new[iw*j + i + 3] == 0x00))
					{
						x1[1] = i;
						y1[1] = j;
					}
				}
				else if (x1[1] == 0)
				{
					if ((S_new[iw*j + i] == 0xff) && (S_new[iw*j + i - 1] == 0xff) && (S_new[iw*j + i - 2] == 0xff) && (S_new[iw*j + i - 3] == 0xff))
					{
						x1[1] = i;
						y1[1] = j;
					}
				}
			}
			if ((x1[0] != 0) && (x1[1] != 0))
				break;
		}
		if ((x1[0] != 0) && (x1[1] != 0))
			break;
	}

	//find top points of golf ball
	for (j = y1[0] - 1; j > 0; j--)
	{
		for (i = x1[0]; i < x1[1] + 1; i++)
		{
			if (x1[2] == 0)
			{
				if (j > 4)
				{
					if ((S_new[iw*j + i] == 0xff) && (S_new[iw*(j - 1) + i] == 0x00) && (S_new[iw*(j - 2) + i] == 0x00) && (S_new[iw*(j - 3) + i] == 0x00))
					{
						x1[2] = i;
						y1[2] = j;
					}
				}
				else
				{
					if (((S_new[iw*j + i] == 0xff) && (S_new[iw*(j + 1) + i] == 0xff) && (S_new[iw*(j + 2) + i] == 0xff) && (S_new[iw*(j + 3) + i] == 0xff)) ||
						((S_new[iw*j + i] == 0xff) && (S_new[iw*(j + 1) + i] == 0xff) && (S_new[iw*(j + 2) + i] == 0xff) && (S_new[iw*(j + 4) + i] == 0xff)) ||
						((S_new[iw*j + i] == 0xff) && (S_new[iw*(j + 1) + i] == 0xff) && (S_new[iw*(j + 3) + i] == 0xff) && (S_new[iw*(j + 4) + i] == 0xff)) ||
						((S_new[iw*j + i] == 0xff) && (S_new[iw*(j + 2) + i] == 0xff) && (S_new[iw*(j + 3) + i] == 0xff) && (S_new[iw*(j + 4) + i] == 0xff)))
					{
						x1[2] = i;
						y1[2] = j;
					}
				}
			}
			else
				break;
		}
	}

	int min_o;
	int max_o;
	int radius_o;
	int ch1, ch2;
	// ¿©±â!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@
	min_o = x1[0];
	max_o = x1[1];
	radius_o = y1[0] - y1[2] + 6;

	for (k = y1[2]; k < y1[0] + 1; k++)
	{
		ch1 = 0;
		ch2 = 0;
		//find most left point
		for (i = x1[0]; i>x1[0] - radius_o - 1; i--)
		{
			//if i is smaller then min_o
			if (i <= min_o)
			{
				if ((i > 3) && (ch1 == 0))
				{
					if ((S_new[iw*k + i] == 0xff) && (S_new[iw*k + i - 1] == 0x00) && (S_new[iw*k + i - 2] == 0x00) && (S_new[iw*k + i - 3] == 0x00))
					{
						min_o = i;
						x1[3] = i;
						y1[3] = k;
						ch1 = 1;
					}
				}
				else if (ch1 == 0)	//when it is close to left end
				{
					if ((S_new[iw*k + i] == 0xff) && (S_new[iw*k + i + 1] == 0xff) && (S_new[iw*k + i + 2] == 0xff) && (S_new[iw*k + i + 3] == 0xff))
					{
						int m, allow1 = 0;
						for (m = x1[0] - 1; m >= i; m--)
						{
							if (S_new[iw*k + m] == 0xff)
								allow1 = 0;
							else
								allow1++;

							if (allow1 >= 3)
								break;
						}
						if (allow1 >= 3)
							break;
						else
						{
							min_o = i;
							x1[3] = i;
							y1[3] = k;
							ch1 = 1;
						}
					}
				}
			}
		}

		//find most right point
		for (j = x1[1]; j<x1[1] + radius_o + 1; j++)
		{
			if (j >= max_o)
			{
				if ((j < 176) && (ch2 == 0))
				{
					if ((S_new[iw*k + j] == 0xff) && (S_new[iw*k + j + 1] == 0x00) && (S_new[iw*k + j + 2] == 0x00) && (S_new[iw*k + j + 3] == 0x00))
					{
						max_o = j;
						x1[4] = j;
						y1[4] = k;
						ch2 = 1;
					}
				}
				else if (ch2 == 0)
				{
					if ((S_new[iw*k + j] == 0xff) && (S_new[iw*k + j - 1] == 0xff) && (S_new[iw*k + j - 2] == 0xff) && (S_new[iw*k + j - 3] == 0xff))
					{
						int n, allow2 = 0;
						for (n = x1[1] + 1; n <= j; n--)
						{
							if (S_new[iw*k + n] == 0xff)
								allow2 = 0;
							else
								allow2++;

							if (allow2 >= 3)
								break;
						}
						if (allow2 >= 3)
							break;
						else
						{
							max_o = j;
							x1[4] = j;
							y1[4] = k;
							ch2 = 1;
						}
					}
				}
			}
		}
	}

	//color the screen~
#if 1
	int color = 0;

	for (i = 0; i < 5; i++)
	{
		//color = (i <= 1) ? ((i == 0) ? 0xf800 : 0xffe0) : ((i == 2) ? 0x07e0:0x001f);
		switch (i) {
		case 0:					//red
			color = 0xf800;
			break;
		case 1:					//yellow
			color = 0xffe0;
			break;
		case 2:					//green
			color = 0x07e0;
			break;
		case 3:					//blue
			color = 0x001f;
			break;
		case 4:					//
			color = 0x8700;
			break;
		}

		output_image[iw*y1[i] + x1[i]] = color;
		if (y1[i] > 1)
			output_image[iw*(y1[i] - 1) + x1[i]] = color;
		if (y1[i] < 118)
			output_image[iw*(y1[i] + 1) + x1[i]] = color;
		if (x1[i] < 178)
		{
			output_image[iw*y1[i] + x1[i] + 1] = color;
			if (y1[i] > 1)
				output_image[iw*(y1[i] - 1) + x1[i] + 1] = color;
			if (y1[i] < 118)
				output_image[iw*(y1[i] + 1) + x1[i] + 1] = color;
		}
		if (x1[i] > 1)
		{
			output_image[iw*y1[i] + x1[i] - 1] = color;
			if (y1[i] > 1)
				output_image[iw*(y1[i] - 1) + x1[i] - 1] = color;
			if (y1[i] < 118)
				output_image[iw*(y1[i] + 1) + x1[i] - 1] = color;
		}
	}
#endif

#if 1
	for (j = 1; j < 119; j++)
	{
		output_image[iw*j + x1[3]] = 0xffe0;
		output_image[iw*j + x1[4]] = 0xffe0;
		output_image[iw*j + (x1[4] + x1[3]) / 2] = 0x07e0;
		output_image[iw*j + 90] = 0x00ff; //boundary
		output_image[iw*j + 97] = 0x00ff;
	}

	for (i = 1; i < 179; i++)
	{
		output_image[iw*y1[2] + i] = 0xffe0;
		output_image[iw*y1[0] + i] = 0xffe0;
		output_image[iw*(y1[0] + y1[2]) / 2 + i] = 0x07e0;
		output_image[iw*(120-20) + i] = 0xff;//boundary
	}
	output_image[iw*(y1[0] + y1[2]) / 2 + (x1[4] + x1[3]) / 2] = 0xf800;

	if (log) {
		fprintf(log, "******************************************************************\n");
		fprintf(log, "x[0]=%d\ty[0]=%d\tx[1]=%d\ty[1]=%d\n", x1[0], y1[0], x1[1], y1[1]);
		fprintf(log, "x[3]=%d\ty[3]=%d\tx[4]=%d\ty[4]=%d\n", x1[3], y1[3], x1[4], y1[4]);
		fprintf(log, "min_x=%d\t max_x=%d\n", x1[3], x1[4]);
		fprintf(log, "min_y=%d\t max_y=%d\n", y1[2], y1[0]);
		fprintf(log, "center point xy=( %d , %d ) \n", (x1[4] + x1[3]) / 2, (y1[0] + y1[2]) / 2);
		fprintf(log, "******************************************************************\n");
	}
#endif

	*y_length = (120 - (y1[0] + y1[2]) / 2 );

	if (log) {
		fprintf(log, "[x, y]=( %d, %d)\n", (x1[4] + x1[3]) / 2, 120 - (y1[0] + y1[2]) / 2);
	}

	free(S_new);
	free(HS_inter);
	free(V);
	free(S);
	free(H);
	free(HSV_data);
	free(fvda);

	return (x1[4] + x1[3]) / 2;

}

void ORANGE_Golf_HS_ver2(FILE* log, int *y_length)
{
	int cnt, n, px, py;

	U8 hue, sat, val;
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));	// 22000 * 2 bytes
	U16* surf = (U16*)malloc(sizeof(U16)*pl);			// surf 21600 * 2 bytes
	U16* pala_mem = (U16*)malloc(sizeof(U16)*pl);

	U16** mysurf = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		mysurf[n] = &surf[n * 180];

	U16** pala = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		pala[n] = &pala_mem[n * 180];
	direct_camera_display_off;
	Motion_SOC(mine_std);
	while(1){
		
		read_fpga_video_data(fvda);
		get_surf(surf, fvda);
		memset(pala_mem, 0, sizeof(U16)*pl);
		int fcnt = 0;


		for (py = 0; py < 120; py++) {
			for (px = 0; px < 180; px++) {
				hue = (U8)((0xff00 & mysurf[py][px]) >> 8);
				sat = (U8)((0x00f0 & mysurf[py][px]) >> 4);
				val = (U8)(0x000f & mysurf[py][px]);

				if ((hue > 0) && (hue < 40)		// Orange
					&& (sat > 2) && (val > 2)) { 
					pala[py][px] = 0xffff;
				}
				
			}
		}

		clear_screen();
		draw_fpga_video_data_full(pala_mem);
		flip();
	}
	
	direct_camera_display_on;


	free(fvda);
	free(surf);
	free(pala_mem);
	free(pala);
	free(mysurf);
}

int MV_F_BigStep_golf(FILE* log, int dist)
{
	int move;
	move = dist;
	if(0<=move && move <20){
		fprintf(log, "\nsection 0\n");
		return 1;
	}
	else if(move < 60){
		Motion_SOC(head_2cm);
		Motion_SOC(walk_1);
		fprintf(log, "\nsection 2@@@@@@@@@|@@@@@@@@@|\n");
	}	
	else{
		// Motion_SOC(h) 20190513 주석처리
		Motion_SOC(head_2cm);
		Motion_SOC(walk_2);
		fprintf(log, "\ntoo far\n");
	}

	return 0;
}

int MV_LR_BigStep_golf(FILE* log, int dist)
{
	int key;
	int cnt = 0;
	if (dist < 40) {
		Motion_SOC(sl_4);
		Motion_SOC(head_2cm);
		fprintf(log, ">> no ball.\n");
		key = 2;
	}
	else if (dist < 80) {
		Motion_SOC(sl_4);
		Motion_SOC(head_2cm);
		fprintf(log, ">> move left fast.\n");
		key = -1;
	}
	// else if (dist >= 88 && dist < 90) {
	// 	Motion_SOC(sl_3);
	// 	Motion_SOC(head_2cm);
	// 	fprintf(log, ">> move left.\n");
	// 	key = 0;
	// }
	else if (dist >= 80 && dist <= 100) { // 90~97
		fprintf(log, "[LOG] mot_runBridge \n>> line is fitted.\n");
		key = 1;
	}
	else if (dist > 100 && dist <= 105) {
		Motion_SOC(sr_4);
		Motion_SOC(head_2cm);
		fprintf(log, ">> move right.\n");
		key = 0;
	}
	else if (dist > 105) {
		Motion_SOC(sr_4);
		Motion_SOC(head_2cm);
		fprintf(log, ">> move right fast.\n");
		key = -1;
	}
	return key;
}
void BLUE_Hole(U16* output, int* hole_length, int* hole_x, int* hole_y)
{
	int x_1 = 0;
	int x_2 = 0;
	int x_length;
	int max = 0;
	int i, j, k;
	int flg;
	//int k = 0;
	U8 hue, sat, val;
	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);
	
	U16** plane = (U16**)malloc(sizeof(U16*) * 120);
	for (i = 0; i < 120; i++)
		plane[i] = &HSV_data[i * 180];

	read_fpga_video_data(fvda);
	//int x__[200] = { 0 };
	//int y__[200] = { 0 };

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memset(output, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	for (i = 0; i < 120; i++) {
		for (j = 0; j < 180; j++) {
			hue = (U8)((0xff00 & plane[i][j]) >> 8);
			sat = (U8)((0x00f0 & plane[i][j]) >> 4);
			val = (U8)(0x000f & plane[i][j]);
			if (hue > 150 && hue < 190 && sat > 3 && val > 3)
				input_image[i * 180 + j] = 0xffff;
		}
	}

	memcpy(output, input_image, pl * 2);

	//Hole_HS(HSV_data, input_image);
	for (j = 1; j < 119; j++)
	{
		flg = 0;
		for (i = 1; i < 90; i++)
		{
			if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i + 1] == 0xffff))
			{
				output[iw*j + i] = 0xf800;
				x_1 = i;
				flg = 1;
				for (i = 179; i > x_1; i--)
				{
					if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i - 1] == 0xffff))
					{
						output[iw*j + i] = 0xf800;
						x_2 = i;
						flg = 2;
						break;
					}
				}
				break;
			}
		}
		if (flg != 2) {
			for (i = 179; i > 89; i--)
			{
				if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i - 1] == 0xffff))
				{
					output[iw*j + i] = 0xf800;
					x_2 = i;
					if(flg != 1)
					for (i = 1; i < x_1; i++)
					{
						if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i + 1] == 0xffff))
						{
							output[iw*j + i] = 0xf800;
							x_1 = i;
							break;
						}
					}
					break;
				}
			}
		}
		x_length = x_2 - x_1;
		if (x_length <= 40) {
			if (max < x_length)
			{
				max = x_length;
				*hole_y = j;
				*hole_length = max;
				*hole_x = x_1;
			}
		}
	}

	#if 1
		for (k = 1; k < 119; k++)
		{
			output[iw*k + x_1] = 0xffe0;
			output[iw*k + x_2] = 0xffe0;
			output[iw*k + (x_1 + x_2) / 2] = 0x07e0;
			output[iw*k + 30] = 0x00ff; //boundary
			output[iw*k + 150] = 0x00ff;
		}

		for (i = 1; i < 179; i++)
		{
			output[iw*j] = 0xffe0;
		}

	#endif

	free(plane);
	free(HSV_data);
	free(input_image);
	free(fvda);
}
void HSV2EACH(U16* HSV_data, U8* H_data, U8* S_data, U8* V_data, int n)
{
	static int i;

	for (i = 0; i < n; i++)
	{
		H_data[i] = (HSV_data[i] >> 8) & 0xff;
		S_data[i] = (HSV_data[i] >> 4) & 0x0f;
		V_data[i] = (HSV_data[i]) & 0x0f;
	}
}
/*
int colorline(FILE* log, int rho, COLOR usr_color, int thr, int is_up) {
	int cnt, n;
	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* surf = (U16*)malloc(sizeof(U16)*pl);
	U16* o_surf;
	U16** img;

	read_fpga_video_data(fvda);
	get_surf(surf, fvda);
	free(fvda);

	o_surf = (U16*)malloc(sizeof(U16)*pl);
	cnt = colorRegion(o_surf, surf, usr_color);
	free(surf);

	if (is_up) {
		
	}
	else {

	}

	clear_screen();
	draw_fpga_video_data_full(o_surf);
	flip();
	free(o_surf);

	if (log) {
		fprintf(log, "[LOG] colorDrill \n");
		fprintf(log, ">> (cnt, %6d)\n", cnt);
	}

	if (cnt < thr)
		return 0;

	return 1;
}
*/

//jj
#if 1

int bndypeek(void) {
	int n, m, htda_num;
	int scr, grp, flg, tmp, state = 0;

	int cnt_max, cnt_max_idx;
	int cnt[4] = { 0 };
	int arr[smp_frame] = { 0 };
	int n_idx_ptr = 0;
	int mid_rho;

	U16* fvda = (U16*)malloc(sizeof(U16)*(pl + hl));
	U16* htda = (U16*)malloc(sizeof(U16)*hl);

	for (n = 0; n < smp_frame; n++) {

		read_fpga_video_data(fvda);
		htda_num = get_htda(htda, fvda);

		// state [-1] : pattern not appear
		// state [ 0] : skewed left (\) 
		// state [ 1] : skewed right (/)
		// state [ 2] : line too far
		// state [ 3] : line far
		// state [ 4] : line fitted
		// state [ 5] : line close
		// state [ 6] : line too close

		flg = hzt_map(htda, htda_num, &grp, &scr);
		if (flg < 0) {
			cnt[0]++;
		}
		else {
			switch (grp) {
			case 0:
				cnt[1]++;
				break;
			case 1:   // insert score into arr.
				arr[n_idx_ptr++] = scr;
				cnt[2]++;
				break;
			case 2:
				cnt[3]++;
				break;
			default:
				break;
			}
		}
	}

	// first, cnt Á¶»ç
	// °¡Àå Å« °ªÀ» °¡Áö´Â cnt?
	cnt_max_idx = 0;
	cnt_max = 0;
	for (n = 0; n < 4; n++)
		if (cnt[n] > cnt_max) {
			cnt_max_idx = n;
			cnt_max = cnt[n];
		}
	// second, arr Á¤·Ä
	// Áß°£°ªÀ¸·Î ÆÇ´Ü.
	if (cnt_max_idx == 0) {   // ¾î¶² ÆÐÅÏµµ °¨ÁöµÇÁö ¾ÊÀ½. (htda µ¥ÀÌÅÍ ¾øÀ½)
		state = -1;
	}
	else if (cnt_max_idx == 1) { // skewed left
		state = 0;
	}
	else if (cnt_max_idx == 2) { // flat line
								 // sort
		for (n = (n_idx_ptr - 1); n > 0; n--)
			for (m = 0; m < n; m++)
				if (arr[m] < arr[m + 1]) {
					tmp = arr[m];
					arr[m] = arr[m + 1];
					arr[m + 1] = tmp;
				}
		// mid_rho¿¡ Áß°£°ª ÀúÀå
		mid_rho = arr[(int)(n_idx_ptr / 2)];
		// Áß°£°ªÀ¸·Î state ÆÇ´Ü.
		if (mid_rho < 38)
			state = 2;
		else if (mid_rho < 42)
			state = 3;
		else if (mid_rho < 52)
			state = 4;
		else if (mid_rho < 60)
			state = 5;
		else
			state = 6;
	}
	else if (cnt_max_idx == 3) { // skewed right
		state = 1;
	}


	free(htda);
	free(fvda);

#ifdef show_bndy_vote_res
	printf("=================================\n");
	printf("   show_bndy_vote_res dbg message\n");
	printf("=================================\n");
	printf("arr |");
	for (n = 0; n < n_idx_ptr; n++)
		printf(" [%3d] |", n);
	printf("\n");
	printf("rho |");
	for (n = 0; n < n_idx_ptr; n++)
		printf(" %4d  |", arr[n]);
	printf("\n");
	printf("--------------------------------\n");
	printf("cnt |");
	for (n = 0; n < 4; n++)
		printf(" [%3d] |", n);
	printf("\n");
	printf("vot |");
	for (n = 0; n < 4; n++)
		printf(" %4d  |", cnt[n]);
	printf("\n");
	printf("=================================\n");
#endif

	return state;
}

int hzt_map(U16* htda, int htda_num, int* st, int* score) {
	int n, m;

	/*
	idx group
	0      : skewed-right lines (\)
	1      : flat lines (-)
	2      : skewed-left lines (/)
	*/

	/*
	return -1 : ¾î¶² ÆÐÅÏµµ °ËÃâµÇÁö ¾Ê¾ÒÀ» ¶§. htda°¡ ¾ø´Ù¸é.
	return 0  : ÆÐÅÏ °ËÃâµÊ.

	*st : idx groupÀÇ ³Ñ¹ö Àü´Þ.
	*score : idx 1 ÀÏ ¶§, Æò±Õ rho °ª. idx 1ÀÏ ¶§¸¸ ÀÇ¹ÌÀÖÀ½.

	*/

	int vot[hzt_stage] = { 0 };
	int idx[hzt_stage];

	for (n = 0; n < hzt_stage; n++)
		idx[n] = n;

	int t_vot;
	int t_idx;

	int sum;

#ifdef show_htda_dat
	printf("=================================\n");
	printf("   show_htda_dat dbg message\n");
	printf("=================================\n");
#endif

	sum = 0;
	for (n = 0; n < htda_num; n++) {
		char rho = ((char)(0x00ff & htda[n])) * 2;
		U8 theta = (U8)((0xff00 & htda[n]) >> 8);


		if (rho < 0)
			rho = -rho;


#ifdef show_htda_dat
		printf("[%3d] rho/theta : %5d / %5d\n", n, rho, theta);
#endif

		if (theta >= 125 && theta < 188) {
			vot[0]++;
		}
		else if (theta >= 6 && theta < 80) {
			vot[hzt_stage - 1]++;
		}
		else if (theta >= 188 || theta < 6) {
			sum += rho;
			vot[1]++;
		}
	}

#ifdef show_hzt_idx2
	printf("=================================\n");
	printf("   show_hzt_idx dbg message\n");
	printf("=================================\n");
	printf("idx |");
	for (n = 0; n < hzt_stage; n++)
		printf(" [%3d] |", idx[n]);
	printf("\n");
	printf("vot |");
	for (n = 0; n < hzt_stage; n++)
		printf(" %4d  |", vot[n]);
	printf("\n");
	printf("=================================\n");
#endif

	/* calc score */
	if (vot[1] > 0)
		*score = (int)sum / vot[1];
	else
		*score = -1;

	/* sort */

	for (n = (hzt_stage - 1); n > 0; n--) {
		for (m = 0; m < n; m++) {
			if (vot[m] < vot[m + 1]) {
				t_vot = vot[m];
				t_idx = idx[m];

				vot[m] = vot[m + 1];
				idx[m] = idx[m + 1];

				vot[m + 1] = t_vot;
				idx[m + 1] = t_idx;
			}
		}
	}

	if (vot[0] == 0)
		return -1;   // no pattern

	*st = idx[0];

#ifdef show_htda_dat
	printf("=================================\n");
#endif

#ifdef show_hzt_idx
	printf("=================================\n");
	printf("   show_hzt_idx dbg message\n");
	printf("=================================\n");
	printf("no. |");
	for (n = 0; n < hzt_stage; n++)
		printf(" [%3d] |", n);
	printf("\n");
	printf("idx |");
	for (n = 0; n < hzt_stage; n++)
		printf(" [%3d] |", idx[n]);
	printf("\n");
	printf("vot |");
	for (n = 0; n < hzt_stage; n++)
		printf(" %4d  |", vot[n]);
	printf("\n");
	printf("=================================\n");
#endif

	return 0;
}

/*
int make_flat(void)
{
int flg, state;
Motion_SOC(head_std);
Motion_SOC(head_right);

flg = 1;
while (flg) {
direct_camera_display_off();
state = bndypeek();
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
*/

void DMZ_SV_new(U16* HSV, U16* output_image)
{
	int i;
	U8* S_data = (U8*)malloc(pl);
	U8* V_data = (U8*)malloc(pl);
	memset(S_data, 0, pl);
	memset(V_data, 0, pl);
	memset(output_image, 0, pl * 2);

	for (i = 0; i < pl; i++)
	{

		S_data[i] = (HSV[i] >> 4) & 0x0f;
		V_data[i] = (HSV[i]) & 0x0f;

		if (i % 180 <= 160)
		{
			if (i > 20879)										//4 line - ignore
			{
				output_image[i] = 0x0;
			}
			else if (i > 20340)
			{
				if ((S_data[i] < 5) && (V_data[i] < 1))		//7 line - 
					output_image[i] = 0xffff;
			}
			else if (i > 19800)
			{
				if ((S_data[i] < 5) && (V_data[i] < 1))		//10 line
					output_image[i] = 0xffff;
			}
			else if (i > 19080)
			{
				if ((S_data[i] < 8) && (V_data[i] < 3))		//14 line
					output_image[i] = 0xffff;
			}
			else if (i < 3600)
			{
				output_image[i] = 0x0;
			}
			else
			{
				if ((S_data[i] < 8) && (V_data[i] < 3))		//Find	 mines	8
					output_image[i] = 0xffff;
			}
		}

	}

	free(V_data);
	free(S_data);
}

//Á¦ÀÏ ±ÙÁ¢ÇÑ Áö·ÚÀÇ Áß½ÉÀÇ xÁÂÇ¥¿Í Á¦ÀÏ °¡±î¿î Á¡±îÁöÀÇ °Å¸®¸¦ Ã£´Â ÇÔ¼ö
int CLOSEONE(U16* output_image, int *center_x)
{
	int i, j;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);
	Motion_SOC(head_2cm);
	read_fpga_video_data(fvda);

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memset(output_image, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	DMZ_SV_new(HSV_data, input_image);

	int close_x[6] = { 0 };
	int close_y[3] = { 0 };
	int radius[3] = { 0 };
	int check = 0;

	memset(output_image, 0, pl * 2);
	memcpy(output_image, input_image, pl * 2);
	// Áö·ÚÀÇ ¾Æ·¡ point y ÁÂÇ¥ Ã£±â
	for (j = ROI_y_max; j > 0; j--)
	{
		check = 0;
		for (i = ROI_x_min; i < ROI_x_max; i++)
		{
			if ((input_image[iw*j + i] == 0xffff) && (!check))
			{
				if (((input_image[iw*(j - 1) + i] == 0xffff)
					&& (input_image[iw*(j - 2) + i] == 0xffff)
					&& (input_image[iw*(j - 3) + i] == 0xffff))
					|| ((input_image[iw*(j - 1) + i] == 0xffff)
						&& (input_image[iw*(j - 2) + i] == 0xffff)
						&& (input_image[iw*(j - 4) + i] == 0xffff))
					|| ((input_image[iw*(j - 2) + i] == 0xffff)
						&& (input_image[iw*(j - 3) + i] == 0xffff)
						&& (input_image[iw*(j - 4) + i] == 0xffff))
					|| ((input_image[iw*(j - 1) + i] == 0xffff)
						&& (input_image[iw*(j - 2) + i] == 0xffff)
						&& (input_image[iw*j + i + 1] == 0xffff)
						&& (input_image[iw*j + i + 2] == 0xffff)))
				{
					check = 1;
					close_y[0] = j;
					close_x[0] = i;
				}
			}
			else if ((check == 1)
				&& (input_image[iw*j + i] == 0xffff)
				&& (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0))
			{
				close_x[1] = i;
			}
			else if ((check == 1)
				&& (input_image[iw*j + i] == 0x00)
				&& (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0))
			{
				break;
			}
		}
		if (close_y[0] > 0)
			break;
	}

	close_x[2] = (close_x[0] + close_x[1]);

	//Á¦ÀÏ °¡±î¿î Áö·ÚÀÇ Á¦ÀÏ ¸Õ point(yÁÂÇ¥) Ã£±â
	//close_x[2]°¡ È¦¼ö¶ó¸é,
	if ((close_x[2] % 2) == 1)
	{	//close_x[0]!=close_x[1] ÀÏ ¶§
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0) && (input_image[iw*(j - 2) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j == 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0)
							&& (input_image[iw*j + close_x[2] + 1] == 0)
							&& (input_image[iw*(j - 1) + close_x[2]] == 0)
							&& (input_image[iw*(j - 1) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0)
							&& (input_image[iw*(j - 1) + close_x[2]] == 0)
							&& (input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
	}
	else
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
	}

	//´ë·«ÀûÀÎ ¹ÝÁö¸§ °è»ê
	radius[0] = (close_y[0] - close_y[1]) / 2 + 1;

	close_y[2] = (close_y[0] + close_y[1]) / 2;

	radius[1] = radius[0];
	radius[2] = radius[0];

	//calculate radius and find close_x[4]&close_x[5](which are x-coordinate of most side positioned pixels)
	//Á¦ÀÏ ¿ÞÂÊ xÁÂÇ¥
	for (i = close_x[2] - radius[0] - 1; i > 0; i--)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 1] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 2] != 0xffff))
		{
			close_x[3] = i + 1;
			break;
		}
		else
			radius[1]++;
	}
	//Á¦ÀÏ ¿À¸¥ÂÊ xÁÂÇ¥
	for (i = close_x[2] + radius[0] + 1; i < 180; i++)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 1] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 2] != 0xffff))
		{
			close_x[4] = i - 1;
			break;
		}
		else
			radius[2]++;
	}

	radius[0] = (radius[1] >= radius[2]) ? radius[1] : radius[2];

#if 1
	//draw lines
#if 1
	for (i = 1; i < 180; i++)
	{
		output_image[iw*close_y[0] + i] = 0xf800;		//red-first mine bottom
		output_image[iw*close_y[1] + i] = 0x07e0;		//green-first mine top
	}
#endif
	//draw yellow line-first mine
#if 1
	for (j = close_y[1]; j < close_y[0] + 1; j++)
	{
		for (i = close_x[2] - radius[0]; i< close_x[2] + radius[0] + 1; i++)
		{
			if (i>0 && i<180)
				if (j > 0 && j < 118)
				{
					if ((i == close_x[2] - radius[0]) || (i == close_x[2] + radius[0]))
						output_image[iw*j + i] = 0xffe0;
					else if ((j == close_y[1]) || (j == close_y[0]))
						output_image[iw*j + i] = 0xffe0;
				}
		}
	}
#endif
	//green point-first mine detected point/red point-second mine detected point
#if 1
	//first
	output_image[iw*close_y[0] + close_x[0]] = 0xf81f;
	if (close_y[0] > 1)
		output_image[iw*(close_y[0] - 1) + close_x[0]] = 0xf81f;
	if (close_y[0]<118)
		output_image[iw*(close_y[0] + 1) + close_x[0]] = 0xf81f;
	if (close_x[0] > 1)
	{
		output_image[iw*close_y[0] + close_x[0] - 1] = 0xf81f;
		if (close_y[0]>1)
			output_image[iw*(close_y[0] - 1) + close_x[0] - 1] = 0xf81f;
		if (close_y[0]<118)
			output_image[iw*(close_y[0] + 1) + close_x[0] - 1] = 0xf81f;
	}
	if (close_x[0] <179)
	{
		output_image[iw*close_y[0] + close_x[0] + 1] = 0xf81f;
		if (close_y[0]>1)
			output_image[iw*(close_y[0] - 1) + close_x[0] + 1] = 0xf81f;
		if (close_y[0]<118)
			output_image[iw*(close_y[0] + 1) + close_x[0] + 1] = 0xf81f;
	}

#endif
#endif

	*center_x = (close_x[3] + close_x[4]) / 2;
	close_y[0] = 120 - close_y[0];

	free(HSV_data);
	free(input_image);
	free(fvda);

	return close_y[0];
}

//¿µ»ó¿¡¼­ °¡Àå °¡±î¿î µÎ Áö·ÚÀÇ À§Ä¡¸¦ ÆÄ¶ó¹ÌÅÍ¿¡ ÀÔ·Â
void TWO_MINES_FINAL(int *center_x1, int *center_x2, int *y1, int *y2)
{
	int i, j;
	// Motion_SOC(head_2cm);  //revised 190114
	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);

	read_fpga_video_data(fvda);

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	DMZ_SV_new(HSV_data, input_image);

	//////////////////add 190115///////////////////
	clear_screen();
	draw_fpga_video_data_full(input_image);
	///////////////////////////////////////////////

	int close_x[6] = { 0 };
	int close_y[4] = { 0 };
	int radius[3] = { 0 };
	int check = 0;

	for (j = 118; j > 0; j--)
	{
		check = 0;
		for (i = ROI_x_min; i < ROI_x_max; i++)
		{
			if ((input_image[iw*j + i] == 0xffff) && (!check))
			{
				if (((input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff))
					|| ((input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 4) + i] == 0xffff))
					|| ((input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff) && (input_image[iw*(j - 4) + i] == 0xffff)))
				{
					check = 1;
					close_y[0] = j;
					close_x[0] = i;
				}
			}
			else if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0) && (check == 1))
			{
				close_x[1] = i;
			}
			else if ((input_image[iw*j + i] == 0x00) && (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0) && (check == 1))
			{
				break;
			}
		}
		if (close_y[0] > 0)
			break;
	}

	close_x[2] = (close_x[0] + close_x[1]);

	if ((close_x[2] % 2) == 1)
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0) && (input_image[iw*(j - 2) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j == 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
	}
	else
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
	}

	radius[0] = (close_y[0] - close_y[1]) / 2 + 1;
	close_y[2] = (close_y[0] + close_y[1]) / 2;

	radius[1] = radius[0];
	radius[2] = radius[0];
	//calculate radius and find close_x[4]&close_x[5](which are x-coordinate of most side positioned pixels)
#if 1
	for (i = close_x[2] - radius[0] - 1; i > 0; i--)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 1] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 2] != 0xffff))
		{
			close_x[4] = i + 1;
			break;
		}
		else
			radius[1]++;
	}

	for (i = close_x[2] + radius[0] + 1; i < 180; i++)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 1] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 2] != 0xffff))
		{
			close_x[5] = i - 1;
			break;
		}
		else
			radius[2]++;
	}

	//printf("radius[0]=%d\t radius[1]=%d\t radius[2]=%d\n", radius[0], radius[1], radius[2]);

	radius[0] = RADIUS;//(radius[1] >= radius[2]) ? radius[1] : radius[2];

					   //search for mine 2
	for (j = close_y[0]; j > 0; j--)
	{
		check = 0;
		if ((j <= close_y[2] + radius[0]) && (j >= close_y[2] - radius[0]))
		{
			for (i = BOUNDARY_L; i < BOUNDARY_R; i++)
			{
				if ((i <= close_x[2] + radius[0]) && (i >= close_x[2] - radius[0]))
					continue;
				else
				{
					if (((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff)) ||
						((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)) ||
						((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)))
					{
						close_y[3] = j;
						close_x[3] = i;
						break;
					}
				}
			}
		}
		else
		{
			for (i = BOUNDARY_L; i < BOUNDARY_R; i++)
			{
				if (((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff)) ||
					((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)) ||
					((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)))
				{
					close_y[3] = j;
					close_x[3] = i;
					break;
				}
			}
		}
		if (close_y[3] > 0)
			break;
	}

#endif 

	*center_x1 = (close_x[4] + close_x[5]) / 2;
	*center_x2 = close_x[3];
	*y1 = 120 - close_y[0];
	*y2 = 120 - close_y[3];

	free(HSV_data);
	free(input_image);
	free(fvda);
}

void TWO_MINES_FINAL2(int *center_x1, int *center_x2, int *y1, int *y2, int x_start, int x_end)
{
	int i, j;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);
	Motion_std(head_2cm);
	read_fpga_video_data(fvda);

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	DMZ_SV_new(HSV_data, input_image);

	int close_x[6] = { 0 };
	int close_y[4] = { 0 };
	int radius[3] = { 0 };
	int check = 0;

	for (j = 118; j > 0; j--)
	{
		check = 0;
		for (i = x_start; i < x_end; i++)
		{
			if ((input_image[iw*j + i] == 0xffff) && (!check))
			{
				if (((input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff))
					|| ((input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 4) + i] == 0xffff))
					|| ((input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff) && (input_image[iw*(j - 4) + i] == 0xffff)))
				{
					check = 1;
					close_y[0] = j;
					close_x[0] = i;
				}
			}
			else if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0) && (check == 1))
			{
				close_x[1] = i;
			}
			else if ((input_image[iw*j + i] == 0x00) && (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0) && (check == 1))
			{
				break;
			}
		}
		if (close_y[0] > 0)
			break;
	}

	close_x[2] = (close_x[0] + close_x[1]);

	if ((close_x[2] % 2) == 1)
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0) && (input_image[iw*(j - 2) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j == 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
	}
	else
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
	}

	radius[0] = (close_y[0] - close_y[1]) / 2 + 1;
	close_y[2] = (close_y[0] + close_y[1]) / 2;

	radius[1] = radius[0];
	radius[2] = radius[0];
	//calculate radius and find close_x[4]&close_x[5](which are x-coordinate of most side positioned pixels)
#if 1
	for (i = close_x[2] - radius[0] - 1; i > 0; i--)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 1] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 2] != 0xffff))
		{
			close_x[4] = i + 1;
			break;
		}
		else
			radius[1]++;
	}

	for (i = close_x[2] + radius[0] + 1; i < 180; i++)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 1] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 2] != 0xffff))
		{
			close_x[5] = i - 1;
			break;
		}
		else
			radius[2]++;
	}

	//printf("radius[0]=%d\t radius[1]=%d\t radius[2]=%d\n", radius[0], radius[1], radius[2]);

	radius[0] = RADIUS;//(radius[1] >= radius[2]) ? radius[1] : radius[2];

	for (j = close_y[0]; j > 0; j--)
	{
		check = 0;
		if ((j <= close_y[2] + radius[0]) && (j >= close_y[2] - radius[0]))
		{
			for (i = x_start; i < x_end; i++)
			{
				if ((i <= close_x[2] + radius[0]) && (i >= close_x[2] - radius[0]))
					continue;
				else
				{
					if (((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff)) ||
						((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)) ||
						((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)))
					{
						close_y[3] = j;
						close_x[3] = i;
						break;
					}
				}
			}
		}
		else
		{
			for (i = x_start; i < x_end; i++)
			{
				if (((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff)) ||
					((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)) ||
					((input_image[iw*j + i] == 0xffff) && (input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff)))
				{
					close_y[3] = j;
					close_x[3] = i;
					break;
				}
			}
		}
		if (close_y[3] > 0)
			break;
	}

#endif 

	*center_x1 = (close_x[4] + close_x[5]) / 2;
	*center_x2 = close_x[3];
	*y1 = 120 - close_y[0];
	*y2 = 120 - close_y[3];

	free(HSV_data);
	free(input_image);
	free(fvda);
}

//xÃà »ó¿¡¼­ Áß½É¿¡ ±ÙÁ¢ÇÑ ÁÂÇ¥µé¸¸ searchÇØ¼­ Áö·Ú Ã£±â 
void CHECKFRONTONE(int *center_x1, int *y1)
{
	int i, j;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);
	Motion_SOC(head_2cm);
	read_fpga_video_data(fvda);

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	DMZ_SV_new(HSV_data, input_image);

	int close_x[5] = { 0 };
	int close_y[3] = { 0 };
	int radius = 0;
	int check = 0;

	for (j = 118; j > 0; j--)
	{
		check = 0;
		for (i = CENTER - 20; i < CENTER + 20; i++)
		{
			if ((input_image[iw*j + i] == 0xffff) && (!check))
			{
				if (((input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff))
					|| ((input_image[iw*(j - 1) + i] == 0xffff) && (input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 4) + i] == 0xffff))
					|| ((input_image[iw*(j - 2) + i] == 0xffff) && (input_image[iw*(j - 3) + i] == 0xffff) && (input_image[iw*(j - 4) + i] == 0xffff)))
				{
					check = 1;
					close_y[0] = j;
					close_x[0] = i;
				}
			}
			else if ((input_image[iw*j + i] == 0xffff) && (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0) && (check == 1))
			{
				close_x[1] = i;
			}
			else if ((input_image[iw*j + i] == 0x00) && (input_image[iw*j + i + 1] == 0x0)
				&& (input_image[iw*j + i + 2] == 0x0) && (check == 1))
			{
				break;
			}
		}
		if (close_y[0] > 0)
			break;
	}

	close_x[2] = (close_x[0] + close_x[1]);

	if ((close_x[2] % 2) == 1)
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0) && (input_image[iw*(j - 2) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j == 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) && (input_image[iw*j + close_x[2] + 1] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) && (input_image[iw*(j - 1) + close_x[2] + 1] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
			else
				close_y[1] = close_y[0];
		}
	}
	else
	{
		if (close_x[1] > 0)
		{
			close_x[2] = (int)(close_x[2] / 2);
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
		else
		{
			if (close_y[0] > 0)
			{
				for (j = close_y[0] - 1; j > 0; j--)
				{
					if (j > 2)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0) &&
							(input_image[iw*(j - 2) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
					else if (j>1)
					{
						if ((input_image[iw*j + close_x[2]] == 0) &&
							(input_image[iw*(j - 1) + close_x[2]] == 0))
						{
							close_y[1] = j + 1;
							break;
						}
					}
				}
			}
		}
	}

	radius = (close_y[0] - close_y[1]) / 2 + 1;
	close_y[2] = (close_y[0] + close_y[1]) / 2;

	//calculate radius and find close_x[4]&close_x[5](which are x-coordinate of most side positioned pixels)
#if 1
	for (i = close_x[2] - radius - 1; i > CENTER - 10; i--)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 1] != 0xffff) &&
			(input_image[iw*(close_y[2]) + i - 2] != 0xffff))
		{
			close_x[3] = i + 1;
			break;
		}
	}

	for (i = close_x[2] + radius + 1; i < CENTER + 10; i++)
	{
		if ((input_image[iw*close_y[2] + i] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 1] != 0xffff) &&
			(input_image[iw*close_y[2] + i + 2] != 0xffff))
		{
			close_x[4] = i - 1;
			break;
		}
	}

#endif 

	*center_x1 = (close_x[4] + close_x[5]) / 2;
	*y1 = 120 - close_y[0];

	free(HSV_data);
	free(input_image);
	free(fvda);
}

/******************ÁÖ¾îÁø °ª¿¡ µû¶ó MotionÀ» Á¦¾îÇÏ´Â ÇÔ¼ö******************/
int MV_F_BigStep(int dist)
{
	int move;
	move = dist / 20;
	switch (move) {
	case 0:
		printf("\nsection 0\n");
		return 1;
	case 1:			// 19 ±îÁö
					//no Motion
		printf("\nsection 1@@@@@@@@@|\n");
		return 1;
	case 2:			// 40~59
		Motion_std(walk_1);
		//Motion_SOC(head_);
		printf("\nsection 2@@@@@@@@@|@@@@@@@@@|\n");
		break;
	case 3:			// 60~79 
		Motion_std(walk_2);
		//Motion_SOC(head_std);
		printf("\nsection 3@@@@@@@@@|@@@@@@@@@|@@@@@@@@@|\n");
		break;
	case 4:			// 80 ~
		Motion_std(walk_3);
		//Motion_SOC(head_std);
		printf("\nsection 4@@@@@@@@@|@@@@@@@@@|@@@@@@@@@|@@@@@@@@@|\n");
		break;
	default:
		Motion_std(walk_3);
		//Motion_SOC(head_std);
		printf("\ntoo far\n");
		break;
	}
	return 0;
}

int MV_F_SmallStep(int dist)
{
	if ((dist <= YMIN) && (dist > TOO_CLOSE))
		return 1;
	else if (dist <= TOO_CLOSE)
		Motion_std(backstep_0);
	else if (dist <= 29)		//sm1ÀÌ Æò±Õ 9
	{
		Motion_std(smallwalk_1);
	}
	else if (dist <= 39)		//sm2°¡ Æò±Õ 14
	{
		Motion_std(smallwalk_2);
	}
	else							//sm3°¡ Æò±Õ 20
	{
		Motion_std(smallwalk_3);
	}
	return 0;
}

void Motion_std(MOTION motion_num)
{
	Motion_SOC(motion_num);
	Motion_SOC(head_2cm);
}

//COUNT NUMBER OF BLACK PIXELS ON THE SIDE
#if 1
//¿ìÃø¿¡ °ËÀº»ö ¼±ÀÌ ±ÙÁ¢ÇØ ÀÖ´Â°¡? -1: 0
int Count_BLACK_R(int black_r_start)
{
	int cnt_black = 0;

	int i, j;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);

	read_fpga_video_data(fvda);

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	DMZ_SV_new(HSV_data, input_image);

	for (j = 1; j < 119; j++)
	{
		for (i = black_r_start; i < 179; i++)
		{
			if (input_image[iw*j + i] == 0xffff)
			{
				cnt_black++;
				if (cnt_black > NEAR_SIDE_EDGE_R)
					break;
			}
		}
		if (cnt_black > NEAR_SIDE_EDGE_R)
			break;
	}

	free(HSV_data);
	free(input_image);
	free(fvda);

	if (cnt_black > NEAR_SIDE_EDGE_R)
		return -1;
	else
		return 0;

}
//ÁÂÃø¿¡ °ËÀº»ö ¼±ÀÌ ±ÙÁ¢ÇØ ÀÖ´Â°¡? 1: 0
int Count_BLACK_L(int black_l_end)
{
	int cnt_black = 0;

	int i, j;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* input_image = (U16*)malloc(pl * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);

	read_fpga_video_data(fvda);

	//memory setting
	memset(input_image, 0, pl * 2);
	memset(HSV_data, 0, pl * 2);
	memcpy(HSV_data, fvda, pl * 2);

	DMZ_SV_new(HSV_data, input_image);

	for (j = 1; j < 119; j++)
	{
		for (i = 1; i < black_l_end; i++)
		{
			if (input_image[iw*j + i] == 0xffff)
			{
				cnt_black++;
				if (cnt_black > NEAR_SIDE_EDGE_L)
					break;
			}
		}
		if (cnt_black > NEAR_SIDE_EDGE_L)
			break;
	}

	free(HSV_data);
	free(input_image);
	free(fvda);

	if (cnt_black > NEAR_SIDE_EDGE_L)
		return 1;
	else
		return 0;
}
#endif 

//ÁÖ¾îÁø °ª¿¡ µû¶ó Áö·Ú Á¢±Ù
int CLOSER_Y(int dist)
{
	int mv;

	mv = dist / 10;

	switch (mv)
	{
	case 0:
	case 1:
		return 1;
	case 2:
		Motion_std(smallwalk_1);
		break;
	case 3:
		Motion_std(smallwalk_1);
		break;
	case 4:
	case 5:
		Motion_std(walk_1);
		break;
	case 6:
	case 7:
	case 8:
		Motion_std(walk_2);
		break;

	case 9:
		//	Motion_std(walk_2);
		//	break;
	default:
		Motion_std(walk_2);
	}
	return 0;
}

//Áö·Ú°¡ Áß½É¿¡ ¸Å¿ì °¡±î¿ï °æ¿ì, ±ÙÁ¢ÇÏ°Ô ÀÌµ¿ÇÏ¿© mine ¸ð¼ÇÀ» ÀÌ¿ëÇØ ´Ù¸®»çÀÌ·Î ±Øº¹
void OVERCOMEMINE(void)
{
	int mine_x, mine_y;

	while (1)
	{
		CHECKFRONTONE(&mine_x, &mine_y);

		//°Å¸®°¡ ³Ê¹« °¡±î¿ï ¶§  backstepÀ¸·Î °Å¸® Á¶±Ý ´Ã¸®±â
		if (mine_y <= TOO_CLOSE)
			Motion_std(backstep_0);
		//°Å¸®°¡ ³Ê¹« ¸Ö ¶§¿¡ °Å¸®¿¡ µû¶ó Á¶±Ý¾¿ Á¢±Ù
		else if (mine_y > YMIN)
			CLOSER_Y(mine_y);
		else if (mine_x - CENTER > CENTER_RANGE)
		{
			Motion_std(MV_R);
		}
		else if (CENTER - mine_x > CENTER_RANGE)
		{
			Motion_std(MV_L);
		}
		else
		{
			Motion_std(mine);
			break;
		}
	}

}

void MINE_FIRST(int*left, int *right)
{
	int L = *left;
	int R = *right;
	int x1, x2, y1, y2;

	while (1)
	{
		TWO_MINES_FINAL(&x1, &x2, &y1, &y2);

		if ((y1 <= TOO_CLOSE) &&
			(BETWEEN_RANGE(x1, SKI_L, SKI_R)))
			Motion_std(backstep_0);
		else if (y1 > YMIN)
			CLOSER_Y(y1);
		else if ((OUT_OF_SIDERANGE(x1, SKI_R, SKI_L))
			&& (OUT_OF_SIDERANGE(x2, SKI_R, SKI_L)))
		{
			Motion_std(PASS_MINE);
			break;
		}
		else if ((abs(x1 - x2) > SKI_R - SKI_L)
			&& (BETWEEN_RANGE(x1, SKI_L, SKI_L + 10)
				|| BETWEEN_RANGE(x1, SKI_R - 10, SKI_R)
				|| BETWEEN_RANGE(x2, SKI_L, SKI_L + 10)
				|| BETWEEN_RANGE(x2, SKI_R - 10, SKI_R)))
		{

			if (BETWEEN_RANGE(x1, SKI_L, SKI_L + 10))
			{
				if (x1 < SKI_L + 5)
				{
					Motion_std(MV_R_BIG);
					R = R + 2;
					L = L - 2;
				}
				else
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else if (BETWEEN_RANGE(x1, SKI_R - 10, SKI_R))
			{
				if (x1 > SKI_R - 5)
				{
					Motion_std(MV_L_BIG);
					L = L + 2;
					R = R - 2;
				}
				else
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
			}
			else if (BETWEEN_RANGE(x2, SKI_L, SKI_L + 10))
			{
				if (x2 < SKI_L + 5)
				{
					Motion_std(MV_R_BIG);
					R = R + 2;
					L = L - 2;
				}
				else
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else if (BETWEEN_RANGE(x2, SKI_R - 10, SKI_R))
			{
				if (x1 > SKI_R - 5)
				{
					Motion_std(MV_L_BIG);
					L = L + 2;
					R = R - 2;
				}
				else
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
			}
			else
			{

				if (abs(x1 - CENTER) > abs(x2 - CENTER))
				{
					if (abs(CENTER - x2) < CENTER_RANGE)
					{
						if (y2 < TOO_CLOSE)
							Motion_std(backstep_0);
						else if (y2 > YMIN)
							CLOSER_Y(y2);
						else
						{
							Motion_std(mine);
							break;
						}
					}
					else if (CENTER - x2 > CENTER_RANGE)
					{
						Motion_std(MV_L);
						L++;
						R--;
					}
					else if (x2 - CENTER > CENTER_RANGE)
					{
						Motion_std(MV_R);
						R++;
						L--;
					}
				}
				else
				{
					if (abs(CENTER - x1) < CENTER_RANGE)
					{
						if (y1 < TOO_CLOSE)
							Motion_std(backstep_0);
						else if (y1 > YMIN)
							CLOSER_Y(y1);
						else
						{
							Motion_std(mine);
							break;
						}
					}
					else if (CENTER - x1 > CENTER_RANGE)
					{
						Motion_std(MV_L);
						L++;
						R--;
					}
					else if (x1 - CENTER > CENTER_RANGE)
					{
						Motion_std(MV_R);
						R++;
						L--;
					}
				}
			}
		}
		else
		{

			if (abs(x1 - CENTER) > abs(x2 - CENTER))
			{
				if (abs(CENTER - x2) < CENTER_RANGE)
				{
					if (y2 < TOO_CLOSE)
						Motion_std(backstep_0);
					else if (y2 > YMIN)
						CLOSER_Y(y2);
					else
					{
						Motion_std(mine);
						break;
					}
				}
				else if (CENTER - x2 > CENTER_RANGE)
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
				else if (x2 - CENTER > CENTER_RANGE)
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else
			{
				if (abs(CENTER - x1) < CENTER_RANGE)
				{
					if (y1 < TOO_CLOSE)
						Motion_std(backstep_0);
					else if (y1 > YMIN)
						CLOSER_Y(y1);
					else
					{
						Motion_std(mine);
						break;
					}
				}
				else if (CENTER - x1 > CENTER_RANGE)
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
				else if (x1 - CENTER > CENTER_RANGE)
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
			}

		}
	}

	*left = L;
	*right = R;
}

void MINE_FIRST_flat(int*left, int *right)
{
	int L = *left;
	int R = *right;
	int x1, x2, y1, y2;
	int dist = 0;
	int huddle = 0;

	while (1)
	{
		TWO_MINES_FINAL(&x1, &x2, &y1, &y2);

		if ((y1 <= TOO_CLOSE) &&
			(BETWEEN_RANGE(x1, SKI_L, SKI_R))){
			Motion_SOC(backstep_0);
			Motion_SOC(head_2cm);
			// Motion_std(backstep_0); //revised 190114
		}
		else if (y1 > YMIN)
			CLOSER_Y(y1);
		else if ((OUT_OF_SIDERANGE(x1, SKI_R, SKI_L))
			&& (OUT_OF_SIDERANGE(x2, SKI_R, SKI_L)))
		{
			Motion_SOC(PASS_MINE);
			Motion_SOC(head_2cm);
			// Motion_std(PASS_MINE); //revised 190114    PASS_MINE = walk_3
			Motion_SOC(smallwalk_0);
			Motion_SOC(head_2cm);
			// Motion_std(smallwalk_0); //revised 190114
			make_flat();
			break;
		}
		else if ((abs(x1 - x2) > SKI_R - SKI_L)
			&& (BETWEEN_RANGE(x1, SKI_L, SKI_L + 10)
				|| BETWEEN_RANGE(x1, SKI_R - 10, SKI_R)
				|| BETWEEN_RANGE(x2, SKI_L, SKI_L + 10)
				|| BETWEEN_RANGE(x2, SKI_R - 10, SKI_R)))
		{

			if (BETWEEN_RANGE(x1, SKI_L, SKI_L + 10))
			{
				if (x1 < SKI_L + 5)
				{
					Motion_SOC(MV_R_BIG);
					Motion_SOC(head_2cm);
					// Motion_std(MV_R_BIG);
					R = R + 2;
					L = L - 2;
				}
				else
				{
					Motion_SOC(MV_R);
			Motion_SOC(head_2cm);
					// Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else if (BETWEEN_RANGE(x1, SKI_R - 10, SKI_R))
			{
				if (x1 > SKI_R - 5)
				{
					Motion_SOC(MV_L_BIG);
					Motion_SOC(head_2cm);
					// Motion_std(MV_L_BIG);
					L = L + 2;
					R = R - 2;
				}
				else
				{
					Motion_SOC(MV_L);
					Motion_SOC(head_2cm);
					// Motion_std(MV_L);
					L++;
					R--;
				}
			}
			else if (BETWEEN_RANGE(x2, SKI_L, SKI_L + 10))
			{
				if (x2 < SKI_L + 5)
				{
					Motion_SOC(MV_R_BIG);
					Motion_SOC(head_2cm);
					// Motion_std(MV_R_BIG);
					R = R + 2;
					L = L - 2;
				}
				else
				{
					Motion_SOC(MV_R);
			Motion_SOC(head_2cm);
					// Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else if (BETWEEN_RANGE(x2, SKI_R - 10, SKI_R))
			{
				if (x1 > SKI_R - 5)
				{
					Motion_SOC(MV_L_BIG);
			Motion_SOC(head_2cm);
					// Motion_std(MV_L_BIG);
					L = L + 2;
					R = R - 2;
				}
				else
				{
					Motion_SOC(MV_L);
			Motion_SOC(head_2cm);
					// Motion_std(MV_L);
					L++;
					R--;
				}
			}
			else
			{

				if (abs(x1 - CENTER) > abs(x2 - CENTER))
				{
					if (abs(CENTER - x2) < CENTER_RANGE)
					{
						if (y2 < TOO_CLOSE)
							{	Motion_SOC(backstep_0);
			Motion_SOC(head_2cm);
								// Motion_std(backstep_0);
							}
						else if (y2 > YMIN)
							CLOSER_Y(y2);
						else
						{
							Motion_SOC(mine);
			Motion_SOC(head_2cm);
							// Motion_std(mine);
							Motion_SOC(smallwalk_0);
			Motion_SOC(head_2cm);
							// Motion_std(smallwalk_0);
							make_flat();
							break;
						}
					}
					else if (CENTER - x2 > CENTER_RANGE)
					{
						Motion_SOC(MV_L);
			Motion_SOC(head_2cm);
						// Motion_std(MV_L);
						L++;
						R--;
					}
					else if (x2 - CENTER > CENTER_RANGE)
					{
						Motion_SOC(MV_R);
			Motion_SOC(head_2cm);
						// Motion_std(MV_R);
						R++;
						L--;
					}
				}
				else
				{
					if (abs(CENTER - x1) < CENTER_RANGE)
					{
						if (y1 < TOO_CLOSE){
							Motion_SOC(backstep_0);
			Motion_SOC(head_2cm);
							// Motion_std(backstep_0);
						}
						else if (y1 > YMIN)
							CLOSER_Y(y1);
						else
						{
							Motion_SOC(mine);
			Motion_SOC(head_2cm);
							// Motion_std(mine);
							Motion_SOC(smallwalk_0);
			Motion_SOC(head_2cm);
							// Motion_std(smallwalk_0);
							make_flat();
							break;
						}
					}
					else if (CENTER - x1 > CENTER_RANGE)
					{
						Motion_SOC(MV_L);
			Motion_SOC(head_2cm);
						// Motion_std(MV_L);
						L++;
						R--;
					}
					else if (x1 - CENTER > CENTER_RANGE)
					{
						Motion_SOC(MV_R);
			Motion_SOC(head_2cm);
						// Motion_std(MV_R);
						R++;
						L--;
					}
				}
			}
		}
		else
		{

			if (abs(x1 - CENTER) > abs(x2 - CENTER))
			{
				if (abs(CENTER - x2) < CENTER_RANGE)
				{
					if (y2 < TOO_CLOSE){
						Motion_SOC(backstep_0);
			Motion_SOC(head_2cm);
						// Motion_std(backstep_0);
					}
					else if (y2 > YMIN)
						CLOSER_Y(y2);
					else
					{
						Motion_SOC(mine);
			Motion_SOC(head_2cm);
						// Motion_std(mine);
						Motion_SOC(smallwalk_0);
			Motion_SOC(head_2cm);
						// Motion_std(smallwalk_0);
						make_flat();
						break;
					}
				}
				else if (CENTER - x2 > CENTER_RANGE)
				{
					Motion_SOC(MV_L);
			Motion_SOC(head_2cm);
					// Motion_std(MV_L);
					L++;
					R--;
				}
				else if (x2 - CENTER > CENTER_RANGE)
				{
					Motion_SOC(MV_R);
			Motion_SOC(head_2cm);
					// Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else
			{
				if (abs(CENTER - x1) < CENTER_RANGE)
				{
					if (y1 < TOO_CLOSE){
						Motion_SOC(backstep_0);
			Motion_SOC(head_2cm);
						// Motion_std(backstep_0);
					}
					else if (y1 > YMIN)
						CLOSER_Y(y1);
					else
					{
						Motion_SOC(mine);
			Motion_SOC(head_2cm);
						// Motion_std(mine);
						Motion_SOC(smallwalk_0);
			Motion_SOC(head_2cm);
						// Motion_std(smallwalk_0);
						make_flat();
						break;
					}
				}
				else if (CENTER - x1 > CENTER_RANGE)
				{
					Motion_SOC(MV_L);
			Motion_SOC(head_2cm);
					// Motion_std(MV_L);
					L++;
					R--;
				}
				else if (x1 - CENTER > CENTER_RANGE)
				{
					Motion_SOC(MV_R);
			Motion_SOC(head_2cm);
					// Motion_std(MV_R);
					R++;
					L--;
				}
			}

		}

		if ((huddle = check_huddle_num(&dist)) > 110) //revised 100-->110  / 190114
			break;
	}

	*left = L;
	*right = R;
}

void MINE_FIRST_noloop(int*left, int *right)
{
	int L = *left;
	int R = *right;
	int x1, x2, y1, y2;

	TWO_MINES_FINAL(&x1, &x2, &y1, &y2);

	if ((y1 <= TOO_CLOSE) &&
		(BETWEEN_RANGE(x1, SKI_L, SKI_R)))
		Motion_std(backstep_0);
	else if (y1 > YMIN)
		CLOSER_Y(y1);
	else if ((OUT_OF_SIDERANGE(x1, SKI_R, SKI_L))
		&& (OUT_OF_SIDERANGE(x2, SKI_R, SKI_L)))
	{
		Motion_std(PASS_MINE);
	}
	else if ((abs(x1 - x2) > SKI_R - SKI_L)
		&& (BETWEEN_RANGE(x1, SKI_L, SKI_L + 10)
			|| BETWEEN_RANGE(x1, SKI_R - 10, SKI_R)
			|| BETWEEN_RANGE(x2, SKI_L, SKI_L + 10)
			|| BETWEEN_RANGE(x2, SKI_R - 10, SKI_R)))
	{

		if (BETWEEN_RANGE(x1, SKI_L, SKI_L + 10))
		{
			if (x1 < SKI_L + 5)
			{
				Motion_std(MV_R_BIG);
				R = R + 2;
				L = L - 2;
			}
			else
			{
				Motion_std(MV_R);
				R++;
				L--;
			}
		}
		else if (BETWEEN_RANGE(x1, SKI_R - 10, SKI_R))
		{
			if (x1 > SKI_R - 5)
			{
				Motion_std(MV_L_BIG);
				L = L + 2;
				R = R - 2;
			}
			else
			{
				Motion_std(MV_L);
				L++;
				R--;
			}
		}
		else if (BETWEEN_RANGE(x2, SKI_L, SKI_L + 10))
		{
			if (x2 < SKI_L + 5)
			{
				Motion_std(MV_R_BIG);
				R = R + 2;
				L = L - 2;
			}
			else
			{
				Motion_std(MV_R);
				R++;
				L--;
			}
		}
		else if (BETWEEN_RANGE(x2, SKI_R - 10, SKI_R))
		{
			if (x1 > SKI_R - 5)
			{
				Motion_std(MV_L_BIG);
				L = L + 2;
				R = R - 2;
			}
			else
			{
				Motion_std(MV_L);
				L++;
				R--;
			}
		}
		else
		{

			if (abs(x1 - CENTER) > abs(x2 - CENTER))
			{
				if (abs(CENTER - x2) < CENTER_RANGE)
				{
					if (y2 < TOO_CLOSE)
						Motion_std(backstep_0);
					else if (y2 > YMIN)
						CLOSER_Y(y2);
					else
					{
						Motion_std(mine);
					}
				}
				else if (CENTER - x2 > CENTER_RANGE)
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
				else if (x2 - CENTER > CENTER_RANGE)
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
			}
			else
			{
				if (abs(CENTER - x1) < CENTER_RANGE)
				{
					if (y1 < TOO_CLOSE)
						Motion_std(backstep_0);
					else if (y1 > YMIN)
						CLOSER_Y(y1);
					else
					{
						Motion_std(mine);
					}
				}
				else if (CENTER - x1 > CENTER_RANGE)
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
				else if (x1 - CENTER > CENTER_RANGE)
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
			}
		}
	}
	else
	{

		if (abs(x1 - CENTER) > abs(x2 - CENTER))
		{
			if (abs(CENTER - x2) < CENTER_RANGE)
			{
				if (y2 < TOO_CLOSE)
					Motion_std(backstep_0);
				else if (y2 > YMIN)
					CLOSER_Y(y2);
				else
				{
					Motion_std(mine);
				}
			}
			else if (CENTER - x2 > CENTER_RANGE)
			{
				Motion_std(MV_L);
				L++;
				R--;
			}
			else if (x2 - CENTER > CENTER_RANGE)
			{
				Motion_std(MV_R);
				R++;
				L--;
			}
		}
		else
		{
			if (abs(CENTER - x1) < CENTER_RANGE)
			{
				if (y1 < TOO_CLOSE)
					Motion_std(backstep_0);
				else if (y1 > YMIN)
					CLOSER_Y(y1);
				else
				{
					Motion_std(mine);
				}
			}
			else if (CENTER - x1 > CENTER_RANGE)
			{
				Motion_std(MV_L);
				L++;
				R--;
			}
			else if (x1 - CENTER > CENTER_RANGE)
			{
				Motion_std(MV_R);
				R++;
				L--;
			}
		}

	}


	*left = L;
	*right = R;
}
void ONLY_LEFT_fix(int *left, int *right)
{
	int L = *left;
	int R = *right;
	int x[2] = { 0 };
	int y[2] = { 0 };
	int tilt = 0;

	while (1)
	{
		TWO_MINES_FINAL2(&x[0], &x[1], &y[0], &y[1], SKI_L - 10, ROI_x_max);
		//Áö·Ú°¡ ÇÏ³ªµµ ¾Èº¸ÀÏ ¶§
		if ((y[0] > VALID_Y) && (y[1] > VALID_Y))
		{
			Motion_std(PASS_MINE);
			break;
		}
		//Áö·Ú°¡ ÇÏ³ª¸¸ º¸ÀÏ ¶§
		else if ((y[1] > VALID_Y) || (x[1]>SKI_R))
		{	//Áö·Ú°¡ Áß½É¿¡ °¡±î¿ï °æ¿ì,
			//if (abs(x[0] - CENTER) <= NEAR_CENTER)
			if (y[0] <= TOO_CLOSE)
				Motion_std(backstep_0);
			else if (y[0] > YMIN)
				CLOSER_Y(y[0]);
			else if (abs(x[0] - CENTER) <= CENTER_RANGE + 5)
			{
				if (x[0] - CENTER > CENTER_RANGE)
				{
					Motion_std(MV_R);
					R++;
					L--;
				}
				else if (CENTER - x[0] > CENTER_RANGE)
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
				else
				{
					OVERCOMEMINE();
					break;
				}
			}
			else
			{
				Motion_std(MV_L);
				L++;
				R--;
			}
		}
		else
		{
			TWO_MINES_FINAL2(&x[0], &x[1], &y[0], &y[1], SKI_L - 5, SKI_R + 5);
			if (y[0] < TOO_CLOSE)
				Motion_std(backstep_0);
			else if (y[0] > VALID_Y)
			{
				Motion_std(PASS_MINE);
				break;
			}
			else if (x[1] > SKI_R)
				CLOSER_Y(y[0]);
			else if (x[0] > SKI_R)
				CLOSER_Y(y[1]);
			Motion_std(MV_L);
			L++;
			R--;
		}

		tilt = Count_BLACK_L(BLACK_END);
		tilt = Count_BLACK_R(BLACK_START);

		if (tilt != 0)
			break;
	}

	*left = L;
	*right = R;
}

void ONLY_RIGHT_fix(int *left, int *right)
{
	int L = *left;
	int R = *right;
	int x[2] = { 0 };
	int y[2] = { 0 };
	int tilt = 0;

	while (1)
	{
		TWO_MINES_FINAL2(&x[0], &x[1], &y[0], &y[1], ROI_x_min, SKI_R + 10);
		//Áö·Ú°¡ ÇÏ³ªµµ ¾Èº¸ÀÏ ¶§
		if ((y[0] > VALID_Y) && (y[1] > VALID_Y))
		{
			Motion_std(PASS_MINE);
			break;
		}
		//Áö·Ú°¡ ÇÏ³ª¸¸ º¸ÀÏ ¶§
		else if ((y[1] > VALID_Y) || (x[1] < SKI_L))
		{	//Áö·Ú°¡ Áß½É¿¡ °¡±î¿ï °æ¿ì,
			//if (abs(x[0] - CENTER) <= NEAR_CENTER)
			if (y[0] <= TOO_CLOSE)
				Motion_std(backstep_0);
			else if (y[0] > YMIN)
				CLOSER_Y(y[0]);
			else if (abs(x[0] - CENTER) <= CENTER_RANGE + 5)
			{
				if (x[0] - CENTER > CENTER_RANGE)
				{

					Motion_std(MV_R);
					R++;
					L--;
				}
				else if (CENTER - x[0] > CENTER_RANGE)
				{
					Motion_std(MV_L);
					L++;
					R--;
				}
				else
				{
					OVERCOMEMINE();
					break;
				}
			}
			else
			{
				Motion_std(MV_R);
				R++;
				L--;
			}
		}
		else
		{
			TWO_MINES_FINAL2(&x[0], &x[1], &y[0], &y[1], SKI_L - 5, SKI_R + 5);
			if (y[0] < TOO_CLOSE)
				Motion_std(backstep_0);
			else if (y[0] > VALID_Y)
			{
				Motion_std(PASS_MINE);
				break;
			}
			else if (x[1] > SKI_R)
				CLOSER_Y(y[0]);
			else if (x[0] > SKI_R)
				CLOSER_Y(y[1]);
			Motion_std(MV_R);
			R++;
			L--;
		}


		tilt = Count_BLACK_L(BLACK_END);
		tilt = Count_BLACK_R(BLACK_START);

		if (tilt != 0)
			break;
	}

	*left = L;
	*right = R;
}

//final mine
int OUT_OF_SIDERANGE(int a, int side_front, int side_back)
{
	if ((a > side_front) || (a < side_back))
		return 1;
	else
		return 0;
}

int BETWEEN_RANGE(int a, int side_left, int side_right)
{
	if ((a > side_left) && (a < side_right))
		return 1;
	else
		return 0;
}



//Blue Huddle
int check_huddle(U16* output, int* dist)
{
	int n, blue_num = 0;
	int px, py;
	int max_y = 120;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);
	U8 Hue, Sat, Val;

	U16** outplane = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		outplane[n] = &output[n * 180];
	U16** HSVplane = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		HSVplane[n] = &HSV_data[n * 180];

	Motion_SOC(head_2cm);
	read_fpga_video_data(fvda);

	memset(HSV_data, 0, pl * 2);

	//memory setting
	memcpy(HSV_data, fvda, pl * 2);

	memset(output, 0, pl * 2);

	for (py = 0; py < 100; py++) {
		for (px = 20; px < 160; px++) {
			Hue = (U8)((0xff00 & HSVplane[py][px]) >> 8);
			Sat = (U8)((0x00f0 & HSVplane[py][px]) >> 4);
			Val = (U8)(0x000f & HSVplane[py][px]);

			if ((Hue > 150) && (Hue < 190)
				&& (Sat > 3) && (Val > 3)) {
				outplane[py][px] = 0x001f;
				max_y = py;
				blue_num++;
			}
		}
	}

	for (py = 0; py < 120; py++)
		outplane[py][89] = 0xf800;
	/*
	for (i = 0; i < pl-3600; i++)
	{
	H_data[i] = (HSV_data[i] >> 8) & 0xff;
	S_data[i] = (HSV_data[i] >> 4) & 0x0f;
	V_data[i] = (HSV_data[i]) & 0x0f;

	if (((H_data[i] > 150) && (H_data[i] < 190))
	&& (S_data[i] > 3) && (V_data[i] > 3))
	{
	output[i] = 0x001f;
	blue_num++;
	}
	}
	*/
	//clear_screen();
	// draw_fpga_video_data_full(outplane);
	// flip();
	*dist = 120 - max_y;
	free(outplane);
	free(HSVplane);
	free(HSV_data);
	free(fvda);

	return blue_num;
}

int check_huddle_num(int* dist)
{
	int n, blue_num = 0;
	int px, py;
	int max_y = 120;

	U16* fvda = (U16*)malloc((pl + hl) * 2);
	U16* HSV_data = (U16*)malloc(pl * 2);
	U8 Hue, Sat, Val;

	//U16** outplane = (U16**)malloc(sizeof(U16*) * 120);
	//for (n = 0; n < 120; n++)
	//	outplane[n] = &output[n * 180];
	U16** HSVplane = (U16**)malloc(sizeof(U16*) * 120);
	for (n = 0; n < 120; n++)
		HSVplane[n] = &HSV_data[n * 180];

	//Motion_SOC(head_2cm);
	read_fpga_video_data(fvda);

	memset(HSV_data, 0, pl * 2);

	//memory setting
	memcpy(HSV_data, fvda, pl * 2);

	for (py = 0; py < 100; py++) {
		for (px = 20; px < 160; px++) {
			Hue = (U8)((0xff00 & HSVplane[py][px]) >> 8);
			Sat = (U8)((0x00f0 & HSVplane[py][px]) >> 4);
			Val = (U8)(0x000f & HSVplane[py][px]);

			if ((Hue > 150) && (Hue < 190)
				&& (Sat > 3) && (Val > 3)) {
				max_y = py;
				blue_num++;
			}
		}
	}

	//for (py = 0; py < 120; py++)
	//	outplane[py][89] = 0xf800;

	*dist = 120 - max_y;
	//free(outplane);
	free(HSVplane);
	free(HSV_data);
	free(fvda);

	return blue_num;
}

#endif