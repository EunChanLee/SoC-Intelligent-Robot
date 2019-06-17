#ifndef __STDIP_EX_H__
#define __STDIP_EX_H__

#define iw 180
#define ih 120
#define pl 21600
#define hl 400

#define setRad 6
#define setCap 10

#define HEIGHT 120
#define WIDTH 180

#define vert_delim 9
#define vert_tilt 3
#define vert_margin (iw/vert_delim)
#define vert_stage (vert_delim*vert_tilt)
#define hzt_delim 9
#define hzt_tilt 2
#define hzt_margin (150/hzt_delim)
#define hzt_stage (hzt_tilt + 1)

#define smp_frame 21

//jj
#if 1
//#define SECOND_THRESHOLD 95
#define SKI_L 45			// È­¸é¿¡¼­ ¿ÞÂÊ ¹ß ³¡ÀÇ xÁÂÇ¥(¿©À¯ºÐ Æ÷ÇÔ)						@@@
#define SKI_R 134			// È­¸é¿¡¼­ ¿À¸¥ÂÊ ¹ß ³¡ÀÇ xÁÂÇ¥(¿©À¯ºÐ Æ÷ÇÔ)						@@@
//#define GO_ROUND_L	49// 
//#define GO_ROUND_R	107// 
#define CENTER 91																					//@@@
#define CENTER_RANGE 2
#define WIDTH_ENOUGH 65									//´Ù¸® ÇÑ°³°¡ µé¾î°¥ Á¤µµ ÀÌ»ó ¿©À¯ºÐ@@@
#define TOO_CLOSE 11
#define PASS_MINE walk_3
#define MV_L sl_1
#define MV_L_BIG sl_2
#define MV_R sr_1
#define MV_R_BIG sr_2
#define NEAR_CENTER 15
#define MARGIN_RANGE 10
#define NEAR_SIDE_EDGE_L 500			//¿Ö ÀÌ·¸°Ô Â÷ÀÌ°¡ Å©Áö...
#define NEAR_SIDE_EDGE_R 400
#define FARENOUGH 70		//Ã¹¹øÂ° Áö·Ú¿Í µÎ¹øÂ° Áö·Ú»çÀÌÀÇ ÃæºÐÇÑ °Å¸®¹üÀ§

//Áö·Ú¸¦ Ã£´Â ¹üÀ§?
#define BOUNDARY_L 30			//SKI_L=45
#define BOUNDARY_R 150			//SKI_R=134

//Ã¹¹øÂ° Áö·Ú¸¦ µÑ·¯½Î´Â ¹ÝÁö¸§
#define RADIUS 7 

//ÇÈ¼¿ ¹üÀ§ °ü·Ã
#define BLACK_END 50
#define BLACK_START 130

//Y-Ãà À¯È¿ °Å¸®
#define VALID_Y 100

//MINE ±Øº¹ °ü·Ã
#define YMIN 19

//ROI
#define ROI_x_min	SKI_L-10
#define ROI_x_max	SKI_R+10
#define ROI_y_min	19
#define ROI_y_max	118

//#define MVMAX 10
//#define MARGIN 2

//Threshold Number for blue Huddle 
#define HUDDLE_BLUE 100 //revised 0114  #define HUDDLE_BLUE 100

#endif


#define _wait DelayLoop(500)
#define _wait_250ms DelayLoop(250)
#define _wait_d DelayLoop(150)

#define XY(x, y) (y*iw + x)

#define ysdad

//vert_map mode defines
typedef enum vOpt_tag {
	single_thr,
	double_thr,
	top_porking,
	bot_porking
}vOpt;

typedef enum vMode_tag {
	vert,
	hrizon,
	peeking,
	fitting
}vMode;

//mid fitting defines
typedef enum arrMid_tag {
	pattern_na = -1,
	skewed_left,
	skewed_right,
	far_far,
	far,
	fitted,
	bit_close,
	too_close
}arrMid;

typedef enum vObs_tag {
	default_obs,
	start_bar,
	final_bar,
	bridge_up,
	bridge_on,
	bridge_down,
	trap_pre,
	trap_hole,
	trap_arrange
}vObs;
//debug define
//#define show_htda_dat
//#define show_htda_dat_detail
//#define show_vert_idx
//#define show_hzt_idx
//#define show_hzt_idx2
//#define show_bndy_vote_res
//#define lcd_dbg

typedef struct pt_tag {
	int x;
	int y;
}pt;
typedef struct vs_tag {
	pt data[100];
	int cx;
	int cy;
	int dn;
}vs;
typedef enum tag_COLOR {
	RED,
	BLUE,
	YELLOW,
	GREEN,
	ORANGE,
	WHITE,
	BLACK,
	NONE
}COLOR;
/*
------------------- Ä«¸Þ¶ó °¢µµÁ¶Àý (1~10) ------------------
1   [head_2cm]  : Á¤ÀÚ¼¼						///// 7°ª:435, 8°ª:503
2   [head_bar]  : Ã³À½ °í°³(³ë¶õ¹Ù)			///// 7°ª:332, 8°ª:503
3   [head_right]  : ¿À¸¥ÂÊ º¸±â				///// 7°ª:274, 8°ª:199 --> 7°ª:370, 8°ª:199
4   [head_std]  : ±âº» °í°³					///// 7°ª:418, 8°ª:503
5   [head_green]  : ÃÊ·Ï»ö³¡					///// 7°ª:292, 8°ª:503
6   [head_bot]  : ¹Ù´Ú º¸±â					///// 7°ª:512, 8°ª:503
7   []  : ¿À¸¥ÂÊ º¸±â °í°³µé°í			///// °ª ¹Ù²ã¾ßÇØ
8   []  : °í°³ Ç«¼÷ÀÌ±â				///// °ª ¹Ù²ã¾ßÇØ
9   []  : ¿À¸¥ÂÊ º¸±â °í°³¼÷ÀÌ°í		///// °ª ¹Ù²ã¾ßÇØ
10  [head_pre]  : 							///// °ª ¹Ù²ã¾ßÇØ
------------------------------------------------------------
¿À¸¥ÂÊ º¸±â (head_right) ÇÏ±â Àü¿¡ °í°³´Â ¹Ù´Ú º¸±â (head_bot) ÀÌ¸é ¾ÈµÊ. ´Ù¸¥ °í°³·Î º¯ÇüÈÄ ¼öÇàÇÒ °Í
¿À¸¥ÂÊ º¸±â (head_right)¿¡¼­ Àü¹æ ÁÖ½Ã·Î °í°³¸¦ ÁÂ¿ì º¯µ¿ÇÒ ¶§, head_pre ¸¦ ²À ¼öÇàÇÒ °Í.

--------------------- º¸Çà¸ð¼Ç (11~26) ---------------------
11  [walk_1]  : °È±â_1 (2°ÉÀ½)
12  [walk_2]  : °È±â_2 (3°ÉÀ½)
13  [walk_3]  : °È±â_3 (4°ÉÀ½)
14  [walk_4]  : °È±â_4 (5°ÉÀ½)
15  [walk_5]  : °È±â_5 (6°ÉÀ½)

16  [backstep_1]  : ÈÄÁø_1 (2°ÉÀ½)
17  [backstep_2]  : ÈÄÁø_2 (3°ÉÀ½)
18  [backstep_3]  : ÈÄÁø_3 (4°ÉÀ½)
19  [backstep_4]  : ÈÄÁø_4 (5°ÉÀ½)
20  [backstep_5]  : ÈÄÁø_5 (6°ÉÀ½)

21  [smallwalk_1]  : ÃÑÃÑ°ÉÀ½_1 (2°ÉÀ½)
22  [smallwalk_2]  : ÃÑÃÑ°ÉÀ½_2 (3°ÉÀ½)
23  [smallwalk_3]  : ÃÑÃÑ°ÉÀ½_3 (4°ÉÀ½)
24  [smallwalk_4]  : ÃÑÃÑ°ÉÀ½_4 (5°ÉÀ½)
25  [smallwalk_0]  : ÃÑÃÑ°ÉÀ½_5 (6°ÉÀ½)

26  [toddler]  : Åäµé·¯
------------------------------------------------------------

----------------- °¢µµÆ²±â/È¾ÀÌµ¿ (31~70) ------------------
31  [tl_1]  : ÁÂÇâÁÂ_1
32  [tl_2]  : ÁÂÇâÁÂ_2
33  [tl_3]  : ÁÂÇâÁÂ_3

34  [tr_1]  : ¿ìÇâ¿ì_1
35  [tr_2]  : ¿ìÇâ¿ì_2
36  [tr_3]  : ¿ìÇâ¿ì_3

37  [tlhead_1]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂÇâÁÂ_1
38  [tlhead_2]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂÇâÁÂ_2
39  [tlhead_3]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂÇâÁÂ_3

40  [trhead_1]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ìÇâ¿ì_1
41  [trhead_2]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ìÇâ¿ì_2
42  [trhead_3]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ìÇâ¿ì_3

43  [tl_90]  : ÁÂÇâÁÂ_90 (90µµ µ¹±â)
44  [tr_90]  : ¿ìÇâ¿ì_90 (90µµ µ¹±â)

45  [sl_1]  : ÁÂ·Î ÀÏº¸_1
46  [sl_2]  : ÁÂ·Î ÀÏº¸_2
47  [sl_3]  : ÁÂ·Î ÀÏº¸_3
48  [sl_4]  : ÁÂ·Î ÀÏº¸_4

49  [sr_1]  : ¿ì·Î ÀÏº¸_1
50  [sr_2]  : ¿ì·Î ÀÏº¸_2
51  [sr_3]  : ¿ì·Î ÀÏº¸_3
52  [sr_4]  : ¿ì·Î ÀÏº¸_4

53  [slhead_1]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂ·Î ÀÏº¸_1
54  [slhead_2]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂ·Î ÀÏº¸_2
55  [slhead_3]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂ·Î ÀÏº¸_3
56  [slhead_4]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ÁÂ·Î ÀÏº¸_4

57  [srhead_1]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ì·Î ÀÏº¸_1
58  [srhead_2]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ì·Î ÀÏº¸_2
59  [srhead_3]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ì·Î ÀÏº¸_3
60  [srhead_4]  : °í°³ µ¹¸°»óÅÂ¿¡¼­ ¿ì·Î ÀÏº¸_4
------------------------------------------------------------

------------------ Àå¾Ö¹°³Ñ±â (101~108) --------------------
101  [_2cm_up] : 2cm ¿Ã¶ó°¡±â
102  [_2cm_down] : 2cm ³»·Á°¡±â
103  [bulldozer] : ºÒµµÀú
104  [trap_dumbling] : ÇÔÁ¤_´ýºí¸µ
105  [huddle_dumbling] : Çãµé_´ýºí¸µ
106  [soccer] : °øÂ÷±â
107  [mine] : Áö·Ú ³Ñ±â

------------------ Ãß°¡¿° (108~) --------------------
108  [walk_1_5] : °È±â_Áß°£
109  [sl_0] : ÁÂ·Î ÀÏº¸_¼öÁ¤
110  [sr_0] : ¿ì·Î ÀÏº¸_¼öÁ¤
111  [backstep_0] : ÈÄÁø_0

------------------------------------------------------------
Updated : 2017-08-03
*/
typedef enum tag_MOTION {
	con_check = 0,
	head_2cm = 1,
	head_bar,
	head_right,
	head_std,
	head_green,
	head_bot,
	head_blue_hole,
	head_blue_hole_new = 9,
	head_pre = 10,

	walk_1 = 11,
	walk_2,
	walk_3,
	walk_4,
	walk_6,
	backstep_1,
	backstep_2,
	backstep_3,
	backstep_4,
	backstep_5,
	smallwalk_1,
	smallwalk_2,
	smallwalk_3,
	smallwalk_4,
	smallwalk_0,
	toddler,

	tl_1 = 31,
	tl_2,
	tl_3,
	tr_1,
	tr_2,
	tr_3,
	tlhead_1,
	tlhead_2,
	tlhead_3,
	trhead_1,
	trhead_2,
	trhead_3,
	tl_90,
	tr_90,
	sl_1,
	sl_2,
	sl_3,
	sl_4,
	sr_1,
	sr_2,
	sr_3,
	sr_4,
	slhead_1,
	slhead_2,
	slhead_3,
	slhead_4,
	srhead_1,
	srhead_2,
	srhead_3,
	srhead_4,
	walk_1_ver2 = 61,
	walk_2_ver2,
	walk_3_ver2,
	walk_4_ver2,
	walk_5_ver2,
	walk_1_ver3 = 71,
	walk_2_ver3,
	walk_3_ver3,
	walk_4_ver3,
	walk_5_ver3,
	_2cm_up = 101,
	_2cm_down,
	bulldozer,
	trap_dumbling,	//104
	huddle_dumbling,
	soccer,
	mine,
	walk_1_5,
	sl_0,
	sr_0,
	backstep_0,
	safe_walk = 119,
	head_down = 125,
	head_down_2,
	crevasse = 241,
	wakeup_front = 249,
	wakeup_back = 250,
	mine_std
}MOTION;


void morphology_Erode(U16 **input_Image,U16 **output_Image,int size_Height, int size_Width);
void morphology_Dilate(U16 **input_Image, U16 **output_Image, int size_Height, int size_Width);

int mot_arrangeMid_2(int fcnt);

int concat_num(char* dst, int num);
int get_htda(U16* htda, U16* fvda);
int get_surf(U16* surf, U16* fvda);
void draw_surf(U16* surf, int slot, int color);
void Motion_while(MOTION motion);
int Motion_SOC(MOTION motion);
int Motion_SOC_log(MOTION motion, int log);
int Motion_walker(MOTION motion, int ver);
int sideLookupCRatio_jh(U16* surf, COLOR user_color, int rho, int xpeek, int isleft);

COLOR colorpeek(U8 hue, U8 sat, U8 val);
void sideRegionCheck(U16* surf, int rho, int isleft, int tag);
COLOR sideLookupColor(U16* surf, int rho, int isleft);
int colorCalib(U16* surf, int ycenter, U8* cal_h, U8* cal_s, U8* cal_v);
int colorRegion(U16* outsurf, U16* hsvsurf, COLOR user_color);
int pure_red(U16* outsurf, U16* hsvsurf);

int chop_boundary(U16* htda, int htda_num, int* st, int* score);
int chop_hrizon(U16* htda, int htda_num, int* grp, int* go_rho, vOpt usr_option, vObs usr_obstacle);
int chop_vert(U16* htda, int htda_num, int* grp, int* go_rho1, int* go_rho2, vObs usr_obstacle);
int vert_map_for_bdgpeek(U16* htda, int htda_num, int* st, int* score_left, int*score_right, vObs obstacle);
void sideRegionCheck2(U16* surf, int rho, int isleft, int tag);

int peek_bndy(FILE* log);
int peek_bndy_2(FILE* log);
int line_target(FILE* log, int tar_rho, vMode behavior, vOpt tar_option, vObs usr_obs);
int line_target2(FILE* log, int tar_rho, vMode behavior, vOpt tar_option, vObs usr_obs);
int peek_start(FILE* log);
int peek_vert(FILE* log, int onBridge, vObs obstacle);
int peek_start_dmz(FILE* log, int cnt);
int bdgpeek(FILE* log, int onBridge, vObs obstacle);
int peek_bdgland(FILE* log);
int peek_start_yellowboard(FILE* log);
//int	mot_arrangeMid_jw(int fcnt);
int mot_gotoReadyLine_bdg_jw0130(int fcnt, int walk_ver);
//
int ORANGE_Golf_HS(FILE* log, int h, int s, U16* output_image, int *y_length);
void ORANGE_Golf_HS_ver2(FILE* log, int *y_length);
int MV_F_BigStep_golf(FILE* log, int dist);
void BLUE_Hole(U16* output, int* hole_length, int* hole_x, int* hole_y);
void HSV2EACH(U16* HSV_data, U8* H_data, U8* S_data, U8* V_data, int n);
int MV_LR_BigStep_golf(FILE* log, int dist);
//jj
#if 1
int bndypeek(void);
int hzt_map(U16* htda, int htda_num, int* st, int* score);
int make_flat(void);

void DMZ_SV_new(U16* HSV, U16* output_image);
int CLOSEONE(U16* output_image, int *center_x); //Á¦ÀÏ ±ÙÁ¢ÇÑ yÁÂÇ¥ ¹ÝÈ¯

void TWO_MINES_FINAL(int *center_x1, int *center_x2, int *y1, int *y2);

int MV_F_BigStep(int dist);
int MV_F_SmallStep(int dist);

int OUT_OF_SIDERANGE(int a, int side_front, int side_back);

int BETWEEN_RANGE(int a, int side_left, int side_right);


void TWO_MINES_FINAL2(int *center_x1, int *center_x2, int *y1, int *y2, int x_start, int x_end);

void CHECKFRONTONE(int *center_x1, int *y1);

void Motion_std(MOTION motion_num);

int Count_BLACK_R(int black_r_start);
int Count_BLACK_L(int black_l_end);

int CLOSER_Y(int dist);

void OVERCOMEMINE(void);

void MINE_FIRST(int*left, int *right);
void MINE_FIRST_flat(int*left, int *right);
void MINE_FIRST_noloop(int*left, int *right);

void ONLY_LEFT_fix(int *left, int *right);
void ONLY_RIGHT_fix(int *left, int *right);

int check_huddle(U16* output, int* dist);
int check_huddle_num(int* dist);
#endif

#endif // __STDIP_H__
