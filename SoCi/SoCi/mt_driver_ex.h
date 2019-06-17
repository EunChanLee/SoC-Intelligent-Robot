#ifndef __MT_DRIVER_EX_H__
#define __MT_DRIVER_EX_H__
int mot_gotoReadyLine_red(int fcnt, int walk_ver);
int mot_gotoReadyLine_crevasses(int fcnt, int walk_ver);
int mot_gotoReadyLine_huddle(int fcnt, int walk_ver);
// 우측 선을 보고 중앙을 맞추는 모션 집합
int mot_arrangeMid(int fcnt);
int mot_arrangeMid_Ver2(int fcnt, int limit);
// 가로선을 찾고, 가로선 앞으로 정렬하는 모션 집합 (가로선 찾을 때까지 움직임)
int mot_gotoReadyLine_trap(int onboard, int fcnt, int walk_ver);
int mot_gotoReadyLine_bdg(int fcnt, int walk_ver);
int mot_tarArrMid(int tar);
// 징검다리를 건너는 모션 집합
int mot_runBridge(int fcnt, int walk_ver);
// 노란 차단선이 사라지면 OK하고, 코드를 진행하는 영상 처리 집합
int watch_waitGreenLight(int fcnt);
int tester_trap(void);
int watch_yellowboard(int fcnt);
#endif
