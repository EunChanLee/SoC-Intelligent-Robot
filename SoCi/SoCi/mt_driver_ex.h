#ifndef __MT_DRIVER_EX_H__
#define __MT_DRIVER_EX_H__
int mot_gotoReadyLine_red(int fcnt, int walk_ver);
int mot_gotoReadyLine_crevasses(int fcnt, int walk_ver);
int mot_gotoReadyLine_huddle(int fcnt, int walk_ver);
// ���� ���� ���� �߾��� ���ߴ� ��� ����
int mot_arrangeMid(int fcnt);
int mot_arrangeMid_Ver2(int fcnt, int limit);
// ���μ��� ã��, ���μ� ������ �����ϴ� ��� ���� (���μ� ã�� ������ ������)
int mot_gotoReadyLine_trap(int onboard, int fcnt, int walk_ver);
int mot_gotoReadyLine_bdg(int fcnt, int walk_ver);
int mot_tarArrMid(int tar);
// ¡�˴ٸ��� �ǳʴ� ��� ����
int mot_runBridge(int fcnt, int walk_ver);
// ��� ���ܼ��� ������� OK�ϰ�, �ڵ带 �����ϴ� ���� ó�� ����
int watch_waitGreenLight(int fcnt);
int tester_trap(void);
int watch_yellowboard(int fcnt);
#endif
