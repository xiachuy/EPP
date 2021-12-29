#include "math.h"

//机械臂解算相关参数
#define pi 3.1415
#define u pi/180
#define i 180/pi
//#define L1 100 //大臂臂长
//#define L2 100 //小臂臂长
#define sigma0 0.523 //pi/6

extern float x,y;//x y 分别为机械臂解算的目标值
extern float r;
extern float sigma1,sigma2,sigma3,sigma4,sigma5;
extern float T_sigma1;
extern float T_sigma2;


