#ifdef __cplusplus
 extern "C" {
#endif
     
#define S_ACCEL 1    
#define T_ACCEL 0

/* S型加速参数 */
#define ACCELERATED_SPEED_LENGTH1 50  //定义加速度的点数（其实也是3000个细分步的意思），调这个参数改变加速点
#define ACCELERATED_SPEED_LENGTH2 50  //定义加速度的点数（其实也是3000个细分步的意思），
#define FRE_MIN 0  //最低的运行频率，调这个参数调节最低运行速度
#define FRE_MAX 2000 //最高的运行频率，调这个参数调节匀速时的最高速度3500
     
#define STOP           0 // 加减速曲线状态：停止
#define ACCEL          1 // 加减速曲线状态：加速阶段
#define DECEL          2 // 加减速曲线状态：减速阶段
#define RUN            3 // 加减速曲线状态：匀速阶段
//	 
//extern int flag;	
//extern int stop_flag;
//extern int all_step_1;    //总共运行步数
//extern int all_step_2;    //总共运行步数
//extern int step_to_run[]; //要匀速运行的步数  //要匀速运行的步数       总共运行步数 = ACCELERATED_SPEED_LENGTH*2 + step_to_run	 
//extern int step_to_run_1;
//extern int	step_to_run_2;

extern float fre[ACCELERATED_SPEED_LENGTH1]; //数组存储加速过程中每一步的频率 
extern unsigned short period[ACCELERATED_SPEED_LENGTH1]; //数组储存加速过程中每一步定时器的自动装载值 

extern float fre2[ACCELERATED_SPEED_LENGTH2]; //数组存储加速过程中每一步的频率 
extern unsigned short period2[ACCELERATED_SPEED_LENGTH2]; //数组储存加速过程中每一步定时器的自动装载值


extern float fre3[ACCELERATED_SPEED_LENGTH2]; //数组存储加速过程中每一步的频率 
extern unsigned short period3[ACCELERATED_SPEED_LENGTH2]; //数组储存加速过程中每一步定时器的自动装载值
 
void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible);