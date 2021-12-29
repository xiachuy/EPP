#ifdef __cplusplus
 extern "C" {
#endif
     
#define S_ACCEL 1    
#define T_ACCEL 0

/* S�ͼ��ٲ��� */
#define ACCELERATED_SPEED_LENGTH1 50  //������ٶȵĵ�������ʵҲ��3000��ϸ�ֲ�����˼��������������ı���ٵ�
#define ACCELERATED_SPEED_LENGTH2 50  //������ٶȵĵ�������ʵҲ��3000��ϸ�ֲ�����˼����
#define FRE_MIN 0  //��͵�����Ƶ�ʣ����������������������ٶ�
#define FRE_MAX 2000 //��ߵ�����Ƶ�ʣ������������������ʱ������ٶ�3500
     
#define STOP           0 // �Ӽ�������״̬��ֹͣ
#define ACCEL          1 // �Ӽ�������״̬�����ٽ׶�
#define DECEL          2 // �Ӽ�������״̬�����ٽ׶�
#define RUN            3 // �Ӽ�������״̬�����ٽ׶�
//	 
//extern int flag;	
//extern int stop_flag;
//extern int all_step_1;    //�ܹ����в���
//extern int all_step_2;    //�ܹ����в���
//extern int step_to_run[]; //Ҫ�������еĲ���  //Ҫ�������еĲ���       �ܹ����в��� = ACCELERATED_SPEED_LENGTH*2 + step_to_run	 
//extern int step_to_run_1;
//extern int	step_to_run_2;

extern float fre[ACCELERATED_SPEED_LENGTH1]; //����洢���ٹ�����ÿһ����Ƶ�� 
extern unsigned short period[ACCELERATED_SPEED_LENGTH1]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ 

extern float fre2[ACCELERATED_SPEED_LENGTH2]; //����洢���ٹ�����ÿһ����Ƶ�� 
extern unsigned short period2[ACCELERATED_SPEED_LENGTH2]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ


extern float fre3[ACCELERATED_SPEED_LENGTH2]; //����洢���ٹ�����ÿһ����Ƶ�� 
extern unsigned short period3[ACCELERATED_SPEED_LENGTH2]; //���鴢����ٹ�����ÿһ����ʱ�����Զ�װ��ֵ
 
void CalculateSModelLine(float fre_[], unsigned short period_[], float len, float fre_max, float fre_min, float flexible);