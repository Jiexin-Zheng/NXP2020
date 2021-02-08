#include "include.h"
#include "common.h"
int dawan_speed=67;
int long_speed=96;
int S_speed=95;
s16 speed_a,speed_b;
extern int start_line_delay_counter,straight_flag,duandian,qulv_yuandaun,qulv_jinduan,qvlv_quanju,error_from_calculation ,star_lineflag,zhangaijishileft_flag,zhangaijishiright_flag;
int podao_flag,speed_to_set;
char dawan_flag,zhidao_ruwan_flag;
extern char S;
void duoji_dianji_Init(void)
{
  gpio_init (PTD3 ,GPO,LOW);        //���1--ʹ��IO��ʼ��
}



void get_pulse(void)//��ȡ�ٶ�
{
    speed_a=-FTM2_CNT;
    FTM2_CNT=0;
}

  /*******************************************************12��������ת�������Ӻ���*****************************************************/  

void motor_backward (int zhankongbi)//zhankongbi 0-10000��ת
{
ftm_pwm_duty(FTM0,FTM_CH4,0);  //���
ftm_pwm_duty(FTM0,FTM_CH5,zhankongbi);   

}

void motor_foreward (int zhankongbi )//zhankongbi 0-10000��ת
{
ftm_pwm_duty(FTM0,FTM_CH4,zhankongbi); // ���
ftm_pwm_duty(FTM0,FTM_CH5,0);   


}


  /*******************************************************�ٶȿ���*****************************************************/  

int error_to_motor_pid,d_error_to_motor_pid,last_error_to_motor_pid,pulse_get=0;
int motor_kp,motor_kd;
int output_duty_to_motor_pid,last_output_duty_to_motor_pid,CS=11;
extern char pof,crossing_flag;
void motor_parameters_init(void)//���������ʼ��
{
error_to_motor_pid=0;
d_error_to_motor_pid=0;
last_error_to_motor_pid=0;
motor_kp=40;
motor_kd=25;
output_duty_to_motor_pid=0;
last_output_duty_to_motor_pid=0;
}
void motor_pid(int pulse_to_set)//�ٶ�PID����
{
 
  pulse_get=speed_a;//
  
  //********************���÷�ת����**************//

   int dut,limit;
   /*if(guai_flag)//Բ����ʼ������
   {
   dut=4500;//��ת����Ϊ45%
    limit=0;//Ŀ���ٶ�-ʵ���ٶ�
   }
   else*/
    if(zhangaijishileft_flag||zhangaijishiright_flag)//�ϰ�
   {
    dut=1000;//��ת����Ϊ10%
    limit=-6;//Ŀ���ٶ�-ʵ���ٶ�
   }else
    if(start_line_delay_counter>30&&pulse_get>6)//��⵽�����ߺ���ʱ150ms���ҵ�ǰ�ٶȴ���6
    {
    dut=2000;//��ת����20%����ת���Ϊ30%ռ�ձȣ�����ͣ���������
    limit=-4;
    }
    else
    if(pof)//�µ�
{
  dut=2000;
limit=-5;
}
    else
    if(straight_flag>20)//��ֱ��
 {
   zhidao_ruwan_flag=0;
   dut=0;//3000
   limit=-10;
 }
    else
    if(zhidao_ruwan_flag)//ֱ������
  {
    dut=2900;//���3000
    limit=0;
  }else//�������
  {
  dut=1500;//1000
  limit=-10;
  }
  
  if(yunsu_flag&&!star_lineflag)//����
  {
  dut=1000;//1000
 limit=-10;
  }
  
  
  //***************�������pid��ʽ****************************//
   error_to_motor_pid=pulse_to_set-pulse_get;//����ƫ��=��������-ʵ������
   d_error_to_motor_pid=error_to_motor_pid-last_error_to_motor_pid;
   last_error_to_motor_pid=error_to_motor_pid;
   output_duty_to_motor_pid=last_output_duty_to_motor_pid+motor_kp*error_to_motor_pid+motor_kd*d_error_to_motor_pid;
   if(output_duty_to_motor_pid>7000)
    output_duty_to_motor_pid=7000; 
   if(output_duty_to_motor_pid<0)
   output_duty_to_motor_pid=0; 
   last_output_duty_to_motor_pid=output_duty_to_motor_pid;
   
   
   if(error_to_motor_pid<limit)//ʵ���ٶȺ������ٶȲ�ֵС���趨���ż�
   motor_backward (dut);//�����趨��ռ�ձȷ�ת
    else 
      if(error_to_motor_pid>6)//��������ٶȱ�ʵ���ٶȴ�6���ϣ���ֱ�ӿ��Ƶ����ת����
        motor_foreward (7500);//bangbang�����㷨
    else
   motor_foreward (last_output_duty_to_motor_pid);//����ͨ��pid����

}


void track_judge()//·���ж�
{
   if(zhangaijishileft_flag||zhangaijishiright_flag)//�ϰ�
   {
     speed_to_set=dawan_speed;//����
     dawan_flag=0;
   }
   else
    if(pof)//�µ�
{
 // gpio_set(PTE1,1);
  straight_flag=0;
speed_to_set=60;
}
else
if(S>25)//Сs��
 {
speed_to_set=S_speed;
 }else
  if(straight_flag>13)//��ֱ��
  {
    speed_to_set=long_speed;      
      dawan_flag=0;
    // gpio_set(PTE1,1);
  }
  else
   
if(qvlv_quanju>12&&qvlv_quanju<=25&&abs(error_from_calculation )<=21&&abs(error_from_calculation )>=13)//С��
{ 
  //gpio_set(PTE1,1);
speed_to_set=dawan_speed;
dawan_flag=0;
}
else
 if(qvlv_quanju>25||abs(error_from_calculation )>21||duandian>10)//����
{
  
 //gpio_set(PTE1,1);
  if(pulse_get==dawan_speed)
  zhidao_ruwan_flag=0;
  dawan_flag=1;
speed_to_set=dawan_speed;
 }
 else
 {
  //gpio_set(PTE1,0);
   dawan_flag=0;
  // zhidao_ruwan_flag=0;
 speed_to_set=dawan_speed;
 }
 if(pulse_get>dawan_speed+10&&qulv_yuandaun>13&&qulv_jinduan<6)//����ֱ�������
 {
   straight_flag=0;
  // led(LED0, LED_OFF);//��ʾ�ر�
   dawan_flag=0;
   zhidao_ruwan_flag=1;
   speed_to_set=dawan_speed;//Ŀ���ٶ�ֱ�ӵ��ڴ����ٶ�
 }
}
