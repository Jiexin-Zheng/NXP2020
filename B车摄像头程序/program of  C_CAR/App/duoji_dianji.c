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
  gpio_init (PTD3 ,GPO,LOW);        //电机1--使能IO初始化
}



void get_pulse(void)//获取速度
{
    speed_a=-FTM2_CNT;
    FTM2_CNT=0;
}

  /*******************************************************12届电机正反转及差速子函数*****************************************************/  

void motor_backward (int zhankongbi)//zhankongbi 0-10000反转
{
ftm_pwm_duty(FTM0,FTM_CH4,0);  //电机
ftm_pwm_duty(FTM0,FTM_CH5,zhankongbi);   

}

void motor_foreward (int zhankongbi )//zhankongbi 0-10000正转
{
ftm_pwm_duty(FTM0,FTM_CH4,zhankongbi); // 电机
ftm_pwm_duty(FTM0,FTM_CH5,0);   


}


  /*******************************************************速度控制*****************************************************/  

int error_to_motor_pid,d_error_to_motor_pid,last_error_to_motor_pid,pulse_get=0;
int motor_kp,motor_kd;
int output_duty_to_motor_pid,last_output_duty_to_motor_pid,CS=11;
extern char pof,crossing_flag;
void motor_parameters_init(void)//电机参数初始化
{
error_to_motor_pid=0;
d_error_to_motor_pid=0;
last_error_to_motor_pid=0;
motor_kp=40;
motor_kd=25;
output_duty_to_motor_pid=0;
last_output_duty_to_motor_pid=0;
}
void motor_pid(int pulse_to_set)//速度PID控制
{
 
  pulse_get=speed_a;//
  
  //********************设置反转力度**************//

   int dut,limit;
   /*if(guai_flag)//圆环开始拐弯了
   {
   dut=4500;//反转力度为45%
    limit=0;//目标速度-实际速度
   }
   else*/
    if(zhangaijishileft_flag||zhangaijishiright_flag)//障碍
   {
    dut=1000;//反转力度为10%
    limit=-6;//目标速度-实际速度
   }else
    if(start_line_delay_counter>30&&pulse_get>6)//检测到起跑线后延时150ms后且当前速度大于6
    {
    dut=2000;//反转力度20%，反转最多为30%占空比，用于停车距离控制
    limit=-4;
    }
    else
    if(pof)//坡道
{
  dut=2000;
limit=-5;
}
    else
    if(straight_flag>20)//长直道
 {
   zhidao_ruwan_flag=0;
   dut=0;//3000
   limit=-10;
 }
    else
    if(zhidao_ruwan_flag)//直道入弯
  {
    dut=2900;//最大3000
    limit=0;
  }else//其他弯道
  {
  dut=1500;//1000
  limit=-10;
  }
  
  if(yunsu_flag&&!star_lineflag)//匀速
  {
  dut=1000;//1000
 limit=-10;
  }
  
  
  //***************电机控制pid公式****************************//
   error_to_motor_pid=pulse_to_set-pulse_get;//脉冲偏差=期望脉冲-实际脉冲
   d_error_to_motor_pid=error_to_motor_pid-last_error_to_motor_pid;
   last_error_to_motor_pid=error_to_motor_pid;
   output_duty_to_motor_pid=last_output_duty_to_motor_pid+motor_kp*error_to_motor_pid+motor_kd*d_error_to_motor_pid;
   if(output_duty_to_motor_pid>7000)
    output_duty_to_motor_pid=7000; 
   if(output_duty_to_motor_pid<0)
   output_duty_to_motor_pid=0; 
   last_output_duty_to_motor_pid=output_duty_to_motor_pid;
   
   
   if(error_to_motor_pid<limit)//实际速度和期望速度差值小于设定的门槛
   motor_backward (dut);//安照设定的占空比反转
    else 
      if(error_to_motor_pid>6)//如果期望速度比实际速度大6以上，就直接控制电机猛转加速
        motor_foreward (7500);//bangbang控制算法
    else
   motor_foreward (last_output_duty_to_motor_pid);//否则通过pid调节

}


void track_judge()//路况判断
{
   if(zhangaijishileft_flag||zhangaijishiright_flag)//障碍
   {
     speed_to_set=dawan_speed;//减速
     dawan_flag=0;
   }
   else
    if(pof)//坡道
{
 // gpio_set(PTE1,1);
  straight_flag=0;
speed_to_set=60;
}
else
if(S>25)//小s弯
 {
speed_to_set=S_speed;
 }else
  if(straight_flag>13)//长直道
  {
    speed_to_set=long_speed;      
      dawan_flag=0;
    // gpio_set(PTE1,1);
  }
  else
   
if(qvlv_quanju>12&&qvlv_quanju<=25&&abs(error_from_calculation )<=21&&abs(error_from_calculation )>=13)//小弯
{ 
  //gpio_set(PTE1,1);
speed_to_set=dawan_speed;
dawan_flag=0;
}
else
 if(qvlv_quanju>25||abs(error_from_calculation )>21||duandian>10)//大弯
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
 if(pulse_get>dawan_speed+10&&qulv_yuandaun>13&&qulv_jinduan<6)//高速直道入大弯
 {
   straight_flag=0;
  // led(LED0, LED_OFF);//提示关闭
   dawan_flag=0;
   zhidao_ruwan_flag=1;
   speed_to_set=dawan_speed;//目标速度直接等于大弯速度
 }
}
