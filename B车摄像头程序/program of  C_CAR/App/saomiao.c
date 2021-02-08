//���ߣ� 
//ʱ�䣺
#include "saomiao.h"
#include "common.h"
#include "include.h"
#include "math.h"
int qianzhan=29;
int kp=170,kd=40,error_from_calculation ,last_middle_position=40,currentzhongjianzhi=40,rightheixian_flag=0,leftheixian_flag=0,xielv;
int output_of_servo_pid, x,y,linshi_x,linshi_right_heixian,linshi_y,ruyuanhuan_flag=0;
extern u8 imgyiwei[60][80];
extern int current_middle_position[60],right_black_line[60],left_black_line[60],straight_flag;
int last_error_to_servo1,last_error_to_servo2,last_error_to_servo3,last_error_to_servo4,error_to_servo,flag_l=0,flag_r=0,linshi_left_heixian,B,left_boundary_flag[60],right_boundary_flag[60];
int the_first_point_of_crosssing,the_first_point_of_top_crossing,S_Z;
int crossing_both_boundaries_lose_flag,chazhi,xx1,xx2,crossing_enter_flag;
int duandian,duandianshu,duandianshu1,currentzhongjian_lk;
int xielv_flag;
int qulv_point=0,obstacle_right=0,obstacle_left=0;
int dian1,youshi_kuan_flag=0;
extern int star_lineflag,zhangaijishiright_flag,zhangaijishileft_flag;
char small_bend_flag,crossing_flag,yuanhuan_flag=0,guai_flag=0;
int qvlv_quanju,qulv_jinduan,qulv_yuandaun;
int qulv_jinduan_right=0,qulv_jinduan_left=0,qulv_yuandaun_right=0,qulv_yuandaun_left=0,qvlv_quanju_right=0,qvlv_quanju_left=0;
int half_value[60];
int yy1,yy2;
extern char S;
int kp1;
static int half_width_group[60]=//��ֱ���ϲ������
{
14,16,16,17,17,18,18,18,19,19,
20,20,21,21,22,22,23,23,23,25,
25,25,25,26,26,26,27,27,28,28,
28,29,29,30,30,30,31,31,32,33,
33,33,33,33,34,34,34,35,35,35,
36,36,36,36,37,37,38,38,38,38      
};
int regression(int startline,int endline)   //���Իع鷽�̼���б��      
{
  if(endline>56)
    endline=56;
  int i;
  int sumX=0,sumY=0,avrX=0,avrY=0 ;
   int num=0,B_up1=0,B_up2=0,B_up,B_down;
   for(i=startline;i<=endline;i++)
   {
            num++;
            sumX+=i;
            sumY+=current_middle_position[i];
   }
         avrX=sumX/num;
         avrY=sumY/num;
         B_up=0;
         B_down=0;
         for(i=startline;i<=endline;i++)
        {
         
            B_up1=(int)(current_middle_position[i]-avrY);
            B_up2=i-avrX;
            B_up+=(int)(10*(B_up1*B_up2));
            B_up=B_up/100*100;
            B_down+=(int)(10*((i-avrX)*(i-avrX)));
         }
   if(B_down==0) 
   B=0;
   else 
   B=B_up*16/B_down;//16pending
   return B;
}


int hy;
void straight_line_extension(int x1,int y1,int x2,int y2)//��������֮������һ���߶�
{
  int i,max,a1,a2;
  a1=x1;
  a2=x2;
  if(a1>a2)
  {max=a1;
  a1=a2;
  a2=max;}
for(i=x1;i>0;i--)
{
  if((x2-x1)!=0)
  {
    hy=(i-x1)*(y2-y1)/(x2-x1)+y1;
   current_middle_position[i]=hy;
   if(hy<=1||hy>=79)
     break;
  }
}
}
void huaxian2(int xxx1,int yyy1,int xxx2,int yyy2)//����֮����߶�
{
  int i2,max2,a12,a22;
  a12=xxx1;
  a22=xxx2;
  if(a12>a22)
  {
    max2=a12;
  a12=a22;
  a22=max2;}
for(i2=xxx1;i2<59;i2++)
{
  if((xxx2-xxx1)!=0)
  {
    hy=(i2-xxx1)*(yyy2-yyy1)/(xxx2-xxx1)+yyy1;
   current_middle_position[i2]=hy;
  }
}
}
void huaxian3(int xxx1,int yyy1,int xxx2,int yyy2)//����֮����߶�
{
  int i2,max2,a12,a22;
  a12=xxx1;
  a22=xxx2;
  if(a12>a22)
  {
    max2=a12;
  a12=a22;
  a22=max2;}
for(i2=xxx1;i2<xxx2;i2++)
{
  if((xxx2-xxx1)!=0)
  {
    hy=(i2-xxx1)*(yyy2-yyy1)/(xxx2-xxx1)+yyy1;
   current_middle_position[i2]=hy;
  }
}
}
//*****************************************************������ƺ���**********************************************************//
int middle_value_of_servo=4520;//�����ֵ������û�аڽ�  pending
void servo_control()//������ƺ���
{
output_of_servo_pid=(int)(kp1*error_to_servo/10-kd*(last_error_to_servo4-error_to_servo));//�����pid�㷨��ʽ��û��С��������kp1Ҫд�����ٳ���10��ʵ��С��
if(output_of_servo_pid>=1000)
output_of_servo_pid=1000;
if(output_of_servo_pid<=-1000)
output_of_servo_pid=-1000;
 ftm_pwm_duty(FTM1,FTM_CH0,middle_value_of_servo+output_of_servo_pid); //633��
} 
//***************************************************ɨ�躯��***************************************************************//
void scan_from_middle()
{
  
  int kuan_flag=0,difference_of_width_for_obstacle_judging=0,small_bend_breakpoint =0,S_COUNT=0,fuduandian=0,qulv_yuanhuan=0,leave_crossing=0;
  memset(left_boundary_flag,0,sizeof(left_boundary_flag));//���㺯��
  memset(right_boundary_flag,0,sizeof(right_boundary_flag));
  int obstacle_flag=0,right_curvature_of_small_bend=0,left_curvature_of_small_bend=0;
  the_first_point_of_top_crossing=0;
  left_black_line[59]=0;
  right_black_line[59]=79; 
  crossing_both_boundaries_lose_flag=0;
  crossing_enter_flag=0;
  xielv_flag=0;
  crossing_flag=0;
  last_middle_position=39;
  if(current_middle_position[58])//pending
   current_middle_position[59]=last_middle_position=current_middle_position[58];
 else
   current_middle_position[59]=39;
   for(y=58;y>=0;y--)//ɨ����58������ͼ�������
   {
     for(x=last_middle_position;x<=79;x++)//���м�����ɨ��
     {
       if(imgyiwei[y][x]==0)
         {
           
         right_black_line[y]=x;
         right_boundary_flag[y]=1; 
         break;
         }
     }
     for(x=last_middle_position;x>=0;x--)//����ɨ��
     { 
       if(imgyiwei[y][x]==0)
       {
         left_black_line[y]=x;
         left_boundary_flag[y]=1;
         break;
       }  
     }
//    if(fuduandian==0)//pending���ж���������δ�������߼��Ͽ�û�д����ã���ע�͵�
//    {
    if(y>48)//���߲��ߣ���������������
      {
        if(right_boundary_flag[y]==0&&left_boundary_flag[y]==1)//ɨ������
         { 
          right_black_line[y]=left_black_line[y]+2*half_width_group[y];//yΪ�����е�xֵ
         }
        else if(right_boundary_flag[y]==1&&left_boundary_flag[y]==0)//ɨ������
         {
           left_black_line[y]=right_black_line[y]-2*half_width_group[y];
         }
       else if(right_boundary_flag[y]==0&&left_boundary_flag[y]==0)//��ɨ����
         {
          left_black_line[y]=0;//������߶�ɨ������ֱ�Ӵ��м���ȡ����
          right_black_line[y]=79;
         }
     }
     else if(y<=48)//��Զ�Ĳ��ߣ�
     {
       if(right_boundary_flag[y]==0&&left_boundary_flag[y]==1)//ɨ������
         { 
     
          right_black_line[y]=right_black_line[y+1]+(abs)(left_black_line[y]-left_black_line[y+1])-1;//���������һ������һ�㣬
                                                                                        //��y+1��Ϊ��������һ�ε�x���꣬
                                                           //left_black_line[y]-left_black_line[y+1]�����ƫ��������һΪ���ڲ���,pending
         }
       else if(right_boundary_flag[y]==1&&left_boundary_flag[y]==0)//ɨ������
         {
       
           left_black_line[y]=left_black_line[y+1]-(abs)(right_black_line[y+1]-right_black_line[y])+1;//absΪȡ����ֵ
         }
         
       else if(right_boundary_flag[y]==0&&left_boundary_flag[y]==0)//��ɨ����
         {
          left_black_line[y]=0;
          right_black_line[y]=79;
         }
     }  
     //*************************************���ߴ���*********************************************************//
   
    current_middle_position[y]=(int)((right_black_line[y]+left_black_line[y])/2);  //��ȡ�м��ߣ���߼��ұ߱�2��
  
     half_value[y]=(int)((right_black_line[y]-left_black_line[y])/2);  //һ������ֵ��û�ã����ģ�pending
    
      //*******************************************************�����˲���ֹ���߳���ë��**************************************************/
  
if(current_middle_position[y]-current_middle_position[y+1]>4&&y<42&&B<0)//pending
current_middle_position[y]=current_middle_position[y+1]+1;
if(current_middle_position[y]-current_middle_position[y+1]<-4&&y<42&&B>0)
current_middle_position[y]=current_middle_position[y+1]-1;  


/******************************************************************���ʼ���**************************************************************/
if(y>30)//���ʽ����ж�
{
  if((current_middle_position[y]-current_middle_position[y+1])>0)
qulv_jinduan_right++;
else
if((current_middle_position[y]-current_middle_position[y+1])<0)
qulv_jinduan_left++;
} 


if(y<25&&y>=7)//����Զ���жϣ�Զ����һ��Ͳ�Ҫ��
{
  if((current_middle_position[y]-current_middle_position[y+1])>0)
qulv_yuandaun_right++;
else if((current_middle_position[y]-current_middle_position[y+1])<0)
qulv_yuandaun_left++;
}
if(y<=55&&y>10)//����ȫ���ж�
{
  if((current_middle_position[y]-current_middle_position[y+1])>0)
qvlv_quanju_right++;
else
if((current_middle_position[y]-current_middle_position[y+1])<0)
qvlv_quanju_left++;
}   

if(current_middle_position[y]>79)//���ߵ����Ʒ���
current_middle_position[y]=79;
if(current_middle_position[y]<0)
current_middle_position[y]=0;

last_middle_position=current_middle_position[y];//�����м������
if(y<56)//Сsλ���жϣ�Сs�����ֱ�߳��//pending
{
if((current_middle_position[y]-current_middle_position[y+2])>0)
right_curvature_of_small_bend++;
else
if((current_middle_position[y]-current_middle_position[y+2])<0)
left_curvature_of_small_bend++;
if(y>36&&abs(current_middle_position[y]-current_middle_position[y+2])>0)
qulv_yuanhuan++;
//printf("y:%d\n",right_curvature_of_small_bend);
//printf("l:%d\n",left_curvature_of_small_bend);
}
if(y<58&&y>=10)
S_COUNT+=current_middle_position[y];//pendingû�õ�
// *******************************************СS�ϵ�����*********************************************//pending
 if(y>8&&(abs)(right_black_line[y]-current_middle_position[y])<5||(abs)(left_black_line[y]-current_middle_position[y])<5)//СS�ϵ��ж�
  small_bend_breakpoint =y;//
else
small_bend_breakpoint =0;

//****************************************************�ϰ��ж�****************************************************************//
if(y<52&&y>5&&qvlv_quanju<11&&duandian<5&&qulv_jinduan<=4&&dian1<9)
{
   if(kuan_flag<=21&&obstacle_flag<2&&2*half_value[y]>2*half_width_group[y]-5)//������
   {
   kuan_flag++;
  //pl+=current_middle_position[y];
   }else
if(2*half_width_group[y]-2*half_value[y]>8&&kuan_flag>17)//������խ
{
 obstacle_flag++;
if(obstacle_flag==6)
difference_of_width_for_obstacle_judging=(current_middle_position[y]+current_middle_position[y+1])/2-(current_middle_position[55]+current_middle_position[54])/2;
}
 if(obstacle_flag>7&&kuan_flag>17&&2*half_value[y]>2*half_width_group[y]-8)//������
 {
   youshi_kuan_flag++;
 }
 if(youshi_kuan_flag>=4)
 {
  if(difference_of_width_for_obstacle_judging>2)
  {
    obstacle_left=1;
    obstacle_right=0;
  }
  else
     if(difference_of_width_for_obstacle_judging<-2)
     {
      
    obstacle_right=1;
    obstacle_left=0;
     }
   youshi_kuan_flag=0;
 }
   
}
  // *******************************************��ֹɨ���ܵ���*********************************************//
  if(y<36&&((abs)(right_black_line[y]-current_middle_position[y])<1||(abs)(left_black_line[y]-current_middle_position[y])<1))//��ֹɨ���ܵ�����
{
  duandian=y;//pending
  fuduandian=y;
  if(y>15)
  {
   duandianshu=y-15;
   if(duandianshu>35)
     duandianshu=35;
  }
     ///////// *********************************************************** ////////////////
     ///////// *********************************************************** //////////////// 
 if(y>qianzhan)//pending
  {
     duandianshu1=y-qianzhan;
     if(duandianshu1>25)
       duandianshu1=25;
  }
 // if(DSYJ_y==28)
   //  DSYJ_currentzhongjian[DSYJ_y]=79;
 //break;
}
else 
{
  duandian=0;
  duandianshu=0;
  duandianshu1=0; 
}
//    }
     }
 /*if(right_black_line[y]-left_black_line[y]<half_width_group[y]&&right_black_line[y+1]-left_black_line[y+1]<half_width_group[y+1])//��ֹɨ���ܵ�����
       break;*/
//}
//********************************************************y�н�����־*******************************************************

S_Z=S_COUNT/(48);//pendingû�õ�

//********************************************************************���ʼ���*******************************************************************
   qvlv_quanju=abs(qvlv_quanju_right-qvlv_quanju_left);//����ȫ��
   qulv_jinduan=abs(qulv_jinduan_right-qulv_jinduan_left);//���ʽ���
   qulv_yuandaun=abs(qulv_yuandaun_right-qulv_yuandaun_left);//����Զ��
   qvlv_quanju_right=qvlv_quanju_left=qulv_jinduan_right=qulv_jinduan_left=qulv_yuandaun_right=qulv_yuandaun_left=0;


// *******************************************************ʮ�ִ���*********************************************************//

  if(duandian<16) // *****************************ʮ��//pending
  {
   for(int i=58;i>7;i--)
   { 
     if(i>45&&(left_boundary_flag[i]==1||right_boundary_flag[i]==1))
     crossing_enter_flag++;//ͼ��ײ�����һ�߽߱綪ʧ
     if((i>9&&i<=40)&&left_boundary_flag[i]==0&&right_boundary_flag[i]==0)
       crossing_both_boundaries_lose_flag++;//ͼ���в����߽߱綼��ʧ
     if(i<45&&(left_boundary_flag[i]==1||right_boundary_flag[i]==1)&&the_first_point_of_top_crossing==0)
       the_first_point_of_top_crossing=i;
   }
   if(crossing_both_boundaries_lose_flag>2&&crossing_enter_flag>10)//����ʮ��
   {
     crossing_flag=1;
     //gpio_set(PTE1,1);
     xielv_flag=1;
    for(int i=49;i>0;i--)
    {      
     if(left_boundary_flag[i]==0&&right_boundary_flag[i]==0)
      {
        the_first_point_of_crosssing=i;
        break;
      }     
     }
     if(the_first_point_of_crosssing<34)
       {
      xx1=the_first_point_of_crosssing+19;
      xx2=the_first_point_of_crosssing+24;
    }
    else 
    if(the_first_point_of_crosssing<40)
       {
      xx1=the_first_point_of_crosssing+13;
      xx2=the_first_point_of_crosssing+18;
    }
    else if(the_first_point_of_crosssing<45)
    {
      xx1=the_first_point_of_crosssing+9;
      xx2=the_first_point_of_crosssing+13;
    }
    else if(the_first_point_of_crosssing<49)
    {
    xx1=the_first_point_of_crosssing+6;
    xx2=the_first_point_of_crosssing+10;
    }else
      if(the_first_point_of_crosssing<55)
    {
    xx1=the_first_point_of_crosssing+2;
    xx2=the_first_point_of_crosssing+4;
    }
    yy1=(current_middle_position[xx1]+current_middle_position[xx1+1]+current_middle_position[xx1-1])/3;
    yy2=(current_middle_position[xx2]+current_middle_position[xx2+1]+current_middle_position[xx2-1])/3;
    //if(!ruyuanhuan_flag)
    straight_line_extension(xx1,yy1,xx2,yy2); 
      
    //straight_line_extension(xx1,current_middle_position[xx1],xx2,current_middle_position[xx2]); 
    }
else if(crossing_both_boundaries_lose_flag>4&&crossing_enter_flag<=10&&the_first_point_of_top_crossing>=20)//��ʮ��
  {
    leave_crossing=1;
  // gpio_set(PTE1,1);
    xielv_flag=1;
    crossing_flag=1;
 //if(!ruyuanhuan_flag)
   huaxian2(the_first_point_of_top_crossing-6,current_middle_position[the_first_point_of_top_crossing-6],58,current_middle_position[58]);

  }
       //  gpio_set(PTE1,0);

  } 
  //********************************************************Բ�����*******************************************************
/*if(crossing_flag&&qulv_yuanhuan<=10&&qulv_jinduan<7&&!leave_crossing)
{
  
 signed char kkk=0,black_num=0;
  for( kkk=(int)(current_middle_position[58]+current_middle_position[57]+current_middle_position[56])/3;kkk<=79;kkk++)//����ɨ��ڿ����
  {
    if(!imgyiwei[8][kkk]||!imgyiwei[10][kkk])
    {
    black_num++;
   
    }
    else
      break;
    
  }
   for( kkk=(int)(current_middle_position[58]+current_middle_position[57]+current_middle_position[56])/3;kkk>=0;kkk--)//����ɨ��ڿ����
   {
      if(!imgyiwei[8][kkk]||!imgyiwei[10][kkk])
    {
    black_num++;
   
    }
    else
      break;
   }
  
  if(black_num>36)//��⵽Բ��
  {
    yuanhuan_flag++;
    //gpio_set(PTC18,0);//����led��ʾ
  }
  else
  {
    yuanhuan_flag=0;
    //gpio_set(PTC18,1);////�ر�led��ʾ
  }
 
}
else
{
yuanhuan_flag=0;
 //gpio_set(PTC18,1);//�ر�led��ʾ
}
  
if(yuanhuan_flag>3)//������⵽Բ����������
{
guai_flag=1;
//gpio_set(PTA14,1);
}

if(guai_flag&&guai_flag<=30)//������ٷ�ת��ʱ��
{
guai_flag++;
if(guai_flag&&guai_flag<=30)//���ӹ��䱣�ֵ�ʱ��
{
//huaxian3(9,75,the_first_point_of_crosssing,current_middle_position[the_first_point_of_crosssing+4]);
ruyuanhuan_flag=1;
}
}
else
guai_flag=0;*/
// ***********************************************************СS���ж�*СS���ж�*****************************************************//
if(small_bend_breakpoint ==0&&right_curvature_of_small_bend>6&&left_curvature_of_small_bend>6&&straight_flag<20&&!crossing_flag)//pending
{
  small_bend_flag=1;
  //led(LED0, LED_ON);
}
else
{
small_bend_flag=0;
//led(LED0, LED_OFF);
}
 }
//************************************************ͼ�������,������ƫ���************************************************/



void error_handle()//ƫ���
{
    if(xielv_flag==0)
    regression(15+duandianshu,43+duandianshu);///б�ʼ���
  dian1=abs((current_middle_position[52]+current_middle_position[53]+current_middle_position[54])/3-39);
//lins=current_middle_position[30];
currentzhongjian_lk=(current_middle_position[qianzhan+duandianshu1]-39);//��

if(zhangaijishiright_flag)//�ϰ���ƫ���
{
currentzhongjian_lk-=5;
B=0;
}
else if(zhangaijishileft_flag)
{
  currentzhongjian_lk+=5;
  B=0;
}
if(B>19)
B=19;
if(B<-19)
B=-19;

error_from_calculation =(int)(1.1*currentzhongjian_lk-0.9*B);//0.95,0.32//ƫ��ϳɣ�����ת�䣬1.1���ĵ�+0,9����б�ʹ����ܵ�ƫ��
/*if(!ruyuanhuan_flag&&ruyuanhuan_flag<10)//��50ms
{
  ruyuanhuan_flag++;
if(error_from_calculation <-4&&crossing_flag)//�ұ߽���Բ��
error_from_calculation +=13;
else
if(error_from_calculation >4&&crossing_flag)//��߽���Բ��
error_from_calculation -=13;
}
else
ruyuanhuan_flag=0;*/

if(S>25)//S��  pending
{
  kp1=110;
  //B=0;
error_from_calculation =current_middle_position[16]-39;
}
else
if(abs(error_from_calculation )<5)//ֱ���ϣ�ƫ�����ֵС��5  pending
{
kp1=kp-20;//ֱ���ϵ�P��Сһ�㣬��ֱֹ������
}
else
kp1=kp;//����ֱ��P��Ҫ��һ�㡣������������ȣ��������ڲࡣ



B=0;
if(star_lineflag)//��⵽������
kp1=80;
last_error_to_servo4=last_error_to_servo3;
last_error_to_servo3=last_error_to_servo2;
last_error_to_servo2=last_error_to_servo1;
last_error_to_servo1=error_to_servo;
error_to_servo=error_from_calculation ; 
servo_control();//������ƺ���









/*****************************************/
/*
  if(gryoscope_pitch > 30){
    ramp_flag = 1;
    motor_kp += 10;
  }
  else if(gryoscope_pitch < -30){
    ramp_flag = 1;
    motor_kp += 10;
  }
  else{
    ramp_flag = 0;
  }

*/




















