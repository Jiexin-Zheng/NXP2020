//作者：
//说明：saomiao文約can_from_middleansaomiao（）函数是图像处理函数，偏差处理以及舵机控制均在此文件下
//duoji_dianji文件下都是速度控制的相关程序，核心程序就是这两个文件
#include "common.h"
#include "include.h"
#include "math.h"
#include "saomiao.h"
uint8 imgbuff[CAMERA_SIZE];     
 int current_middle_position[60];
 int right_black_line[60];
 int left_black_line[60];
int quanjuCount;
int OutData[4],tuoluoyivar;
int star_lineflag=0;
int five_ms_counter=0;
int start_line_judge_flag=0;
int start_line_delay_counter=0,straight_flag,yunsu_flag,yuzhi;
extern int podao_flag,obstacle_right,obstacle_left,qvlv_quanju,duandian,speed_to_set,dawan_speed,qulv_jinduan;
extern void  track_judge();
char go=0,S1,S,zhichong_flag;
extern char lcd_show_enable1;

//s16 speed_a;
extern int error_from_calculation ;
int zhangaijishiright_flag=0,zhangaijishileft_flag=0;
//int kp=22,kd=10,right_black_line[60],last_error_to_servo1,error_to_servo,left_black_line[60],last_middle_position=40,currentzhongjianzhi=40,rightheixian_flag,leftheixian_flag,xielv;
//函数声明
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void PIT0_IRQHandler();
void Priority_Set();
extern char small_bend_flag;
 char pof;
//void PIT1_IRQHandler();
/*!
 *  @brief      main函数
 *  @since      v5.3
 *  @note       山外 DMA 采集摄像头 实验
                注意，此例程 bus频率设为100MHz(50MHz bus频率会太慢而导致没法及时采集图像)

                此例程使用的上位机为【山外多功能调试助手】
                具体资料请参考：山外多功能调试助手资料专辑
                                http://www.vcan123.com/forum.php?mod=collection&action=view&ctid=27
 */
void hecheng()//虚拟中线函数
{
unsigned int ji;
for(ji=0;ji<=59;ji++)
      {
       imgyiwei[ji][current_middle_position[ji]]=0 ;
      }
}
void sendimg(uint8 *imgaddr,uint32 imgsize)
{
img_extract((u8 *)imgyiwei, imgbuff,CAMERA_SIZE);      //解压图像
scan_from_middle();
uint8 cmd[4] = {0, 255, 1, 0 };    //yy_摄像头串口调试 使用的命令
hecheng();
uart_putbuff(VCAN_PORT, cmd, sizeof(cmd));    //先发送命令
uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
}




void star_line_judg()//起跑线检测   pending
{
 int kk,white_flag=0,black_flag=0,black_to_white_flag=0,white_to_black_flag=0;
  for(kk=5;kk<=72;kk++)
  {
    if(imgyiwei[45][kk]>0)
      white_flag=1;
    else
    if(white_flag&&imgyiwei[45][kk]==0)
    {
      white_to_black_flag++;
      white_flag=0;
    }
    
    if(imgyiwei[45][kk]==0)
      black_flag=1;
    else
    if(black_flag&&imgyiwei[45][kk]>0)
    {
      black_to_white_flag++;
      black_flag=0;
    }
      
  }
  if(white_to_black_flag>=4&&black_to_white_flag>=4&&white_to_black_flag-black_to_white_flag<=2)
    star_lineflag=1;
}


void lkzhongjian()
{
  unsigned int i;
 for(i=0;i<=59;i++)
  { 
    imgyiwei[i][current_middle_position[i]]=0;

  }
 
 
}
/*void sendimg(uint8 *imgaddr,uint32 imgsize)
{
 
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_摄像头串口调试 使用的命令
//hecheng()；
    uart_putbuff(VCAN_PORT, cmd, sizeof(cmd));    //先发送命令

   uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像
       // uart_putbuff(VCAN_PORT, current_middle_position, 60); //再发送图像
}*/
//**************************************************************************
/*
*  功能说明：SCI示波器CRC校验
内部调用函数
*  参数说明： 无
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//**************************************************************************
static unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++)
    {
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

//************************************************
//
/*
*  功能说明：SCI示波器发送函数

*  参数说明：
OutData[]  需要发送的数值赋予该数组
*  函数返回：无符号结果值
*  修改时间：2013-2-10
*/
//****************************************************
void OutPut_Data(void)
{   
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {
        temp[i]  = (int)OutData[i];
        temp1[i] = (unsigned int)temp[i];

    }

    for(i=0;i<4;i++)
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*  2+1] = (unsigned char)(temp1[i]/256);
    }

    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;

    for(i=0;i<10;i++)
    {
        uart_putchar (UART0,(char)databuf[i]);
    }
}


void SendHex(unsigned char hex) {
  unsigned char temp;
  temp = hex >> 4;
  if(temp < 10) {
    uart_putchar(UART0,temp + '0');
  } else {
    uart_putchar(UART0,temp - 10 + 'A');
  }
  temp = hex & 0x0F;
  if(temp < 10) {
    uart_putchar(UART0,temp + '0');
  } else {
   uart_putchar(UART0,temp - 10 + 'A');
  }
}

void SendImageData(unsigned char ImageData[][80]) 
{
  

    int lll1,lll2;
    unsigned char crc = 0;
scan_from_middle();
lkzhongjian();
    /* Send Data */
    uart_putchar(UART0,'*');
    uart_putchar(UART0,'L');
    uart_putchar(UART0,'D');

    SendHex(0);
    SendHex(0);
    SendHex(0);
    SendHex(0);
    
  //  imgyiwei[60][80]; 

    for(lll2=0;lll2<80;lll2++)
    {
      
      for(lll1=0;lll1<60; lll1++)
        SendHex(ImageData[lll1][lll2]);
    }
    
    
   /* for(ll1=0;ll1<60;ll1++)
 {
  for(ll0=0;ll0<80;ll0++) 
  {
   if(imgyiwei[ll1][ll0]==0)
      Draw_potL(ll0,ll1,0,1);
    else
      Draw_potL(ll0,ll1,1,1);
  }
 }*/
    

    SendHex(crc);
    uart_putchar(UART0,'#');
}
#if 1
//编码器初始化
void FTM_QUAD_init()
{
    /*开启端口时钟*/
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

    /*选择管脚复用功能*/
   // PORTA_PCR12 = PORT_PCR_MUX(7);
   // PORTA_PCR13 = PORT_PCR_MUX(7);
    PORTA_PCR10 = PORT_PCR_MUX(6);
    PORTA_PCR11 = PORT_PCR_MUX(6);

    /*使能FTM1、FTM2时钟*/
   // SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;
    SIM_SCGC3|=SIM_SCGC3_FTM2_MASK;

  //  FTM1_MOD = 65535; //可根据需要设置
    FTM2_MOD = 65535;

  //  FTM1_CNTIN = 0;
    FTM2_CNTIN = 0;

  //  FTM1_MODE |= FTM_MODE_WPDIS_MASK; //禁止写保护
    FTM2_MODE |= FTM_MODE_WPDIS_MASK; //禁止写保护
  //  FTM1_MODE |= FTM_MODE_FTMEN_MASK; //FTMEN=1,关闭TPM兼容模式，开启FTM所有功能
    FTM2_MODE |= FTM_MODE_FTMEN_MASK; //FTMEN=1,关闭TPM兼容模式，开启FTM所有功能

  //  FTM1_QDCTRL &= ~FTM_QDCTRL_QUADMODE_MASK; //选定编码模式为A相与B相编码模式
  //  FTM1_QDCTRL |= FTM_QDCTRL_QUADEN_MASK; //使能正交解码模式
  //  FTM2_QDCTRL &= ~FTM_QDCTRL_QUADMODE_MASK; //选定编码模式为A相与B相编码模式   0x8u  0x00001000取反即 0x11110111
   FTM2_QDCTRL |= 0x08;//~0x00u; 
    
    FTM2_QDCTRL |= FTM_QDCTRL_QUADEN_MASK; //使能正交解码模式
//QUADMODE=1;
 //   FTM1_SC |= FTM_SC_CLKS(3);  //选择外部时钟
//   FTM1_CONF |=FTM_CONF_BDMMODE(3); //可根据需要选择
    FTM2_SC |= FTM_SC_CLKS(3);
//   FTM2_CONF |=FTM_CONF_BDMMODE(3);

}
#endif
//**************************************************************************
void  main(void)
{
    DisableInterrupts; //all the initialization  are pending 
    

    motor_parameters_init();

    Priority_Set();
    camera_init(imgbuff);                                  //这里设定  imgbuff 为采集缓冲区！！！！！！

    
    ftm_pwm_init(FTM0,FTM_CH4,10000,0);  //电机初始化
    ftm_pwm_init(FTM0,FTM_CH5,10000,0);
    
    
    ftm_pwm_init(FTM1,FTM_CH0,300,4495);//舵机初始化
       
    ftm_quad_init(FTM2);//编码器初始化

   
    pit_init_ms(PIT0,5);//定时器中断5ms
     my_lcd_init();//液晶初始化
    
    set_vector_handler(PIT0_VECTORn,PIT0_IRQHandler);   // 设置中断服务函数到中断向量表里
    enable_irq(PIT0_IRQn);                             // 使能PIT中断
    set_vector_handler(PORTA_VECTORn,PORTA_IRQHandler);    //设置PORTA的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn,DMA0_IRQHandler);      //设置DMA0的中断服务函数为 DMA0_IRQHandler
    EnableInterrupts;	 
  //  adc_init (ADC1_SE4a);
  
//gpio_init(PTE28,GPO,0);//off

  
    while(1)
    { 
  //    gpio_init(PTE28,GPO,0);//off
      camera_get_img();//pending
     // my_lcd_show();
     if(start_line_judge_flag==1)//pending
      star_line_judg();
   // lptmr_delay_ms(1000);
    // gpio_init(PTE28,GPO,1);//off
//sendimg((u8 *)imgyiwei, CAMERA_W * CAMERA_H);//我的上位机，不注释为向电脑发送图像
    }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()//pending
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
//void PIT1_IRQHandler()
//{
//shizi_count++;
//}
void DMA0_IRQHandler()//pending
{
    camera_dma();
    img_extract((u8 *)imgyiwei, imgbuff,CAMERA_SIZE); 
          

}

void PIT0_IRQHandler(void)//定时器中断服务函数
{
 //lcd_show_enable1=0;//注释掉是调用液晶
PIT_Flag_Clear(PIT0);
  if(lcd_show_enable1)//为0跳出液晶
  {
      my_lcd_show();
  }
  else
  {
     /*****************************************************脉冲提取*************************************************************/     
  get_pulse();//获取电机转速 
  
     if(start_line_judge_flag==0)
      five_ms_counter++;
        if(five_ms_counter==300)//起步延时1.5秒pending
      go=1;//小车前进

    
    if(five_ms_counter>=2000)//10s后才开始检测起跑线
        {
          start_line_judge_flag=1;
          five_ms_counter=2000;
        }



/*****************************************************边线提取*************************************************************/  

scan_from_middle();//图像处理

   if(star_lineflag==1)//检测到起跑线后开始延时
        start_line_delay_counter++;
   
       if(!star_lineflag&&go)
     {
       if(yunsu_flag==1)
      speed_to_set=dawan_speed;//pending
      else
        track_judge();//路况判断       
     }
       else
       if(start_line_delay_counter>30)//检测到起跑线，并延时150ms
         speed_to_set=0; 
      //if(pof)
        //speed_to_set=0;
     motor_pid(speed_to_set); //控制电机转速
  
/************************************************************************S弯判定****************************************************/  
if(small_bend_flag)
{
    S1=1;
    //small_bend_flag=0;
}
else
{
S=0;
S1=0;
}
    
    if(S1&&S<=25)
      S++;

   if(S>=25)//pending 推测开跑前有一段时间可以试跑调试
   {
     zhichong_flag=1;
     //gpio_set(PTE1,1);
   }
   else
   {
     zhichong_flag=0;
     //gpio_set(PTE1,0);
   }
   /*******************************************************坡道处理*****************************************************/  
   // if(abs(error_from_calculation )<=16&&qvlv_quanju<=10&&duandian<12&&qulv_jinduan<12)
    // tuoluoyivar=adc_once(ADC1_SE4a,ADC_8bit);
   // else
     // tuoluoyivar=104;
    
  /*  if(abs(tuoluoyivar-104)>40&&!podao_flag)
    {
      podao_flag=1;
     // gpio_set(PTE1,1);
    }
    if(podao_flag&&podao_flag<800)
    {
podao_flag++;

    }
else
{
podao_flag=0;
//gpio_set(PTE1,0);
}

if(podao_flag>0&&podao_flag<130)
pof=1;
else
pof=0;*/
/*******************************************************障碍处理*****************************************************/  

    if(obstacle_right==1) 
    {
  zhangaijishiright_flag=1;
  obstacle_right=0;
    }
  else if(obstacle_left==1)
  {
 zhangaijishileft_flag=1;
   obstacle_left=0;
  } 
 
  
  if(zhangaijishileft_flag&&zhangaijishileft_flag<35)
  {
   zhangaijishileft_flag++;
   // led(LED0, LED_ON);
  }
  else
    if(zhangaijishiright_flag&&zhangaijishiright_flag<35)
    {
    zhangaijishiright_flag++;
    }
  else 
    zhangaijishiright_flag=zhangaijishileft_flag=0;
/************************************************************************直道判定*直道判定*****************************************************/  

    if(abs(error_from_calculation )<=13&&qvlv_quanju<=13&&duandian<10)
    {
      straight_flag++;      
    }
    else
    {
      straight_flag=0;
    }
    //gpio_turn(PTA17);
 /*****************************************************偏差处理*************************************************************/  

  error_handle();
    
  }

     
}

void Priority_Set(void)
{
 // NVIC_SetPriorityGrouping(4);            //设置优先级分组,4bit 抢占优先级,没有亚优先级

  NVIC_SetPriority(PORTA_IRQn,0);         //配置优先级
  NVIC_SetPriority(DMA0_IRQn,1);          //配置优先级
  NVIC_SetPriority(PIT0_IRQn,2);          //配置优先级

}

/*
variables'name:



resources allocated:

*/