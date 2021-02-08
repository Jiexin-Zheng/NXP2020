#include "include.h"
#include "common.h"
//uint8 img[CAMERA_W*CAMERA_H];                           //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
//uint8 imgyiwei[60][80];

int flag_stop_time,flag_stop_miaoshu;
int FCDC_flag_zhidao,FCDC_flag_querenzhidao,FCDC_time_querenzhidao,FCDC_flag_zhidao;
int flag_stopx,piancha,FCDC_flag_wandao,FCDC_podao_flag,FCDC_podao_speed,FCDC_zhidao_speed;
int FCDC_xiaowan,FCDC_huanwan_speed,FCDC_jiwan_speed,FCDC_xiaowan,FCDC_huanwan_speed,FCDC_huanwan_chasu,FCDC_jiwan_chasu;
              

void DSJ_speed_canshu(void)
{
FCDC_zhidao_speed=75;  //70    80 70  70  240 240 33  /90 80 75 240 240 35/
FCDC_huanwan_speed=75;//65
FCDC_jiwan_speed=75;//60
FCDC_huanwan_chasu=220;//
FCDC_jiwan_chasu=250;//
//FCDC_podao_speed=40;
DST_SXT_duoji_kp=360;
}




void speedcontrol(void)
{
  
  
             // if((GPIO_Get(PORTB,3)==0)&&(flag_stop_miaoshu>10))
       if((flag_stop_miaoshu>20))////////////////////////////////////////////////////////////////////////////////////////
              flag_stopx=1;
  
  
                flag_stop_time++;        //LED关闭 小车准备出发开始计时停车时间。
              if(flag_stop_time>200)     //1S 250
                   {
                       flag_stop_miaoshu++;
                       flag_stop_time=0;
                   }  
              
              
              if(FCDC_flag_zhidao==1&&FCDC_flag_querenzhidao==0)       //直道标志=1  确认直道=0；
                  FCDC_time_querenzhidao++;                             //time++  每4ms 加一次
              else
                  FCDC_time_querenzhidao=0;
         
         
              if(FCDC_time_querenzhidao>100&&FCDC_flag_zhidao==1)                        //400ms直道确认  之后 确认是直道
               {
                    FCDC_flag_querenzhidao=1;
               }
              
              
              
               piancha=danxian_piancha;
               // if(flag_stopx&&flag_stop)    
               if(flag_stopx)       //停车  液晶停车+达到停车要求（超过一定时间后且检测到起跑线）
                {
                          dianji_PID_zuo(0);    //6.28  --75     //65
                          dianji_PID_you(0);    //6.28  --64     //54
                }
               else
               {
                    if((danxian_piancha>-5)&&(danxian_piancha<5))// 直道  
                    {
                        FCDC_flag_zhidao=1;
                        FCDC_flag_wandao=0; //直道标志位为1，弯道为0
                        if(FCDC_flag_querenzhidao)//真正的长直道
                          {
                             if(FCDC_podao_flag)//坡道
                             {
                                dianji_PID_zuo(FCDC_podao_speed);    //6.28  --75     //65
                                dianji_PID_you(FCDC_podao_speed);    //6.28  --64     //54
                             }
                             else//长直道
                             { 
                               dianji_PID_zuo(FCDC_zhidao_speed);    //6.28  --75     //65
                               dianji_PID_you(FCDC_zhidao_speed);    //6.28  --64     //54
                           }
                          }
                        else//弯道中短暂的误判直道，速度按照弯道速度来。
                           {
                             if(FCDC_xiaowan)//缓弯
                                {
                                   dianji_PID_zuo(FCDC_huanwan_speed);    //6.28  --75     //65
                                   dianji_PID_you(FCDC_huanwan_speed);    //6.28  --64     //54
                                }
                            else//急弯
                                {
                                   dianji_PID_zuo(FCDC_jiwan_speed);    //6.28  --75     //65
                                   dianji_PID_you(FCDC_jiwan_speed);    //6.28  --64     //54
                                }
                           }
                          
                      
                    }
                    else                   //弯道
                    { 
                       if(danxian_piancha>=5)     //左转
                       {
                            FCDC_flag_zhidao=0;
                            FCDC_flag_wandao=1;    //弯道标志给1，直道标志给0
                           if(danxian_piancha<15)
                            {
                              FCDC_xiaowan=1; 
                             dianji_PID_zuo(FCDC_huanwan_speed-piancha*FCDC_huanwan_chasu/1000);    //6.28  --75     //65
                             dianji_PID_you(FCDC_huanwan_speed);  
                            }
                           else
                           {
                              FCDC_xiaowan=0; 
                               dianji_PID_zuo(FCDC_jiwan_speed-piancha*FCDC_jiwan_chasu/1000);    //6.28  --75     //65
                               dianji_PID_you(FCDC_jiwan_speed);  
                           }
                       }
                       else if(danxian_piancha<-5)  //右转
                       {
                            FCDC_flag_zhidao=0;
                            FCDC_flag_wandao=1;
                           if(danxian_piancha>-15)
                           {
                             FCDC_xiaowan=1;
                              dianji_PID_zuo(FCDC_huanwan_speed);    //6.28  --75     //65
                             dianji_PID_you(FCDC_huanwan_speed+piancha*FCDC_huanwan_chasu/1000); 
                           }
                           else
                           {
                             FCDC_xiaowan=1;
                              dianji_PID_zuo(FCDC_jiwan_speed);    //6.28  --75     //65
                          dianji_PID_you(FCDC_jiwan_speed+piancha*FCDC_jiwan_chasu/1000); 
                           }
                           
                       }
                    
                    }
                    
    
                 }  
              
              
              
              
              
              
              
}