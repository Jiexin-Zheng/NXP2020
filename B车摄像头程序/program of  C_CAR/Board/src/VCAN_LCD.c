/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       VCAN_LCD.c
 * @brief      LCD 函数库
 * @author     山外科技
 * @version    v5.0
 * @date       2013-06-26
 */


#include "common.h"
#include "VCAN_LCD.h"
extern int kp1,kd,S_speed,middle_value_of_servo,current_middle_position[60];
Site_t site = {3,5};
char    enter_flag=0,lcd_show_enable1=1;
 int  site_flag=0;
 int BCOLOUR_NEW[8]={BCOLOUR,BCOLOUR,BCOLOUR,BCOLOUR,BCOLOUR,BCOLOUR,BCOLOUR,BCOLOUR};
 extern uint8 imgbuff[60*80];
 extern uint8 imgyiwei[60][80]; 
 extern void scan_from_middle();
/*!
 *  @brief      LCD初始化
 *  @since      v5.0
 */
void LCD_init(void)
{
    Site_t site = {0, 0};
    Size_t size ;

    LCD_INIT();                             //初始化LCD

    //LCD_SET_DIR(LCD_DIR+1);

    //由于初始化的时候进行 屏幕方向 选择，因而初始化完成后，才获取宽高
    size.W = LCD_W;
    size.H = LCD_H;

    LCD_rectangle(site, size, BCOLOUR);     //初始化背景
}

/*!
 *  @brief      显示实心矩形
 *  @param      site    左上角坐标
 *  @param      size    矩形大小
 *  @param      rgb565  颜色
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        Size_t size = {50,60};  // W = 50 ,H = 60
                        LCD_rectangle(site, size, RED);
 */
void LCD_rectangle(Site_t site, Size_t size, uint16 rgb565)
{
    uint32 n, temp;

    LCD_PTLON(site, size);              //开窗

    temp = (uint32)size.W * size.H;
    LCD_RAMWR();                        //写内存
    for(n = 0; n < temp; n++)
    {
        LCD_WR_DATA( rgb565 );          //写数据
    }
}

void LCD_clear(uint16 rgb565)     //清屏
{
    Site_t site = {0,0};
    Size_t size;
    size.H = LCD_H;
    size.W = LCD_W;

    LCD_rectangle(site, size, rgb565);

}

//画十字形
void LCD_cross(Site_t site,uint16 len,uint16 Color)
{
    //画两条直线
    Site_t sitetmp;
    Size_t size;
    int16  stmp,etmp;
    uint16 w = LCD_W,h = LCD_H;

    ASSERT((site.x<LCD_W) && (site.y<LCD_H));       //十字线的中心不能超出液晶范围

    stmp = site.x - len/2;                          //计算水平线的x轴起点和结束点，限制不超出液晶范围
    if(stmp < 0)stmp = 0;
    etmp = site.x + len/2;
    if(etmp >= w)etmp= w-1;

    sitetmp.x = stmp;
    sitetmp.y = site.y;
    size.W = etmp - stmp +1;
    size.H = 1;
    LCD_rectangle(sitetmp,size, Color);

    stmp = site.y - len/2;                          //计算水平线的x轴起点和结束点，限制不超出液晶范围
    if(stmp < 0)stmp = 0;
    etmp = site.y + len/2;
    if(etmp >= h)etmp= h-1;

    sitetmp.x = site.x;
    sitetmp.y = stmp;
    size.W = 1;
    size.H = etmp - stmp +1;
    LCD_rectangle(sitetmp,size, Color);



}

/*!
 *  @brief      画点
 *  @param      site    左上角坐标
 *  @param      rgb565  颜色
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_point(site, RED);
 */
void LCD_point(Site_t site, uint16 rgb565)
{
    Size_t size = {1, 1};
    LCD_PTLON(site, size);
    LCD_RAMWR();                        //写内存
    LCD_WR_DATA(rgb565);                //写数据
}

/*!
 *  @brief      画一堆点
 *  @param      site        点坐标数组
 *  @param      point_num   点的数量
 *  @param      rgb565      颜色
 *  @since      v5.0
*  Sample usage:        Site_t site[3] = {{10,20},{11,21},{12,22}};   //3个点，坐标分别是  {10,20},{11,21},{12,22}
                        LCD_points(site,3, RED);
 */
void LCD_points          (Site_t *site,uint32 point_num, uint16 rgb565)                 //画一堆点
{
    while(point_num--)
    {
        LCD_point(site[point_num],rgb565);                 //画点
    }
}

/*!
 *  @brief      显示字符
 *  @param      site    左上角坐标
 *  @param      ascii   字符
 *  @param      Color   字体颜色
 *  @param      bkColor 背景颜色
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_char(site,'0', BLUE,RED);
 */
void LCD_char(Site_t site, uint8 ascii, uint16 Color, uint16 bkColor)
{
#define MAX_CHAR_POSX (LCD_W-8)
#define MAX_CHAR_POSY (LCD_H-16)

    uint8 temp, t, pos;
    Size_t size = {8, 16};

    if(site.x > MAX_CHAR_POSX || site.y > MAX_CHAR_POSY)
    {
        return;
    }

    LCD_PTLON(site, size);

    LCD_RAMWR();                    //写内存

    for (pos = 0; pos < 16; pos++)
    {
        temp = ascii_8x16[((ascii-0x20)*16)+pos];

        for(t = 0; t < 8; t++)
        {

            if(temp & 0x80)
            {
                LCD_WR_DATA(Color);
            }
            else
            {
                LCD_WR_DATA(bkColor);
            }
            temp <<= 1;
        }
    }
    return;
#undef MAX_CHAR_POSX
#undef MAX_CHAR_POSY
}

/*!
 *  @brief      显示字符串
 *  @param      site    左上角坐标
 *  @param      Str     字符串地址
 *  @param      Color   字体颜色
 *  @param      bkColor 背景颜色
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_str(site,"www.vcan123.com", BLUE,RED);
 */
void LCD_str(Site_t site, uint8 *Str, uint16 Color, uint16 bkColor)
{
#define MAX_CHAR_POSX (LCD_W-8)
#define MAX_CHAR_POSY (LCD_H-16)
    while(*Str != '\0')
    {
        if(site.x > MAX_CHAR_POSX )
        {
            //换行
            site.x = 0;
            site.y += 16;
        }
        if(site.y > MAX_CHAR_POSY )
        {
            //一屏
            site.y = 0;
            site.x = 0;
        }

        LCD_char(site, *Str, Color, bkColor);
        site.x += 8;
        Str ++ ;
    }
#undef MAX_CHAR_POSX
#undef MAX_CHAR_POSY
}

/*!
 *  @brief      显示数字
 *  @param      site    左上角坐标
 *  @param      num     数字
 *  @param      Color   字体颜色
 *  @param      bkColor 背景颜色
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_num(site,123, BLUE,RED);
 */
void LCD_num(Site_t site, uint32 num, uint16 Color, uint16 bkColor)
{
    uint32 res = num;
    uint8 t = 0;
    Site_t sitetemp;
    sitetemp.y = site.y;

    if( num == 0 )
    {
        LCD_char(site, '0', Color, bkColor);
        return;
    }
    while( res )  /*得到数字长度t*/
    {
        res /= 10;
        t++;
    }

    while(num)
    {
        sitetemp.x = site.x + (8 * (t--) - 8);
        LCD_char(sitetemp, (num % 10) + '0', Color, bkColor);
        num /= 10 ;
    }
}

/*!
 *  @brief      显示数字（清空多余的位）
 *  @param      site            左上角坐标
 *  @param      num             数字
 *  @param      max_num_bit     最大的位数
 *  @param      Color           字体颜色
 *  @param      bkColor         背景颜色
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_num_BC(site,123,5, BLUE,RED);
 */
void LCD_num_BC(Site_t site, uint32 num, uint8 max_num_bit, uint16 Color, uint16 bkColor)
{
    uint32 res = num;
    uint8 t = 0;
    Site_t sitetemp;
    Size_t size;

    sitetemp.y = site.y;

    if( num == 0 )
    {
        LCD_char(site, '0', Color, bkColor);

        site.x += 8;
        size.H  = 16;
        size.W  = 8 * (max_num_bit - 1);
        LCD_rectangle(site, size, bkColor);

        return;
    }
    while( res )            /*得到数字长度t*/
    {
        res /= 10;
        t++;
    }
    if(t >= max_num_bit )    //限制最大长度
    {
        t = max_num_bit;
    }

    res = t;

    while( t != 0 )
    {
        sitetemp.x = site.x + (8 * (t--) - 8);
        LCD_char(sitetemp, (num % 10) + '0', Color, bkColor);
        num /= 10 ;
    }

    if(res != max_num_bit )
    {
        size.W = 8 * (max_num_bit - res);
        site.x += (8 * res);
        size.H  = 16;
        LCD_rectangle(site, size, bkColor);
    }
}

/*!
 *  @brief      灰度图像显示
 *  @param      site            左上角坐标
 *  @param      size            显示图像大小
 *  @param      img             图像地址
 *  @since      v5.0
 *  Sample usage:       Site_t site = {10,20};      //x = 10 ,y = 20
                        Size_t size = {320,240};    //W = 320,H = 240
                        LCD_Img_gray(site, size, img);
 */
void LCD_Img_gray(Site_t site, Size_t size, uint8 *img)
{
    uint32     total = (size.H * size.W);
    uint16     imgtemp;
    uint8     *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //开窗
    LCD_RAMWR();                                //写内存

    while(total--)
    {
        imgtemp     = (uint16) * (pimg++);
        imgtemp = GRAY_2_RGB565(imgtemp);
        LCD_WR_DATA( imgtemp );               //写图像数据
    }
}

/*!
 *  @brief      缩放灰度图像显示
 *  @param      site            左上角坐标
 *  @param      size            显示图像大小
 *  @param      img             图像地址
 *  @param      imgsize         图像大小
 *  @since      v5.0
 *  Sample usage:       Site_t site = {10,20};          //x = 10 ,y = 20
                        Size_t size = {80,60};          //W = 80,H = 60
                        Size_t imgsize = {320,240};     //W = 320,H = 240
                        LCD_Img_gray_Z(site, size, img,imgsize);
 */
void LCD_Img_gray_Z(Site_t site, Size_t size, uint8 *img, Size_t imgsize)
{
    uint32 temp, tempY;
    uint16 x, y;
    uint16 X, Y;
    uint8 *pimg = (uint8 *)img;
    uint16 rgb;

    LCD_PTLON(site, size);                      //开窗

    LCD_RAMWR();                                //写内存

    for(y = 0; y < size.H; y++)
    {
        Y = ( (  y * imgsize.H   ) / size.H) ;
        tempY = Y * imgsize.W ;

        for(x = 0; x < size.W; x++)
        {
            X = ( x * imgsize.W  ) / size.W ;
            temp = tempY + X;
            rgb = GRAY_2_RGB565(pimg[temp]);    //
            LCD_WR_DATA(rgb);
        }
    }
}



void LCD_Img_Binary(Site_t site, Size_t size, uint8 *img)
{
    uint32     total = (size.H * size.W) / 8;
    uint8     imgtemp;
    uint8       bitindex;
    uint8     *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //开窗

    LCD_RAMWR();                                //写内存

    while(total--)
    {
        imgtemp     = *(pimg++);
        bitindex    = 8;
        while(bitindex--)
        {
            if( imgtemp & (0x01 << bitindex) )
            {
                LCD_WR_DATA( BINARY_COLOR );      //写图像数据
            }
            else
            {
                LCD_WR_DATA(  BINARY_BGCOLOR  );      //写图像数据
            }
        }
    }
}

void LCD_Img_Binary_Z(Site_t site, Size_t size, uint8 *img, Size_t imgsize)
{

    uint32 temp, tempY;
    uint16 x, y;
    uint16 X, Y;
    uint8 *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //开窗

    LCD_RAMWR();                                //写内存

    for(y = 0; y < size.H; y++)
    {
        Y = ( (  y * imgsize.H  ) / size.H) ;
        tempY = Y * imgsize.W ;

        for(x = 0; x < size.W; x++)
        {
            X = (( x * imgsize.W  ) / size.W) ;
            temp = tempY + X;
            if( (pimg[temp>>3] & (1 << (7 - (temp & 0x07))) ) == 0  )
            {
                LCD_WR_DATA( BINARY_BGCOLOR );        //写图像数据
            }
            else
            {
                LCD_WR_DATA( BINARY_COLOR );      //写图像数据
            }
        }
    }
}

void LCD_wave(Site_t site,Size_t size,uint8 *img,uint8 maxval,uint16 Color ,uint16 bkColor)
{

    uint8 h;
    uint16 y = site.y + size.H - 1;

    LCD_rectangle(site, size,bkColor);
    site.x += size.W;
    img  += (size.W - 1);
    while(size.W --)
    {
        if(*img >= maxval )
        {
            h = size.H - 1;
        }
        else
        {
            h = ((*img )* size.H )/maxval;
        }
        site.y = y - h ;
        site.x--;
        img --;
        LCD_point(site, Color);
    }
}

void LCD_wave_display(Site_t site,Size_t size,uint8 *img,uint8 maxval,uint16 Color)
{

    uint8 h;
    uint16 y = site.y + size.H - 1;

    site.x += size.W;
    img  += (size.W - 1);
    while(size.W --)
    {
        if(*img >= maxval )
        {
            h = size.H - 1;
        }
        else
        {
            h = ((*img )* size.H )/maxval;
        }
        site.y = y - h ;
        site.x--;
        img --;
        LCD_point(site, Color);
    }
}

void my_lcd_init()
{
  lcd_show_enable1=1;
Site_t site = {3,5};


    LCD_init();
    LCD_str(site,"xiang zuo an",FCOLOUR,BCOLOUR);
    site.y = 20;
    LCD_str(site,"<<<<<<<<<<<<",FCOLOUR,BCOLOUR);
    site.y = 5;
   key_Init();
    
}

void canshu_chang_init()//参数名称显示
{
  
LCD_clear(BCOLOUR);     //清屏
  Site_t site = {3,5};
    LCD_str(site,"kp:",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 15;
    LCD_str(site,"kd:",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 15;
    LCD_str(site,"dawansudu:",FCOLOUR,BCOLOUR);   //显示8*16字符串
   site.y += 15;
    LCD_str(site,"zhidaosudu:",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 15;
    LCD_str(site,"qianzhan:",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 15;
    LCD_str(site,"yuzhi(show):",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 15;
    LCD_str(site,"S_speed:",FCOLOUR,BCOLOUR);   //显示8*16字符串
    site.y += 15;
    LCD_str(site,"DJmiddle:",FCOLOUR,BCOLOUR);   //显示8*16字符串
     
    
     BCOLOUR_NEW[0]=YELLOW;
     site_flag=4;
}

int vaul_change(int vaul)//按键改动参数值函数
{
if(key_status()==4)//下
    {
      systick_delay_ms(5);//消抖
    vaul--;
  
    while(key_status()==4);
    systick_delay_ms(5);//消抖
    }
    else
      if(key_status()==3)//上
      {
        systick_delay_ms(5);//消抖
        vaul++;
    while(key_status()==3);
    systick_delay_ms(5);//消抖
      }
return vaul;
}
void canshu_chang()//改参数界面函数
{
   yuzhi=0x55;
  while(enter_flag)
  {
        site.y = 4;
        site.x = 90;
       
        LCD_num_C (site, kp, FCOLOUR , BCOLOUR_NEW[0]);
        site.y += 15;
        LCD_num_C (site, kd, FCOLOUR , BCOLOUR_NEW[1]);
        site.y += 15;
        LCD_num_C (site, dawan_speed , FCOLOUR , BCOLOUR_NEW[2]);
        site.y += 15;
        LCD_num_C (site, long_speed , FCOLOUR , BCOLOUR_NEW[3]);
        site.y += 15;
        LCD_num_C (site, qianzhan , FCOLOUR , BCOLOUR_NEW[4]);
        site.y += 15;
        LCD_num_C (site, yuzhi, FCOLOUR , BCOLOUR_NEW[5]);
        site.y += 15;
        LCD_num_C (site, S_speed , FCOLOUR , BCOLOUR_NEW[6]);
        site.y += 15;
        LCD_num_C (site, middle_value_of_servo , FCOLOUR , BCOLOUR_NEW[7]);
        
         switch((site_flag-4)/15)
      {
      case 0:kp=vaul_change(kp);break;
      case 1:kd=vaul_change(kd);break;
      case 2:dawan_speed=vaul_change(dawan_speed);break;
      case 3:long_speed=vaul_change(long_speed);break;
      case 4:qianzhan=vaul_change(qianzhan);break;
      case 5:yuzhi=vaul_change(yuzhi);break;
      case 6:S_speed=vaul_change(S_speed);break;
      case 7:middle_value_of_servo=vaul_change(middle_value_of_servo);break;
      }
      if(key_status()==2)//确定
      {
        systick_delay_ms(5);//消抖
        while(key_status()==2);
        systick_delay_ms(5);//消抖
        enter_flag=0;
      
      }
  }
        site.y = 4;
        site.x = 90;
       
        LCD_num_C (site, kp, FCOLOUR , BCOLOUR_NEW[0]);
        site.y += 15;
        LCD_num_C (site, kd, FCOLOUR , BCOLOUR_NEW[1]);
        site.y += 15;
        LCD_num_C (site, dawan_speed , FCOLOUR , BCOLOUR_NEW[2]);
        site.y += 15;
        LCD_num_C (site, long_speed , FCOLOUR , BCOLOUR_NEW[3]);
        site.y += 15;
        LCD_num_C (site, qianzhan , FCOLOUR , BCOLOUR_NEW[4]);
        site.y += 15;
        LCD_num_C (site, yuzhi, FCOLOUR , BCOLOUR_NEW[5]);
        site.y += 15;
        LCD_num_C (site, S_speed , FCOLOUR , BCOLOUR_NEW[6]);
        site.y += 15;
        LCD_num_C (site, middle_value_of_servo , FCOLOUR , BCOLOUR_NEW[7]);
/************************************************按键改参数**************************************************/     
      site.y=site_flag;
        if(key_status()==4)//下
    {
      systick_delay_ms(4);//消抖
     
     site.y=15+site_flag;
     site_flag=site.y;
     if(site_flag>=19)
   BCOLOUR_NEW[(site.y-19)/15]=BCOLOUR;
    if(site_flag>109)
      site_flag=4;
      if(site_flag==4)
   BCOLOUR_NEW[(site_flag-4)/15]=BCOLOUR;
    BCOLOUR_NEW[(site_flag-4)/15]=YELLOW;
    while(key_status()==4);
    systick_delay_ms(5);//消抖
    }
    else
      if(key_status()==3)//上
      {
        systick_delay_ms(4);//消抖
     
     site.y=site_flag-15;
     site_flag=site.y;
     if(site_flag<=109)
   BCOLOUR_NEW[(site.y+19)/15]=BCOLOUR;
    if(site_flag>1000)
    {
 BCOLOUR_NEW[0]=BCOLOUR;
      site_flag=109;
      site.y=109;
    }
  
   
    BCOLOUR_NEW[(site_flag-4)/15]=YELLOW;
    while(key_status()==3);
    systick_delay_ms(5);//消抖
      }
      if(key_status()==2)//确定
      {
        systick_delay_ms(5);//消抖
        while(key_status()==2);
        systick_delay_ms(5);//消抖
        enter_flag=1;
      
      }
     
}
int caidan()//菜单栏函数
{
  LCD_clear(BCOLOUR);     //清屏
  BCOLOUR_NEW[0]=BCOLOUR_NEW[1]=BCOLOUR_NEW[2]=BCOLOUR_NEW[3]=BCOLOUR_NEW[4]=BCOLOUR_NEW[5]=BCOLOUR_NEW[6]=BCOLOUR_NEW[7]=BCOLOUR;
  site.y = 5;
  site.x = 30;
 LCD_str(site,"tu xiang?",FCOLOUR,BCOLOUR);   //看图像
    site.y += 30;
    LCD_str(site,"can shu?",FCOLOUR,BCOLOUR);   //调参数
    site.y += 30;
    LCD_str(site,"suo ping?",FCOLOUR,BCOLOUR);   //锁屏
    site.y = 5;
      site.x = 0;
    LCD_str(site,"__",FCOLOUR,YELLOW);//描线
    while(1)
    {
       
  site.x = 0;
      
    if(key_status()==4)//下
    {
      systick_delay_ms(5);//消抖
      LCD_str(site,"  ",FCOLOUR,BCOLOUR);//覆盖线线
    site.y+=30;
    if(site.y>65)
      site.y=5;
    LCD_str(site,"__",FCOLOUR,YELLOW);//描线
    while(key_status()==4);
    systick_delay_ms(5);//消抖
    }
    else
      if(key_status()==3)//上
      {
        systick_delay_ms(5);//消抖
        LCD_str(site,"  ",FCOLOUR,BCOLOUR);//覆盖线线
        if(site.y>=10)
      site.y-=30;
      else
    if(site.y<10)
      site.y=65;
    LCD_str(site,"__",FCOLOUR,YELLOW);//描线
    while(key_status()==3);
    systick_delay_ms(5);//消抖
      }
    else
      if(key_status()==2)//确定
      {
        systick_delay_ms(5);//消抖
        while(key_status()==2);
        systick_delay_ms(5);//消抖
      switch(site.y)
      {
      case 5:return 1;break;//看图像
      case 35:canshu_chang_init();return 2;break;//调参数
      case 65:return 3;break;//锁屏
      }
      }
    }
}
void show_img()
{ 
  int iiii=0;
 Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size;                   //显示区域图像大小
    // LCD_init();

   size.H = CAMERA_H;
    size.W = CAMERA_W;
   // img_extract((u8 *)imgyiwei, imgbuff,CAMERA_SIZE);      //解压图像
    scan_from_middle();
    LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
    for(iiii=59;iiii>=0;iiii--)
    {
      Site_t site[3] = {{current_middle_position[iiii],iiii},{current_middle_position[iiii]+1,iiii},{current_middle_position[iiii]-1,iiii}};   //3个点，坐标分别是  {10,20},{11,21},{12,22}
    LCD_points(site,3, GREEN);
    }
   
}
 unsigned char caidan_answer=0;
void show_go()
{
    LCD_clear(BCOLOUR);     //清屏
  site.y = 60;
  site.x = 20;
LCD_str(site,"LETS GO!",FCOLOUR,BCOLOUR);   //显示8*16字符串
lcd_show_enable1=0;
}
void my_lcd_show()
{
  lcd_show_enable1=1;
  //**********************************************按键处理***************************************************//
       
        if(key_status()==1)//左键
        {
          
           systick_delay_ms(5);//消抖
           while(key_status()==1);
           systick_delay_ms(5);//消抖
         caidan_answer=caidan();
         
        }
        else;
          switch(caidan_answer)
          {
          case 1:show_img();break;//看图
          case 2:canshu_chang();break;//调参数
          case 3:show_go();break;//锁屏
          }
}