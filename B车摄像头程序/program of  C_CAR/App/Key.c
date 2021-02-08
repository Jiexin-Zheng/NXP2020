#include "common.h"
#include "include.h"
int key_5xiang[5]={0,0,0,0,0};
int keyup_,keydown_,keyenter_,keyleft_,keyright_;//2015.6.23

void key_Init(void)
{
  //gpio_init (PTA17, GPO, 1u);// LED

  gpio_init(PTD12, GPI, HIGH);  //left 12
  gpio_init(PTD14, GPI, HIGH);  //enter 14
  gpio_init(PTD15, GPI, HIGH);  //UP 15
  gpio_init(PTD13, GPI, HIGH);  //DOWN13
  gpio_init(PTD11, GPI, HIGH);  //RIGHT11
}

int key_status(void)
{
  
  key_5xiang[0]=gpio_get(PTD12);
  key_5xiang[1]=gpio_get(PTD14);
  key_5xiang[2]=gpio_get(PTD15);
  key_5xiang[3]=gpio_get(PTD13);
  key_5xiang[4]=gpio_get(PTD11);
    if(key_5xiang[0]==0)
      return 1;
    else if(key_5xiang[1]==0)
      return 2;
    else if(key_5xiang[2]==0)
      return 3;
    else if(key_5xiang[3]==0)
      return 4;
    else if(key_5xiang[4]==0)
      return 5;
    else
      return 0;
}