/*!
*     COPYRIGHT NOTICE
*     Copyright (c) 2013,山外科技
*     All rights reserved.
*     技术讨论：山外论坛 http://www.vcan123.com
*
*     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
*     修改内容时必须保留山外科技的版权声明。
*
* @file       main.c
* @brief      山外K60 平台主程序
* @author     山外科技
* @version    v5.3
* @date       2015-04-07
*/

#include "include.h"
#include "define.h"
//FIRSTCAR
//extern uint8 s;
int16 vag[1];
//static uint8 start_line_count=0;
float Speed_KP = 9.5;//12.8//7.4
float Speed_KI = 0.5;//0.38
float HighSpeed_KP = 16.5;//20.8//8.4
float HighSpeed_KI = 0.9;//0.64
void BM_value()
{
  if(1==BM1)
  {
    UFF_NORMAL = UFF1_NORMAL ;
    UFF_HX = UFF1_HX;
    dajiao_Prospect_See = 92;
    servos.direction_p = 0.9;//0.68;
    servos.direction_d = 0.5;//0.58;
    Obstacle_offset = 25;
  }
  if(1==BM2)
  {
    UFF_NORMAL = UFF2_NORMAL;
    UFF_HX = UFF2_HX;
    dajiao_Prospect_See = 90;
    servos.direction_p = 0.9;
    servos.direction_d = 0.6;
    Obstacle_offset = 26;
  }
  if(1==BM3)
  {
    UFF_NORMAL = UFF3_NORMAL ;
    UFF_HX = UFF3_HX;
    dajiao_Prospect_See = 95;
    servos.direction_p = 0.72;
    servos.direction_d = 0.58;
    Obstacle_offset = 27;
  }
  if(1==BM4)
  {
    UFF_NORMAL = UFF4_NORMAL ;
    UFF_HX = UFF4_HX;
    dajiao_Prospect_See = 95;
    servos.direction_p = 0.74;
    servos.direction_d = 0.54;
    Obstacle_offset = 30;
  }
  if(1==BM5)
  {
    UFF_NORMAL = UFF0_NORMAL ;
    UFF_HX = UFF0_HX;
    dajiao_Prospect_See = 95;
    servos.direction_p = 0.72;
    servos.direction_d = 0.58;
    Obstacle_offset = 27;
  }
}

void  main(void) 
{
  System_init();
  
//  servos.direction_p = Servo_KP;
//  servos.direction_d = Servo_KD;
  motor.speed_p =Speed_KP;
  motor.speed_i =Speed_KI;
  
  motor.speed_filter_error[0] = 0;
  motor.speed_filter_error[1] = 0;
  motor.speed_filter_error[2] = 0;
  
  BM_value();
  TFTShow_img_flag = BM6;
  img_send_flag = BM7;
  if(TFTShow_img_flag+img_send_flag)    
  {
    motor.speed_p = 0;
    motor.speed_i = 0;
  }
  DELAY_MS(200);
  while(1)
 {
    camera_get_img();//获取图像
    
    if(img_switch_flag == 0)
        img_extract(img_handle,srcimg,img_buffer2,CAMERA_SIZE);
    else if(img_switch_flag == 1)
        img_extract(img_handle,srcimg,img_buffer,CAMERA_SIZE);
    send();

    Last_Prospect_See=Prospect_See;

    Get_Edge();
        display();
    /*上位机接收*/
    if(BM8)
    {
      while(1)
      {
        if(img_switch_flag != 0)
           vcan_sendimg(img_buffer, CAMERA_SIZE);                   //发送到上位机
        else
           vcan_sendimg(img_buffer2, CAMERA_SIZE);                  //发送到上位机
      }
    }

    direction_control();

    speed_control();
    
    while(ov7725_eagle_img_flag != IMG_FINISH)           //等待图像采集完毕
    {
      if(ov7725_eagle_img_flag == IMG_FAIL)            //假如图像采集错误，则重新开始采集
      {
        ov7725_eagle_img_flag = IMG_START;           //开始采集图像
        PORTC_ISFR = ~0;                             //写1清中断标志位(必须的，不然回导致一开中断就马上触发中断)
        enable_irq(PORTC_IRQn);                      //允许PTA的中断
      }
    }
  }  
}
//电机测试
//    ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,400);
//    ftm_pwm_duty(L_GO_FTM,L_GO_CH,0);
//    ftm_pwm_duty(R_GO_FTM,R_GO_CH,0);
//    ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,400);
//    DELAY_MS(3000);
//    ftm_pwm_duty(L_BACK_FTM,L_BACK_CH,0);
//    ftm_pwm_duty(L_GO_FTM,L_GO_CH,200);
//    ftm_pwm_duty(R_GO_FTM,R_GO_CH,200);
//    ftm_pwm_duty(R_BACK_FTM,R_BACK_CH,0);
//    DELAY_MS(3000);
//        LED_P6x8Str(0,5,"speed_L     =");
//        LED_PrintsignValueI4(80,5,(int)motor.speed_L);                   
//        LED_P6x8Str(0,6,"speed_R     =");
//        LED_PrintsignValueI4(80,6,(int)motor.speed_R);
//void  main(void)
//{
//  Motor_init();            //电机初始化
//  Servo_Motor_init();       //舵机初始化
//  OLED_init();
//  ftm_pwm_duty(Servo_ftm,Servo_CH,Servo_mid-160);
//  while(1)
//  {
//    uint8 L,R;
//    uint32 speed_L,speed_R;
//    L=gpio_get(LEFT_ENCODER_DIRECTION);
//    R=gpio_get(RIGHT_ENCODER_DIRECTION);
//    if(L==0)
//    speed_L= DMA_count_get(LEFT_ENCODER_DMA_CH);
//    else
//    ; 
//    if(R==0)
//    ;
//    else
//    speed_R= DMA_count_get(RIGHT_ENCODER_DMA_CH);
//    if(speed_L >= 30000)
//    {
//      while(1)
//      {
//      LED_PrintValueI5(0,0,(int)speed_L);
//      LED_PrintValueI5(0,1,(int)speed_R);
//      }
//    }
//  }
//}



