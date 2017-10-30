#include  "mm.h"
#include "include.h"

extern float var[6];

void display()
{
    if(1 == TFTShow_img_flag)
    { 
      ScreenShow();
    }
    /*
    if(1 == OLED_dis_flag)
    {
      if(HX_STATE == HX_BEFORE)
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"l_turn_point=");
        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
        LED_P6x8Str(0,4,"r_turn_point=");
        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,5,"speed_L     =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_L); 
        LED_P6x8Str(0,6,"speed_R     =");
        LED_PrintsignValueI4(80,6,(int)motor.speed_R);
        LED_P6x8Str(0,7,"HX_BEFORE");
//        LED_P6x8Str(0,7,"servo       =");
//        LED_PrintsignValueI4(80,7,(int)(servos.direction_duty_output-Servo_mid));
      }
      if(HX_STATE == HX_ON )
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"l_turn_point=");
        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
        LED_P6x8Str(0,4,"r_turn_point=");
        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,5,"speed_set_L =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_set_L); 
        LED_P6x8Str(0,6,"speed_set_R =");
        LED_PrintsignValueI4(80,6,(int)motor.speed_set_R);
        LED_P6x8Str(0,7,"HX_ON    ");
//        LED_P6x8Str(0,7,"servo       =");
//        LED_PrintsignValueI4(80,7,(int)(servos.direction_duty_output-Servo_mid));
      }
      if(HX_STATE == HX_ONON )
      {
        LED_PrintsignValueI4(80,6,(int)motor.speed_set_R);
        LED_P6x8Str(0,7,"HX_ONON    ");
      }
      if(HX_STATE == HX_OUT)
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"speedL_R:");
        LED_PrintsignValueI4(54,3,(int)motor.speed_L);
        LED_PrintsignValueI4(90,3,(int)motor.speed_R);
//        LED_P6x8Str(0,3,"l_turn_point=");
//        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
//        LED_P6x8Str(0,4,"r_turn_point=");
//        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,4,"speed_set_L =");
        LED_PrintsignValueI4(80,4,(int)motor.speed_set_L); 
        LED_P6x8Str(0,5,"speed_set_R =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_set_R);
        LED_P6x8Str(0,6,"SERVE_OUT:");
        LED_PrintsignValueI4(80,6,(int)(servos.direction_duty_output-Servo_mid));
        LED_P6x8Str(0,7,"HX_OUT   ");
      }
      if(HX_STATE == HX_NORMAL)
      {
        LED_P6x8Str(0,0,"camer.error =");
        LED_PrintsignValueF4(80,0,camer.error); 
        LED_P6x8Str(0,1,"Prospect_See=");
        LED_PrintValueI4(80,1,(int)Prospect_See);
        LED_P6x8Str(0,2,"speed_set   =");
        LED_PrintsignValueI4(80,2,(int)motor.speed_set);  
        LED_P6x8Str(0,3,"speedL_R:");
        LED_PrintsignValueI4(54,3,(int)motor.speed_L);
        LED_PrintsignValueI4(90,3,(int)motor.speed_R);
//        LED_P6x8Str(0,3,"l_turn_point=");
//        LED_PrintsignValueI4(80,3,(int)l_cross_turn_point); 
//        LED_P6x8Str(0,4,"r_turn_point=");
//        LED_PrintsignValueI4(80,4,(int)r_cross_turn_point);
        LED_P6x8Str(0,4,"speed_set_L =");
        LED_PrintsignValueI4(80,4,(int)motor.speed_set_L); 
        LED_P6x8Str(0,5,"speed_set_R =");
        LED_PrintsignValueI4(80,5,(int)motor.speed_set_R);
        LED_P6x8Str(0,6,"SERVE_OUT:");
        LED_PrintsignValueI4(80,6,(int)(servos.direction_duty_output-Servo_mid));
        LED_P6x8Str(0,7,"HX_NORMAL");
      }
    }*/
//        if(Car_Stop_flag == 1)
//        {
//          gpio_init (PTB2,GPO, 0);
//          gpio_set (PTB2, 0);
//        }
//        else
//        {
//          gpio_init (PTB2,GPO, 0);
//          gpio_set (PTB2, 1);
//        }
}
void send()
{
    /*Matlab接收图像*/
//    if(1)
//    {
//        if(img_switch_flag != 0)
//        vcan_sendmatlabimg(img_buffer,CAMERA_SIZE);
//        else
//        vcan_sendmatlabimg(img_buffer2,CAMERA_SIZE);
//    }
//    if(0)//Matlab 示波器发送控制
//    {
//      vag[0] = motor.avg_speed;
//      vcan_sendmatlabware((uint8_t *)vag, sizeof(vag));
//    }
    /*示波器*/
    if(img_send_flag)//1 == send_osc_flag)
    {
       var[0] = camer.error;
       var[1] = servos.direction_p*100;
       var[2] = motor.speed_set_R;
       var[4] = motor.speed_set_L;//camer.error-camer.last_error;
       var[3] = motor.speed_L;//HX_STATE;
       var[5] = motor.speed_R;//motor.speed_set_R;
       
       vcan_sendware((uint8_t *)var, sizeof(var));
    }
}
