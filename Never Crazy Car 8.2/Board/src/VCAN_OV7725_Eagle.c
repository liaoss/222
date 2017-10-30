/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       VCAN_OV7725_Eagle.c
 * @brief      鹰眼ov7725驱动代码
 * @author     山外科技
 * @version    v5.0
 * @date       2013-09-07
 */
#include "common.h"
#include "MK60_gpio.h"
#include "MK60_port.h"
#include "MK60_dma.h"
#include "VCAN_camera.h"
#include "Extern.h"



#define OV7725_EAGLE_Delay_ms(time)  DELAY_MS(time)

uint8   *ov7725_eagle_img_buff;
volatile IMG_STATUS_e      ov7725_eagle_img_flag = IMG_FINISH;   //图像状态

//内部函数声明
static uint8 ov7725_eagle_reg_init(void);
static void ov7725_eagle_port_init();


/*!
 *  @brief      鹰眼ov7725初始化
 *  @since      v5.0
 */

uint8 ov7725_eagle_init(uint8 *imgaddr)
{
    ov7725_eagle_img_buff = imgaddr;
    while(ov7725_eagle_reg_init() == 0);
    ov7725_eagle_port_init();
    return 0;
}


/*!
 *  @brief      鹰眼ov7725管脚初始化（内部调用）
 *  @since      v5.0
 */
void ov7725_eagle_port_init()
{
    //DMA通道0初始化，PTA27触发源(默认上升沿)，源地址为PTB_B0_IN，目的地址为：IMG_BUFF，每次传输1Byte
    dma_portx2buff_init(CAMERA_DMA_CH, (void *)&PTD_B0_IN, (void *)ov7725_eagle_img_buff, PTC15, DMA_BYTE1, CAMERA_DMA_NUM, DADDR_KEEPON);

    DMA_DIS(CAMERA_DMA_CH);
    disable_irq(PORTC_IRQn);                        //关闭PTC的中断
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);                   //清除通道传输中断标志位
    DMA_IRQ_EN(CAMERA_DMA_CH);

    port_init(PTC15, ALT1 | DMA_FALLING | PULLDOWN );         //PCLK

    port_init(PTC16, ALT1 | IRQ_RISING  | PULLDOWN | PF);     //场中断，上拉，上降沿触发中断，带滤波

}

/*!
 *  @brief      鹰眼ov7725场中断服务函数
 *  @since      v5.0
 */
void ov7725_eagle_vsync(void)
{

    //场中断需要判断是场结束还是场开始
    if(ov7725_eagle_img_flag == IMG_START)                   //需要开始采集图像
    {
        ov7725_eagle_img_flag = IMG_GATHER;                  //标记图像采集中
        disable_irq(PORTC_IRQn);

#if 1

        PORTC_ISFR = 1 <<  PT15;            //清空PCLK标志位

        DMA_EN(CAMERA_DMA_CH);                  //使能通道CHn 硬件请求
        PORTC_ISFR = 1 <<  PT15;               //清空PCLK标志位
                                                                    
        DMA_DADDR(CAMERA_DMA_CH) = (uint32)ov7725_eagle_img_buff;    //恢复地址

#else
        PORTC_ISFR = 1 <<  PT15;            //清空PCLK标志位
        dma_repeat(CAMERA_DMA_CH, (void *)&PTB_B0_IN, (void *)ov7725_eagle_img_buff,CAMERA_DMA_NUM);
#endif
    }
    else                                        //图像采集错误
    { 
        disable_irq(PORTC_IRQn);                        //关闭PTA的中断
        ov7725_eagle_img_flag = IMG_FAIL;                    //标记图像采集失败
    }
}

/*!
 *  @brief      鹰眼ov7725 DMA中断服务函数
 *  @since      v5.0
 */
void ov7725_eagle_dma()
{
    ov7725_eagle_img_flag = IMG_FINISH ;
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //清除通道传输中断标志位
}

/*!
 *  @brief      鹰眼ov7725采集图像（采集到的数据存储在 初始化时配置的地址上）
 *  @since      v5.0
 */
void ov7725_eagle_get_img()
{        
    ov7725_eagle_img_flag = IMG_START;                   //开始采集图像  
    //在开中断之前交换存储数组
    if(img_switch_flag == 0)
       {
         ov7725_eagle_img_buff =img_buffer2;img_switch_flag=1;
       }
    else  
       {
         ov7725_eagle_img_buff = img_buffer;img_switch_flag = 0; 
       }
    //开中断，等待鹰眼场中断信号来临
    PORTC_ISFR = ~0;                        //写1清中断标志位(必须的，不然回导致一开中断就马上触发中断)
    enable_irq(PORTC_IRQn);                         //允许PTA的中断   
}



/*OV7725初始化配置表*/
reg_s ov7725_eagle_reg[] =
{
    //寄存器，寄存器值次
    {OV7725_COM4         , 0x81},//0
    {OV7725_CLKRC        , 0x00},//1
    {OV7725_COM2         , 0x03},//2
    {OV7725_COM3         , 0xD0},//3
    {OV7725_COM7         , 0x40},//4
    {OV7725_HSTART       , 0x3F},//5
    {OV7725_HSIZE        , 0x50},//6
    {OV7725_VSTRT        , 0x03},//7
    {OV7725_VSIZE        , 0x78},//8
    {OV7725_HREF         , 0x00},//9
    {OV7725_SCAL0        , 0x0A},//10
    {OV7725_AWB_Ctrl0    , 0xE0},//11
    {OV7725_DSPAuto      , 0xff},//12
    {OV7725_DSP_Ctrl2    , 0x0C},//13
    {OV7725_DSP_Ctrl3    , 0x00},//14
    {OV7725_DSP_Ctrl4    , 0x00},//15
    
#if (CAMERA_W == 80)             
    {OV7725_HOutSize     , 0x14},//16
#elif (CAMERA_W == 160)          
    {OV7725_HOutSize     , 0x28},//16
#elif (CAMERA_W == 240)          
    {OV7725_HOutSize     , 0x3c},//16
#elif (CAMERA_W == 320)          
    {OV7725_HOutSize     , 0x50},//16
#else                            
                                 
#endif                           
                                 
#if (CAMERA_H == 60 )            
    {OV7725_VOutSize     , 0x1E},//17
#elif (CAMERA_H == 120 )         
    {OV7725_VOutSize     , 0x3c},//17
#elif (CAMERA_H == 180 )         
    {OV7725_VOutSize     , 0x5a},//17
#elif (CAMERA_H == 240 )         
    {OV7725_VOutSize     , 0x78},//17
#else                            
                                 
#endif                           
                                 
    {OV7725_EXHCH        , 0x00},//18
    {OV7725_GAM1         , 0x0c},//19
    {OV7725_GAM2         , 0x16},//20
    {OV7725_GAM3         , 0x2a},//21
    {OV7725_GAM4         , 0x4e},//22
    {OV7725_GAM5         , 0x61},//23
    {OV7725_GAM6         , 0x6f},//24
    {OV7725_GAM7         , 0x7b},//25
    {OV7725_GAM8         , 0x86},//26
    {OV7725_GAM9         , 0x8e},//27
    {OV7725_GAM10        , 0x97},//28
    {OV7725_GAM11        , 0xa4},//29
    {OV7725_GAM12        , 0xaf},//30
    {OV7725_GAM13        , 0xc5},//31
    {OV7725_GAM14        , 0xd7},//32
    {OV7725_GAM15        , 0xe8},//33
    {OV7725_SLOP         , 0x20},//34
    {OV7725_LC_RADI      , 0x00},//35
    {OV7725_LC_COEF      , 0x13},//36
    {OV7725_LC_XC        , 0x08},//37
    {OV7725_LC_COEFB     , 0x14},//38
    {OV7725_LC_COEFR     , 0x17},//39
    {OV7725_LC_CTR       , 0x05},//40
    {OV7725_BDBase       , 0x99},//41
    {OV7725_BDMStep      , 0x03},//42
    {OV7725_SDE          , 0x04},//43
    {OV7725_BRIGHT       , 0x00},//44
    {OV7725_CNST         , 0x40},//45 //阈值段 0x28(40)   0x2D(45)  0x32(50)   0x37(55)  0x3C(60) //黑的多调高
    {OV7725_SIGN         , 0x06},//46
    {OV7725_UVADJ0       , 0x11},//47
    {OV7725_UVADJ1       , 0x02},//48

};
/*
50帧：
{COM4         , 0xC1},
{CLKRC        , 0x02},


75帧：
{COM4         , 0x41},
{CLKRC        , 0x00},


112帧：
{COM4         , 0x81},
{CLKRC        , 0x00},


150帧：
{COM4         , 0xC1},
{CLKRC        , 0x00},
*/

uint8 ov7725_eagle_cfgnum = ARR_SIZE( ov7725_eagle_reg ) ; /*结构体数组成员数目*/


/*!
 *  @brief      鹰眼ov7725寄存器 初始化
 *  @return     初始化结果（0表示失败，1表示成功）
 *  @since      v5.0
 */
uint8 ov7725_eagle_reg_init(void)
{
    uint16 i = 0;
    uint8 Sensor_IDCode = 0;
    SCCB_GPIO_init();

    //OV7725_Delay_ms(50);
    if( 0 == SCCB_WriteByte ( OV7725_COM7, 0x80 ) ) /*复位sensor */
    {
        DEBUG_PRINTF("\n警告:SCCB写数据错误");
        return 0 ;
    }

    OV7725_EAGLE_Delay_ms(50);

    if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, OV7725_VER ) )    /* 读取sensor ID号*/
    {
        DEBUG_PRINTF("\n警告:读取ID失败");
        return 0;
    }
    DEBUG_PRINTF("\nGet ID success，SENSOR ID is 0x%x", Sensor_IDCode);
    DEBUG_PRINTF("\nConfig Register Number is %d ", ov7725_eagle_cfgnum);
    if(Sensor_IDCode == OV7725_ID)
    {
        for( i = 0 ; i < ov7725_eagle_cfgnum ; i++ )
        {
            if( 0 == SCCB_WriteByte(ov7725_eagle_reg[i].addr, ov7725_eagle_reg[i].val) )
            {
                DEBUG_PRINTF("\n警告:写寄存器0x%x失败", ov7725_eagle_reg[i].addr);
                return 0;
            }
        }
    }
    else
    {
        return 0;
    }
    DEBUG_PRINTF("\nOV7725 Register Config Success!");
    return 1;
}


