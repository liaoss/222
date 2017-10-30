#include  "common.h"
#include  "VCAN_camera.h"
#include  "system.h"
#include  "extern.h"
#include  "Motor.h"


camer_status camer;

int8  circle_count = 0;
//int8 rampup_count=0,rampdown_count=0;
//int16 obstacle_position;
//int Ramp_Rate[2] = {0};
int val_area[120][4] = {0};
int border[120][4];
TRACK_INFM  left,right;
//uint8 fork_head,fork_tail;
//int8 fork_flag;
int left_track_tail=0,right_track_tail=0;


float Correction_array[120] = { 
  4.32,4.15,4.00,3.86,3.72,3.48,3.48,3.27,3.18,3.09,
  3.00,2.92,2.84,2.70,2.70,2.57,2.57,2.45,2.45,2.35,
  2.35,2.25,2.25,2.16,2.16,2.08,2.08,2.04,2.00,2.00,
  1.93,1.93,1.86,1.86,1.80,1.80,1.80,1.74,1.74,1.69,
  1.69,1.66,1.64,1.61,1.59,1.59,1.57,1.54,1.52,1.50,
  1.50,1.48,1.46,1.44,1.44,1.44,1.40,1.40,1.37,1.37,
  1.37,1.33,1.33,1.32,1.30,1.30,1.29,1.29,1.27,1.26,
  1.26,1.24,1.24,1.24,1.24,1.21,1.21,1.21,1.19,1.19,
  1.19,1.17,1.16,1.16,1.16,1.15,1.14,1.14,1.14,1.13,
  1.11,1.11,1.11,1.10,1.09,1.08,1.08,1.08,1.07,1.06,
  1.06,1.06,1.06,1.05,1.04,1.04,1.04,1.04,1.04,1.03,
  1.02,1.02,1.02,1.02,1.02,1.02,1.00,1.00,1.00,1.00
};
int16 rempcheck_array[120]={ 
  25,26,27,28,29,31,31,33,34,35,
  36,37,38,40,40,42,42,44,44,46,
  46,48,48,50,50,52,52,53,54,54,
  56,56,58,58,60,60,60,62,62,64,
  64,65,66,67,68,68,69,70,71,72,
  72,73,74,75,75,75,77,77,79,79,
  79,81,81,82,83,83,84,84,85,86,
  86,87,87,87,87,89,89,89,91,91,
  91,92,93,93,93,94,95,95,95,96,
  97,97,97,98,99,100,100,100,101,102,
  102,102,102,103,104,104,104,104,104,105,
  106,106,106,106,106,106,108,108,108,108,
};

//压缩二值化图像解压（空间 换 时间 解压）
//srclen 是二值化图像的占用空间大小
//【鹰眼解压】鹰眼图像解压，转为 二维数组 - 智能车资料区 - 山外论坛 http://vcan123.com/forum.php?mod=viewthread&tid=17&ctid=6
//解压的时候，里面有个数组，配置黑、白对应的值是多少。
void img_extract(void *dst1, void *dst2, void *src, uint32_t srclen)
{
    uint8_t colour[2] = {255, 0}; //0 和 1 分别对应的颜色
    uint8_t * mdst1 = dst1;
    uint8_t * mdst2 = dst2;
    uint8_t * msrc = src;
    //注：山外的摄像头 0 表示 白色，1表示 黑色
    uint8_t tmpsrc;
    while(srclen --)
    {
        tmpsrc = *msrc++;
        *mdst2++ = ~tmpsrc;
        *mdst1++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *mdst1++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}
//void img_extract(void *dst, void *src1, uint32_t srclen)
//{
//    uint8_t colour[2] = {255, 0}; //0 和 1 分别对应的颜色
//    uint8_t * mdst = dst;
//    uint8_t * msrc = src;
//    //注：山外的摄像头 0 表示 白色，1表示 黑色
//    uint8_t tmpsrc;
//    while(srclen --)
//    {
//        tmpsrc = *msrc++;
//        *mdst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
//        *mdst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
//    }
//}

void CopyToSrcimg(void *dst, void *src, uint32_t srclen)
{
    uint8_t * mdst = dst;
    uint8_t * msrc = src;
    while(srclen --)
    {
      *mdst++ = ~*msrc++;
    }
}
/******************************************************
* @function name : 图像处理
* @ data : 2016/9/18
* @function description : 赛道边缘提取
******************************************************/
void Get_Edge()
{
    int track_end=0;
//    float cdet;
//    int rampdect_head,rampdect_tail,rampdect_roadw=0,ramp_rate;
//    uint8 cpixel,ccolum,clcount=0,crcount=0;
    uint8 fork_head=EMPTY,fork_tail=EMPTY;
//    int left_track_tail=0,right_track_tail=0;
//    int left_jump1=0,right_jump1=0;
    int8 fork_flag = 0;
    track_state_e track_type = TRACK_COMMON;
    uint8 maxw_row;
//    TRACK_INFM  left,right;
    /////////////初始化///////////////////////////////////
    data_init();
    Prospect_See = 0;
    /******************赛道信息提取*********************/
    ///////////////////赛道边缘跟踪//////////////////////
    track_end = Edge_Tracking();
    StartPoint = (val_area[118][0]+val_area[118][1])/2;
    if(StartPoint < 0 || StartPoint > 19 || val_area[118][1]-val_area[118][0]<=0)       StartPoint = 10;
    ////////////////////赛道中间扫描//////////////////////
    fork_flag = Track_Scanning(&fork_head, &fork_tail, track_end);
    /////////////////////转化赛道边界///////////////////////
    maxw_row = AcquireRealBorder(fork_head, fork_tail, fork_flag, track_end);
    if(border[maxw_row][3]-border[maxw_row][2] < UKBLACK_WIDTH)     fork_flag = 0;
    srcnumb = fork_flag;
    /********************赛道信息运算处理***********************/
    /////////////////////左右线滤波//////////////////////
    TrackFilter(119, track_end, Lline);
    TrackFilter(119, track_end, Rline);
    /////////////////////斜率/////////////////////
    CalFirstDeriva(&left, Lline, track_end);
    CalFirstDeriva(&right, Rline, track_end);
    /////////////////////拐点/////////////////////
    AcquireInflecPoint(&left, track_end, Lline, 'l');
    AcquireInflecPoint(&right, track_end, Rline, 'r');
    ////////////////////跳变点////////////////////
    AcquireJumpPoint(&left, track_end, 'l');
    AcquireJumpPoint(&right, track_end, 'r');
    Prospect_See = track_end;
    left_track_tail = MAX(left.jump_point[0],track_end);
    right_track_tail = MAX(right.jump_point[0],track_end);
    /*********************赛道类型判断及处理*************************/
    /////////////////////////处理障碍物////////////////////////////////////
    if(Obstacle_flag)
    {
      left_track_tail = track_end;
      right_track_tail = track_end;
      track_type = Obstacle_handle(fork_flag,fork_head,fork_tail,track_end);
    }
    //////////////////////////////////////////////////////////////////环形标志位处理
    if(circle_flag != NUCIRCLE)       
    {
      Circle_flag_handle(fork_flag,fork_head,fork_tail);
      track_type = TRACK_CIRCLE;
    }
    //////////////////////////////////////////////////////////////////判断环形
    if(circle_flag == NUCIRCLE && track_type == TRACK_COMMON && ABS(left.inf_point-right.inf_point)<20 && left.inf_point>20 && right.inf_point>20)
    {
      track_type = CircleJudge();
      if(track_type == TRACK_CIRCLE)
      {
//        PTE2_OUT = 1;
        if(!Current_Turn_state)         circle_flag = LEFT;
        else                           circle_flag = RIGHT;
        Current_Turn_state = Turn_state & Texting_state;//判定Current_Turn_state是否为0来左右转；0左1右 
        Texting_state = Texting_state << 1;
        if(Texting_state == 0x20)//0010 0000
          Texting_state = 0x01;
      }
    }
    ///////////////////////////////////////////////////////////////////环形处理
    if(circle_flag != NUCIRCLE) 
    {
      left_track_tail = track_end;
      right_track_tail = track_end;
      Circle_handle(track_end,fork_flag,fork_head,fork_tail);
    }
    ///////////////////////////////处理岔路//////////////////////////////////////////
    if(fork_flag && track_type == TRACK_COMMON)//处理岔路情况
    {
        left_track_tail  = track_end;
        right_track_tail = track_end;
      //障碍与起跑线///////////////////////////////////////////////////////////////////////////////////////
        if(circle_flag == NUCIRCLE && left.inf_point<fork_tail && right.inf_point<fork_tail && left.jump_point[0]<fork_tail && right.jump_point[0]<fork_tail
           && MMN(Lline,fork_tail,fork_head+10,0)>0 && MMN(Rline,fork_tail,fork_head+10,1)<159)//判断障碍物起跑线
        {
            track_type = ScratchObstacleJudge(maxw_row, fork_head, fork_tail);
            switch(track_type)
            {
            case TRACK_OBSTACLE_L:
              Obstacle_flag = 1;
              break;
            case TRACK_OBSTACLE_R:
              Obstacle_flag = 2;
              break;
            case TRACK_SCRATCH:
              if(Scratch_flag == 0 || Scratch_flag == 3)  Scratch_flag ++;
            }
        }
        //斜十字///////////////////////////////////////////////////////////////////////////////////////
        if(circle_flag == NUCIRCLE && track_type == TRACK_COMMON && left.inf_point-fork_head>10 && left.inf_point>40)//判断左斜十字 
        {
            track_type = ObliqueCrossJudge(left.inf_point, maxw_row, fork_tail, fork_head, 'l');
            if(track_type == TRACK_OBLI_CRO_L)
              fork_select(Lline,left.inf_point+3,fork_head-3,fork_tail,'r');
        }
        if(circle_flag == NUCIRCLE && track_type == TRACK_COMMON && right.inf_point-fork_head>10 && right.inf_point>40)//判断右斜十字 
        {
            track_type = ObliqueCrossJudge(right.inf_point, maxw_row, fork_tail, fork_head, 'r');
            if(track_type == TRACK_OBLI_CRO_R)
              fork_select(Rline,right.inf_point+3,fork_head-3,fork_tail,'l');
        }
    }
    ///////////////////////////////////////////////////////////////////起跑线计数
    if(Scratch_flag != 0 || Scratch_flag != 3)
    {
      switch(Scratch_flag)
      {
      case 1:   if( sratch_handle() )   Scratch_flag = 2;break;                    //第一次到达起跑线
      case 2:   if( !sratch_handle() )   Scratch_flag = 3;break;                   //第一次通过起跑线
      case 4:   if( sratch_handle() )   Scratch_flag = 5;break;                    //第二次到达起跑线
      case 5:   if( !sratch_handle() )   {Scratch_flag = 6;Car_Stop_flag=1;}break; //第二次通过起跑线
      }
    }
    srcnuma = Scratch_flag;
    ////////////////////////////////////////////////////////////////////////十字
//    if(left.jump_count && right.jump_count && circle_flag == NUCIRCLE && !fork_flag && track_type == TRACK_COMMON)
//      ComCross_handle(&right,&left,&left_track_tail,&right_track_tail,track_end);
    /////////////////////////////////////////////////////////////////////赛道末端延长
    if(left_track_tail > 80)    left_track_tail = track_end;
    if(right_track_tail > 80)    right_track_tail = track_end;
//    if(circle_flag != NUCIRCLE)  
//    {
//      left_track_tail = track_end;
//      right_track_tail = track_end;
//    }
    left_track_tail = MIN(left_track_tail+5,110);
    right_track_tail = MIN(right_track_tail+5,110);
    if(Lline[left_track_tail]>80)
      ExtendTheTrack(left_track_tail, 0, Lline);
    else if(Rline[right_track_tail]<80)
      ExtendTheTrack(right_track_tail, 0, Rline);

    ////////////////////////////////////////////////////////////////////获取中线
    aquire_Mline(Lline, Rline, Mline);
    ////////////////////////////////////////////////////////////////////计算偏差
    camer_error_calculation();
    ///////////////////////////////////////////////////////////////////前瞻与复杂度
    if(!Prospect_See)
      Prospect_See = track_end+1;
    Track_complexity = ABS((int16)camer.error);
    
//    if(track_end==119)  Car_Stop_flag=1;//出赛道保护
/*
   if(!fork_flag && track_type == TRACK_COMMON && Obstacle_flag == 0 && circle_flag == NUCIRCLE 
      && camera.dir_error < 10)
   {
    rampdect_head = MIN( MAX((int8)(1.33 * car.angle + 179.6),0) , 119);
    rampdect_tail = MAX( (int8)(1.31 * car.angle + 175.6) , 0);
    if(track_end<rampdect_tail && left.jump_point[0]<rampdect_tail && right.jump_point[0]<rampdect_tail && right.inf_point<rampdect_tail && left.inf_point<rampdect_tail
       && (Lline[rampdect_tail]-Lline[119])*(Rline[rampdect_tail]-Rline[119])<0
       && MMN_deriva(rampdect_head,rampdect_tail,left.First_deriva,'a')<=0.6 
       && MMN_deriva(rampdect_head,rampdect_tail,right.First_deriva,'b')>=-0.6 )
    {
      for(row = rampdect_head;row >= rampdect_tail;row--)
        rampdect_roadw += (Rline[row]-Lline[row]);
      rampdect_roadw = rampdect_roadw/(rampdect_head-rampdect_tail+1);
      row = (int)(0.0267*car.angle*car.angle+4.93*car.angle+262.3);
      ramp_rate = rampdect_roadw * 100 / row;
      if(ramp_rate > 130 && Ramp_flag==0 && Scratch_flag > 1)//检测上坡
      {
        Ramp_Rate[0] = ramp_rate;
        if(Ramp_Rate[0] > Ramp_Rate[1])
        {
          Ramp_Rate[1] = Ramp_Rate[0];
          rampup_count ++;
        }
        if(rampup_count >= 5)
        {
          Ramp_flag = 1;
          PTE2_OUT = 1;
        }
      }
//      else if(Ramp_flag == 1 && ramp_rate < 100)//检测坡顶
//      {
//        Ramp_flag = 2;
//      }
//      else if(Ramp_flag == 2 && ramp_rate > 150)//检测下坡
//      {
//        Ramp_flag = 3;
//      }
//      else if(Ramp_flag == 3 && ramp_rate < 100)//坡道结束
//      {
//        Ramp_flag = 0;
//        PTE2_OUT = 0;
//        rampup_count = 0;
//      }        
    }
    else
    {
      ramp_rate = 0;
      rampup_count = 0;
    }
   }
   else
   {
     ramp_rate = 0;
     rampup_count = 0;
   }*/
}

/******************************************************
* @function name : 计算偏差
* @ data : 2016/9/18
* @function description : 方向控制
******************************************************/
void camer_error_calculation()
{
  int8 i;
  float err = 0;
  camer.last_error = camer.error;
  for(i = 119; i > 119-dajiao_Prospect_See; i --)
  {
    err += (Mline[i] - 79)*Correction_array[i];
  }
  camer.error = err /dajiao_Prospect_See;//85
  
  if(camer.error-camer.last_error>20)//限幅滤波
  camer.error=camer.last_error+20;
  if(camer.error-camer.last_error<-20)//限幅滤波
  camer.error=camer.last_error-20;
  
  if(camer.error>150) camer.error=150;
  if(camer.error<-150) camer.error=-150;
}
/******************************************************
* @function name : 赛道扫描
* @ data : 2017/5/24
* @function description : 先扫描赛道边缘，再扫描赛道中间
******************************************************/
int Edge_Tracking()//扫描赛道边界
{
    int8 head_point=21,tail_point=21;
    int row,colum;
    head_point = StartPoint;
    tail_point = StartPoint;
    for(row = 119;row>-1;row --)
    {
      ///////////////扫左线///////////////////////////////////
       if(srcimg[row][head_point]==255 || row==119)//白向左扫 
       {
           for(colum = head_point;colum > -1;colum --)
           {
                if(colum==0 || (srcimg[row][colum]<255 && srcimg[row][colum+1]==255 && colum && colum<19) )//&& srcimg[row][colum-1]==0 
//                if(( ((srcimg[row][colum]<255 && srcimg[row][colum+1]==255)||(srcimg[row][colum]<255 && srcimg[row][colum-1]==0)) && colum && colum<19)
//                  ||(colum==0))
                {
                    val_area[row][0] = colum;break;
                }
           }
       }
       else                             //黑向右扫
       {
           for(colum = head_point;colum < tail_point && colum<20;colum ++)
           {
//               if((srcimg[row][colum] && srcimg[row][colum+1] && colum==0) || (colum && srcimg[row][colum]))//srcimg[row][colum]<255 && srcimg[row][colum+1]==255
                if(srcimg[row][colum])
                {
                    val_area[row][0] = colum;break;
                }
           }
       }
      ///////////////////扫右线/////////////////////////////////////
       if(srcimg[row][tail_point]==255 || row==119)//白向右扫 
       {
           for(colum = tail_point;colum < 20;colum ++)
           {
                if(colum==19 || (srcimg[row][colum]<255 && srcimg[row][colum-1]==255 && colum<19 && colum>0) )// && srcimg[row][colum+1]==0 
//                if(( ((srcimg[row][colum]<255 && srcimg[row][colum-1]==255)||(srcimg[row][colum]<255 && srcimg[row][colum+1]==0)) && colum<19 && colum>0)
//                   ||(colum==19))
                {
                    val_area[row][1] = colum;break;
                }
           }
       }
       else                         //黑向左扫
       {
           for(colum = tail_point;colum > head_point && colum > -1;colum --)
           {
//               if((srcimg[row][colum] && srcimg[row][colum-1] && colum==19) || (colum<19 && srcimg[row][colum]))// srcimg[row][colum]<255 && srcimg[row][colum-1]==255
                if(srcimg[row][colum])
                {
                    val_area[row][1] = colum;break;
                }
           }
       }
       
       if(val_area[row][0]>-1 && val_area[row][0]<20 && val_area[row][1]>-1 && val_area[row][1]<20)
       {
         head_point = val_area[row][0];
         tail_point = val_area[row][1];
       }
       else
       {
         head_point = 21;
         tail_point = 21;
       }
//       head_point = MIN(val_area[row][0]+1,19);
//       tail_point = MAX(val_area[row][1]-1,0);
       if(head_point == 21 || tail_point == 21 || head_point >= tail_point) break;
    }
    return row;
}
int8 Track_Scanning(uint8* fork_head,uint8* fork_tail,int track_end)//扫描赛道中间
{
    int row,colum;
    uint8 head=EMPTY,tail=EMPTY,head1=EMPTY,tail1=EMPTY,max_height=0;
    uint8 head_point=EMPTY,tail_point=EMPTY;
    uint8 end = MAX(track_end,20);
    for(row = 119;row > end;row --)
    {
        for(colum = val_area[row][0];colum < val_area[row][1];colum ++)
        {
            if(val_area[row][2] == EMPTY && srcimg[row][colum]<255 && srcimg[row][colum+1]<255)// && srcimg[row][colum-1]==255
                val_area[row][2] = colum;
            else if(val_area[row][2] != EMPTY && ((srcimg[row][colum]<255 && srcimg[row][colum+1]==255) || (srcimg[row][colum]<255 && colum+1==val_area[row][1])) )
            {
                val_area[row][3] = colum;break;
            }
        }
        if(val_area[row][3] != EMPTY)
        {
            head1 = row;
            for(row --;row > track_end;row --)
            {
                head_point = val_area[row+1][2];
                tail_point = val_area[row+1][3];
                if(head_point == val_area[row+1][0])    head_point++;
                if(tail_point == val_area[row+1][1])    tail_point--;
              ///////////////扫左线///////////////////////////////////
               if(srcimg[row][head_point]==255)//白向右扫
               {
                   for(colum = head_point;colum <= tail_point;colum ++)
                   {
                        if(srcimg[row][colum]<255)// && srcimg[row][colum-1]==255
                        {
                            val_area[row][2] = colum;break;
                        }
                   }
               }
               else                             //黑向左扫
               {
                   for(colum = head_point;colum >= val_area[row][0];colum --)
                   {
                       if(srcimg[row][colum]<255 && srcimg[row][colum-1]==255 && colum>val_area[row][0]+1)//srcimg[row][colum]<255 && srcimg[row][colum-1]==255
                        {
                            val_area[row][2] = colum;break;
                        }
                       else if(colum==val_area[row][0]+1 && srcimg[row][colum]<255)
                       {
                            if(srcimg[row][colum-1]%2)
                            {
                                val_area[row][2] = colum;break;
                            }
                            else
                            {
                                val_area[row][2] = colum-1;break;
                            }
                       }
                       else if(colum==val_area[row][0] && srcimg[row][colum+1]<255)
                       {
                            if(srcimg[row][colum]%2)
                            {
                                val_area[row][2] = colum+1;break;
                            }
                            else
                            {
                                val_area[row][2] = colum;break;
                            }
                       }
                   }
               }
              ///////////////////扫右线/////////////////////////////////////
               if(srcimg[row][tail_point]==255)//白向左扫
               {
                   for(colum = tail_point;colum >= head_point;colum --)
                   {
                        if(srcimg[row][colum]<255)// && srcimg[row][colum+1]==255
                        {
                            val_area[row][3] = colum;break;
                        }
                   }
               }
               else                         //黑向右扫
               {
                   for(colum = tail_point;colum <= val_area[row][1];colum ++)
                   {
                       if(srcimg[row][colum]<255 && srcimg[row][colum+1]==255 && colum<val_area[row][1]-1)//srcimg[row][colum]<255 && srcimg[row][colum+1]==255
                        {
                           val_area[row][3] = colum;break;
                        }
                       else if(colum==val_area[row][1]-1 && srcimg[row][colum]<255)
                       {
                            if(srcimg[row][colum+1]>=128)
                            {
                                val_area[row][3] = colum;break;
                            }
                            else 
                            {
                                val_area[row][3] = colum+1;break;
                            }
                       }
                       else if(colum==val_area[row][1] && srcimg[row][colum-1]<255)
                       {
                            if(srcimg[row][colum]>=128)
                            {
                                val_area[row][3] = colum-1;break;
                            }
                            else 
                            {
                                val_area[row][3] = colum;break;
                            }
                       }
                   }
               }
                if(val_area[row][2] == EMPTY || val_area[row][3] == EMPTY)  break;
            }
            tail1 = row+1;
            if(head1-tail1 > max_height)
            {
                head = head1;
                tail = tail1;
                max_height = head1-tail1;
            }
            if(max_height > UKBLACK_HEIGHT )
            {
                *fork_head = head;
                *fork_tail = tail;
                return 1;
            }
        }
    }
    return 0;
}
/******************************************************
* @function name : 赛道边界转化
* @ data : 2017/5/24
* @function description : 将赛道边界解压,获取真实赛道边界
******************************************************/
uint8 ConvertToBorder(uint8 pixnum,uint8 pixel,char flag)//转化边界
{
    uint8 count=0;
    uint8 x;
    int8 binary[8]={0},i=8;

    binary[7] = (pixel >> 7) & 0x01;
    binary[6] = (pixel >> 6) & 0x01;
    binary[5] = (pixel >> 5) & 0x01;
    binary[4] = (pixel >> 4) & 0x01;
    binary[3] = (pixel >> 3) & 0x01;
    binary[2] = (pixel >> 2) & 0x01;
    binary[1] = (pixel >> 1) & 0x01;
    binary[0] = (pixel >> 0) & 0x01;

    if(flag == 'l')
    {
        i = 7;
        while(!binary[i] && i>-1)
        {
            count ++;
            i --;
        }
        x = pixnum*8+count;
    }
    else 
    {
        i = 0;
        while(!binary[i] && i<8)
        {
            count ++;
            i ++;
        }
        x = (pixnum+1)*8-count-1;
    }
    return x;
}
uint8 AcquireRealBorder(uint8 fork_head,uint8 fork_tail,int8 fork_flag,int track_end)//得到赛道边界
{
    int8 row,col;
    uint8 max_width=0,maxw_row=EMPTY;//secw_row=EMPTY;
    for(row = 119;row > track_end;row --)
    {
        col = (uint8)val_area[row][0];border[row][0] = ConvertToBorder(col,srcimg[row][col],'l');Lline[row] = border[row][0];
        col = (uint8)val_area[row][1];border[row][1] = ConvertToBorder(col,srcimg[row][col],'r');Rline[row] = border[row][1];
    }
    if(fork_flag)
    {
        for(row = fork_head; row >= fork_tail; row --)
        {
            col = (uint8)val_area[row][2];border[row][2] = ConvertToBorder(col,srcimg[row][col],'r');
            col = (uint8)val_area[row][3];border[row][3] = ConvertToBorder(col,srcimg[row][col],'l');
            if(border[row][3]-border[row][2] > max_width)
            {
//                secw_row = maxw_row;
                maxw_row = row;
                max_width = border[row][3]-border[row][2];
            }
        }
    }
    return maxw_row;
}
/******************************************************
* @function name : 赛道滤波
* @ data : 2017/5/24
* @function description : 中位值滤波处理赛道噪声
******************************************************/
void TrackFilter(int16 start,int16 end,int16 * line)
{
    int8 i;
    int16 max,min;
    for(i=start - 1;i>end;i--)
    {
        if(*(line + i+1) > *(line + i))   {max=i+1;min=i;}
        else  {max=i;min=i+1;}
        if(*(line + max) > *(line + i-1))  line[i] = MAX(*(line + i-1),*(line + min));
        else    line[i] = MAX(*(line + max),*(line + min));
    }
}
/******************************************************
* @function name : 赛道信号处理
* @ data : 2017/5/24
* @function description : 计算赛道边界的斜率，跳变点，拐点
******************************************************/
void CalFirstDeriva(TRACK_INFM* track, int16* line,int end)//斜率
{
    int8 row = 117;
    while(row > end)
    {
        track->First_deriva[row] = (float)((line[row+2] - line[row])/2.0);
        row --;
    }
}
void AcquireInflecPoint(TRACK_INFM* track, int end, int16* line, char flag)//拐点
{
  int8 row;
  track->inf_point = 0;
  end = MAX(end,15);
  if(flag == 'l')
  {
   for(row = 115;row > end ; row --)//远处的拐点无意义 故只找到15行
    {
        if(((*(line+row)-*(line+row-4))*(*(line+row+4)-*(line+row)))<0 || ((*(line+row+4)-*(line+row))==0 && (*(line+row)-*(line+row-4))>2))//找左拐点的方法
        {
            if(*(line+row)==0)continue;
            if(track->inf_point == 0)
            {
                track->inf_point = row-3;
                break;      //找到拐点跳出循环
            }
        }
    }
  }
  else
  {
    for(row = 115;row > end ; row --)//远处的拐点无意义 故只找到15行
    {
        if( ((*(line+row)-*(line+row-4))*(*(line+row+4)-*(line+row)))<0 || ((*(line+row+4)-*(line+row))==0 && (*(line+row)-*(line+row-4))<-2) )//找左拐点的方法
        {
            if(*(line+row)==159)continue;
            if(track->inf_point == 0)
            {
                track->inf_point = row-3;
                break;      //找到拐点跳出循环
            }
        }
    }
  }
}
void AcquireJumpPoint(TRACK_INFM* track, int end, char flag)//跳变点
{
    int8 row,rising_flag,num=0;
    float variance;
    track->jump_point[0] = 0;
    track->jump_point[1] = 0;
    track->jump_point[2] = 0;
    end = MAX(end,20);
    for(row = 115;row > end;row --)
    {
        track->average[row] = (float)((track->First_deriva[row+2] + track->First_deriva[row+1] + track->First_deriva[row])/3.0);
        variance = calculate_variance(track->First_deriva[row+2], track->First_deriva[row+1], track->First_deriva[row], track->average[row]);
        if(variance > VARIANCE_THRESHOLD && rising_flag)
        {
            track->jump_point[num] = row+2;
            if(num && track->jump_point[num-1] - track->jump_point[num] < 5) num--;//相邻两跳变点距离过近则舍去
            num ++;
            if(num > 2) break;
            rising_flag = 0;//上升沿置0
        }
        else if(!rising_flag && variance < VARIANCE_THRESHOLD)
            rising_flag = 1;//下降沿置1
    }
    track->jump_count = num;
    if(flag == 'l')
    {
        while(num --)
        {
            row = track->jump_point[num];
            if(Lline[row+1]+Lline[row+2]+Lline[row+3] > Lline[row-1]+Lline[row-2]+Lline[row-3])     track->jump_point_state[num] = 1;
            else              track->jump_point_state[num] = 0;
        }
    }
    else
    {
        while(num --)
        {
            row = track->jump_point[num];
            if(Rline[row+1]+Rline[row+2]+Rline[row+3] > Rline[row-1]+Rline[row-2]+Rline[row-3])     track->jump_point_state[num] = 0;
            else              track->jump_point_state[num] = 1;
        }
    }
}
float calculate_variance(float a,float b, float c,float average)//方差
{
    float variance;
    variance = ((a - average)*(a - average)+(b - average)*(b - average)+(c - average)*(c - average))/3;
    return variance;
}
uint8 LostLineCount(int16 start, int16 end, int16* line, char flag)//计算丢线行数
{
    int8 row = start;
    uint8 count=0;
    if(flag == 'l')
    {
        while(row >= end)
        {
            if(*(line+row) < 5)    count ++;
            row --;
        }
    }
    else 
    {
        while(row >= end)
        {
            if(*(line+row) > 154)  count ++;
            row --;
        }
    }
    return count;
}
uint8 MaxMin(uint8 start,uint8 end,int16* line,char choose,char type)
{
    int16 value,temp;
    int row;
    value = *(line+start);
    temp = start;
    if(choose == 'a')
    {
        for(row = start-1;row >= end;row --)
        {
            if(*(line+row) > value)
            {
                value = *(line+row);
                temp =  row;
            }
        }
    }
    else
    {
        for(row = start-1;row >= end;row --)
        {
            if(*(line+row) < value)
            {
                value = *(line+row);
                temp =  row;
            }
        }
    }
    if(type == 'r')     return temp;
    else                return value;
}
int16 MMN(int16 *line,uint8 s,uint8 e,uint8 choose)
{
    int16 temp;
    if(choose==1)//最大值
    {
        temp=0;
        for(;s<e+1;s++)
        {
            if(temp<line[s])
                temp = line[s];
        }
    }
    else
    {
        temp = 159;
        for(;s<e+1;s++)
        {
            if(temp>line[s])
                temp = line[s];
        }
    }
    return temp;
}
/******************************************************
* @function name : 赛道判断
* @ data : 2017/5/24
* @function description : 判断赛道类型，斜十字，环形，障碍，起跑线
******************************************************/
track_state_e ObliqueCrossJudge(int8 infpoint,int8 maxw_row,uint8 fork_tail,uint8 fork_head,char flag)//斜十字
{
    int x,y;
    uint8 choose_row=(uint8)(MIN(infpoint+20,110));
    if(flag == 'l')
    {
        while(Lline[choose_row]==0 && choose_row>infpoint)    choose_row--;
        x = Lline[infpoint]-Lline[choose_row];
        y = Lline[choose_row] + x * (choose_row - maxw_row + 1) / (choose_row - infpoint);
        if(ABS(y-border[maxw_row][2]) > ABS(y-border[maxw_row][3]))// && maxw_row-fork_tail<2)
          return TRACK_OBLI_CRO_L;
    }
    else
    {
        while(Rline[choose_row]==159 && choose_row>infpoint)    choose_row--;
        x = Rline[infpoint]-Rline[choose_row];
        y = Rline[choose_row] + x * (choose_row - maxw_row + 1) / (choose_row - infpoint);
        if(ABS(y-border[maxw_row][2]) < ABS(y-border[maxw_row][3]))// && maxw_row-fork_tail<2
          return TRACK_OBLI_CRO_R;
    }
    return TRACK_COMMON;
}
track_state_e CircleJudge()//环形
{
    float k1=0,k2=0,k3=0;
    float b1=0,b2=0;
    int16 l_line[120]={0},r_line[120]={159};
    int8 low_Lpoint=0,low_Rpoint=0;
    int16 hx_Lpoint=0,hx_Rpoint=0,cow=0,Black_count=0,row;
    int16 r_cross_turn_point=right.inf_point,l_cross_turn_point=left.inf_point;
    low_Lpoint=l_cross_turn_point+18;
    low_Rpoint=r_cross_turn_point+18;
    if(low_Lpoint>=119)low_Lpoint=119;
    if(low_Rpoint>=119)low_Rpoint=119;
    k1=(float)(Lline[l_cross_turn_point+3]-Lline[low_Lpoint])/(float)(l_cross_turn_point+3-low_Lpoint);
    k2=(float)(Rline[low_Rpoint]-Rline[r_cross_turn_point+3])/(float)(low_Rpoint-r_cross_turn_point-3);    
    b1=Lline[l_cross_turn_point+3]-k1*(l_cross_turn_point+3);
    b2=Rline[r_cross_turn_point+3]-k2*(r_cross_turn_point+3);
    for(row=l_cross_turn_point;row>=0;row--)
    {
        l_line[row]=(int16)(k1*row+b1);
        if(l_line[row+1]<0 || l_line[row+1]>159)        break;
    }
    for(row=r_cross_turn_point;row>=0;row--)
    {
        r_line[row]=(int16)(k2*row+b2);
        if(r_line[row+1]<0 || r_line[row+1]>159)        break;
    }
    //////////////////////////////////////
    for(row=l_cross_turn_point-3;row>=0;row--)
    {
        if(255==img_handle[row+1][l_line[row+1]] && 0==img_handle[row][l_line[row]])
        {
         hx_Lpoint=row;   
         break;
        }
    }
    for(row=r_cross_turn_point-3;row>=0;row--)
    {
        if(255==img_handle[row+1][r_line[row+1]] && 0==img_handle[row][r_line[row]])
        {
         hx_Rpoint=row;
         break;
        }
    }
    ///////////////////////////////////////////////////////////
    if(hx_Lpoint && hx_Rpoint && r_line[row]-l_line[row]>10)
    {
        if(hx_Rpoint==hx_Lpoint)
        {
            for(cow=(l_line[hx_Lpoint]);cow<=(r_line[hx_Rpoint]);cow++)
            {
                Black_count+=img_handle[row][cow];
            }
            Black_count/=255;
        }
        else
        {
            k3=(float)((r_line[hx_Rpoint]-l_line[hx_Lpoint])*1.0/(hx_Rpoint-hx_Lpoint));
            for(cow=(l_line[hx_Lpoint]);cow<=(r_line[hx_Rpoint]);cow++)
            {
                row=(int8)(hx_Lpoint+(cow-l_line[hx_Lpoint])/k3);
                Black_count+=img_handle[row][cow];
            }
            Black_count/=255;
        }
    }
    else Black_count=10;//右线必须大于左线

    if(hx_Lpoint && hx_Rpoint && MAX(hx_Lpoint,hx_Rpoint)>15 && ABS(hx_Lpoint-hx_Rpoint)<15 && MAX(l_cross_turn_point,r_cross_turn_point)-MIN(hx_Lpoint,hx_Rpoint)>15 && 0==Black_count&& (k3>2.0 || k3<-2.0) && k1<=0 && k1>=-1.0 && k2>=0 && k2<=1.0 )//(fork_point && fork_point<MIN(l_cross_turn_point,r_cross_turn_point) && (Lline2[fork_point]+Rline2[fork_point])/2>l_line[fork_point]+4 && (Lline2[fork_point]+Rline2[fork_point])/2<r_line[fork_point]-4 && fork_long>3)
        return TRACK_CIRCLE;
    else
      return TRACK_COMMON;
}
track_state_e ScratchObstacleJudge(int8 maxw_row,uint8 fork_head,uint8 fork_tail)//障碍起跑
{
    int8 center=(border[maxw_row][0]+border[maxw_row][1])/2;
    int8 head=val_area[maxw_row][0],tail=val_area[maxw_row][1];
    int8 colum=val_area[maxw_row][0]+1,count1=0;//count2=0;
    while(colum < tail)
    {
        if(srcimg[maxw_row][colum] && srcimg[maxw_row][colum]<255) count1++;
//        else if(srcimg[maxw_row][colum] == 255) count2++;
        colum ++;
    }
    if(val_area[maxw_row][3]-val_area[maxw_row][2]+2>=val_area[maxw_row][1]-val_area[maxw_row][0] && count1>(tail-head)/2 && fork_head<110)
        return TRACK_SCRATCH;
    if( //border[maxw_row][3]-border[maxw_row][2])*4>(border[maxw_row][1]-border[maxw_row][0]) 
            (border[maxw_row][3]-border[maxw_row][2])*2<(border[maxw_row][1]-border[maxw_row][0]) && count1<3 && fork_head-fork_tail>15 && fork_head>30)//&& fork_head>25)count2>MAX((tail-head-1)/2,2)
    {
        if(border[maxw_row][2]>=center-10)      return TRACK_OBSTACLE_R;
        else if(border[maxw_row][3]<=center+10) return TRACK_OBSTACLE_L;
    }
    return TRACK_COMMON;
}
/******************************************************
* @function name : 赛道处理
* @ data : 2017/5/24
* @function description : 赛道补线，两点连线，向上向下延长
******************************************************/
int8 ExtendTheTrack(uint8 start, uint8 end, int16* line)
{
    int8 row;
    float det;
    det = (*(line+start) - *(line+start+5))/5.0;
//    det = det / 3;
    if(!det && (*(line+start)!=0 || *(line+start)!=159))    return 0;
    for(row = start-1;row >= end;row --)
        *(line+row) = *(line+start-2) + (int)((start-1-row)*det);
    return 1;
}
void matchline(int16 start,int16 startp,int16 end,int16 endp,int16 * line)//两点连线
{
    float a0,a1;
    int16 i,divide;
    divide = startp-endp;
     if(divide != 0)
     {
        a1 = (start-end)*1.0/divide;
        a0 = (start-a1*startp)*1.0;
         if(a1 != 0)
         {
            for(i = start-1;i > end;i --)
                line[i] = (int16)((i-a0)/a1);
         }
     }
     else
     {
        for(i = start-1;i > end;i --)
            line[i] = startp;
     }
}
/******************************************************
* @function name : 赛道处理
* @ data : 2017/5/24
* @function description : 获取赛道中线
******************************************************/
void aquire_Mline(int16* Lline,int16* Rline,int16* Mline)
{
    int8 i = 119;
    while(i>-1)
    {
        if( (*(Lline+i)!=0 && *(Rline+i)!=159) || (*(Lline+i)==0 && *(Rline+i)==159) )
            *(Mline+i) = (*(Lline+i) + *(Rline+i))/2;
        else if(*(Lline+i)>0 && *(Rline+i)==159)
            *(Mline+i) = *(Lline+i) + TRAOFF;
        else if(*(Lline+i)==0 && *(Rline+i)<159)
            *(Mline+i) = *(Rline+i) - TRAOFF;
//        Lline ++;
//        Rline ++;
//        Mline ++;
        i --;
    }
}
/******************************************************
* @function name : 赛道处理
* @ data : 2017/5/24
* @function description : 选择岔路
******************************************************/
void fork_select(int16* line,int8 a,int8 b,int8 c,char flag)
{
  int16 row;
  if(flag == 'l')
  {
    matchline(a,border[a][1],b,border[b][2],line);
    for(row = b;row >= c;row --)
        *(line+row) = border[row][2];
  }
  else
  {
    matchline(a,border[a][0],b,border[b][3],line);
    for(row = b;row >= c;row --)
        *(line+row) = border[row][3];
  }
}
void data_init()
{
  int row;
  for(row = 119;row > -1;row --)
  {
      val_area[row][0] = EMPTY;
      val_area[row][1] = EMPTY;
      val_area[row][2] = EMPTY;
      val_area[row][3] = EMPTY;
      
      border[row][0] = EMPTY;
      border[row][1] = EMPTY;
      border[row][2] = EMPTY;
      border[row][3] = EMPTY;
      
      Rline[row] = RINIT;
      Lline[row] = LINIT;
      Mline[row] = MINIT;
   }
//  fork_head = EMPTY;
//  fork_tail = EMPTY;
//  fork_flag = 0;
  left_track_tail=0;
  right_track_tail=0;
}
void ComCross_handle(TRACK_INFM* right,TRACK_INFM* left,int* left_track_tail, int* right_track_tail,int track_end)
{
    int16 l_count=0,r_count=0;
    int16 left_jump1=119,left_jump2=left->jump_point[0],right_jump1=119,right_jump2=right->jump_point[0];
//   while(l_count < left->jump_count)
//   {
//      if(left->jump_point_state[l_count])  left_jump1 = left->jump_point[l_count];
//      else {left_jump2 = left->jump_point[l_count];break;}
//      l_count ++;
//   }
//   if(left_jump2 && !left_jump1)  left_jump1 = 119;
//   while(r_count < right->jump_count)
//   {
//      if(right->jump_point_state[r_count])  right_jump1 = right->jump_point[r_count];
//      else {right_jump2 = right->jump_point[r_count];break;}
//      r_count ++;
//   }
//   if(right_jump2 && !right_jump1)    right_jump1 = 119;
   if(LostLineCount(left_jump1, left_jump2, Lline, 'l')>(left_jump1-left_jump2)/2
       && LostLineCount(right_jump1, right_jump2, Rline, 'r')>(right_jump1-right_jump2)/2
       && ABS(right_jump1-left_jump1)<20 && ABS(right_jump2-left_jump2)<20)
   {
       *left_track_tail = MAX(left->jump_point[l_count+1],track_end);
       *right_track_tail = MAX(right->jump_point[r_count+1],track_end);
       left_jump1=MIN(119,left_jump1+5);
       left_jump2 -= 5;
       right_jump1=MIN(119,right_jump1+5);
       right_jump2 -= 5;
       if(Lline[left_jump1]<Lline[left_jump2] && Rline[right_jump1]>Rline[right_jump2])
       {
         matchline(left_jump1, Lline[left_jump1], left_jump2, Lline[left_jump2], Lline);
         matchline(right_jump1, Rline[right_jump1], right_jump2, Rline[right_jump2], Rline);
       }
   }
}
float error_filter(float a,float b,float c)
{
  if(MAX(a,b) < c)      return MAX(a,b);
  else if( c < MIN(a,b) ) return MIN(a,b);
  else                  return c;
}
float MMN_deriva(uint8 start,uint8 end,float* line,char type)
{
    float value;
    int row;
    value = *(line+start);
    if(type == 'a')
    {
        for(row = start-1;row >= end;row --)
        {
            if(*(line+row) > value)
            {
                value = *(line+row);
            }
        }
    }
    else
    {
        for(row = start-1;row >= end;row --)
        {
            if(*(line+row) < value)
            {
                value = *(line+row);
            }
        }
    }
    return value;
}
track_state_e Obstacle_handle(uint8 fork_flag,uint8 fork_head,uint8 fork_tail,int track_end)
{
    track_state_e track_type;
    int row;
    static int16 OBS_Position;
    if(Obstacle_flag==1)//左障碍
    {
      track_type = TRACK_OBSTACLE_L;
      if(fork_flag && fork_head>OBS_Position)
      {
//        if(left.jump_count>1 && left.jump_point_state[0]==0 && left.jump_point_state[1]==1 && fork_tail-left.jump_point[0]==1)
//        {
//          fork_tail = left.jump_point[1];//left_track_tail = MAX(left.jump_point[2],track_end);
//        }
        OBS_Position = MAX(fork_tail-20,OBS_Position);
        for(row = MIN(fork_head+15,119);row >= fork_tail-15 && row > -1;row --)
          Lline[row] = Rline[row] - Obstacle_offset;
      }
      else if(left.jump_count>1 && left.jump_point_state[0]==0 && left.jump_point_state[1]==1 && left.jump_point[1]>MAX(track_end,40))
              //&& MaxMin(left.jump_point[0]-5,left.jump_point[1]+5,Lline,'i','v')>0)
      {
        //left_track_tail = MAX(left.jump_point[2],track_end);
        for(row = MIN(left.jump_point[0]+15,119);row >= left.jump_point[1]-15 && row > -1;row --)
          Lline[row] = Rline[row] - Obstacle_offset;
      }
      else if(left.jump_count && left.jump_point_state[0]==1 && left.jump_point[0]>MAX(track_end,40))
              //&& MaxMin(119,left.jump_point[0]+5,Lline,'i','v')>0)
      {
//        obstacle_position = left.jump_point[0]-5;
        //left_track_tail = MAX(left.jump_point[1],track_end);
        for(row = 119;row >= left.jump_point[0]-15 && row > -1;row --)
          Lline[row] = Rline[row] - Obstacle_offset;
      }
      else// if(left.jump_point[0]<obstacle_position)
      {
        OBS_Position = 0;
        Obstacle_flag = 0;
        track_type = TRACK_COMMON;
      }
    }
    else if(Obstacle_flag==2)//右障碍
    {
      track_type = TRACK_OBSTACLE_R;
      if(fork_flag && fork_head>OBS_Position)
      {
//        if(right.jump_count>1 && right.jump_point_state[0]==0 && right.jump_point_state[1]==1 && fork_tail-right.jump_point[0]==1)
//        {
//          fork_tail = right.jump_point[1];right_track_tail = MAX(right.jump_point[2],track_end);
//        }
//        srcnuma = fork_head;
        OBS_Position = MAX(fork_tail-20,OBS_Position);
        for(row = MIN(fork_head+15,119);row >= fork_tail-15 && row > -1;row --)
          Rline[row] = Lline[row] + Obstacle_offset;
      }
      else if(right.jump_count>1 && right.jump_point_state[0]==0 && right.jump_point_state[1]==1 && right.jump_point[1]>MAX(track_end,40))
//              && MaxMin(right.jump_point[0]-5,right.jump_point[1]+5,Lline,'a','v')<159)
      {
        //right_track_tail = MAX(right.jump_point[2],track_end);
        for(row = MIN(right.jump_point[0]+15,119);row >= right.jump_point[1]-15 && row > -1;row --)
          Rline[row] = Lline[row] + Obstacle_offset;
      }
      else if(right.jump_count && right.jump_point_state[0]==1 && right.jump_point[0]>MAX(track_end,40))
//              && MaxMin(119,right.jump_point[0]+5,Lline,'a','v')<159)
      {
//        obstacle_position = right.jump_point[0]-5;
        //right_track_tail = MAX(right.jump_point[1],track_end);
        for(row = 119;row >= right.jump_point[0]-15 && row > -1;row --)
          Rline[row] = Lline[row] + Obstacle_offset;
      }
      else// if(right.jump_point[0] < obstacle_position)
      {
        OBS_Position = 0;
        Obstacle_flag = 0;
        track_type = TRACK_COMMON;
      }
    }
    return track_type;
}
uint8 sratch_handle()
{
    int8 head=val_area[118][0],tail=val_area[118][1];
    int8 colum=val_area[118][0]+1,count=0;
    while(colum < tail)
    {
//        if(srcimg[118][colum] && srcimg[118][colum]<255) count++;
        if(srcimg[118][colum]==255) count++;
        colum ++;
    }
    if(count == tail-head-1 && count>8)    return 0;
    return 1;
//    if(count>(tail-head)/2)     return 1;
//    return 0;
}
void Circle_handle(int track_end,uint8 fork_flag,uint8 fork_head,uint8 fork_tail)
{
  static int8 CIRHAN_mark;
  int cir_start=0,cir_end,row=0;
//  float cdet;
  track_end = MAX(track_end,0);
  /**********************左环形*****************************/
  if(circle_flag <= LCIRCLE)
  {
    ///////////////////////////////////////////////////////
    if(circle_flag == LEFT || circle_flag == LCIRCLE)//环形前
    {
      if(fork_flag)
      {
        if(right.inf_point>fork_head)     cir_start = right.inf_point+3;// && right.inf_point<80
        else                              cir_start = 119;// while(Rline[cir_start] != 159 && cir_start > track_end)     cir_start --;}
        cir_end = fork_head-3;
        CIRHAN_mark = cir_end;
        while((Rline[cir_start]-Rline[cir_end])/(cir_start-cir_end)*1.0 < (border[cir_end+3][2]-border[cir_end][2])/3.0 && cir_end>fork_tail)      cir_end--;// && (border[cir_end+3][2]-border[cir_end][2])>=0
        fork_select(Rline,cir_start,cir_end,fork_tail,'l');
      }
      else
      {
        if(right.inf_point && right.inf_point > CIRHAN_mark)     {cir_end = right.inf_point;cir_start = right.inf_point;}
        else                    {cir_end = 119;cir_start = 119;}
        while(Rline[cir_end] != 159 && cir_end > track_end)     cir_end --;
        while(Rline[cir_end] == 159 && cir_end > track_end)     cir_end --;
        while((Rline[cir_start]-Rline[cir_end])/(cir_start-cir_end)*1.0 < (Rline[cir_end+3]-Rline[cir_end])/3.0 && cir_end>track_end)      cir_end--;// && (Rline[cir_end+3]-Rline[cir_end])>=0
        if(Rline[cir_end]>=Rline[cir_start])    
        {
          cir_end = cir_start;
          while(Lline[cir_end] != 0 && cir_end > track_end)     cir_end --;
          while(Lline[cir_end] == 0 && cir_end > track_end)     cir_end --;
          matchline(cir_start,Rline[cir_start],cir_end+3,Lline[cir_end+3],Rline);
          ExtendTheTrack(cir_end+6, 0, Rline);
          row = cir_end+3;
          while(row>0)  {Lline[row] = 0; row --;}
        }
        else
          matchline(cir_start,Rline[cir_start],cir_end,Rline[cir_end],Rline);
      }
      Prospect_See = cir_end;
    }
    /////////////////////////////////////////////////
    else if(circle_flag == LCIRCLE_IN)//环形中
    {
      if(fork_flag)// && fork_head>25
      {
        CIRHAN_mark = 120;
        for(row=fork_tail;row<=fork_head;row++)
          Rline[row] = border[row][2];
        matchline(119,Rline[119],fork_head-1,border[fork_head-1][2],Rline);
        cir_end = fork_head;
      }
      else if(CIRHAN_mark==120 && !fork_flag && left.jump_point[0]>30)
      {
        if(!left.jump_point_state[0])  cir_end = left.jump_point[0];
        else if(!left.jump_point_state[1] && left.jump_point[1]>30)     cir_end = left.jump_point[1];
        matchline(119,Rline[119],cir_end,Lline[cir_end],Rline);
        ExtendTheTrack(cir_end+3, 0, Rline);
        row = cir_end+3;
        while(row>0)  {Lline[row] = 0; row --;}
      }
      Prospect_See = cir_end;
    }
    //////////////////////////////////////////////
    else if(circle_flag == LCIRCLE_OUT)//出环形
    {
      CIRHAN_mark = -1;
      if(fork_flag)
      {
        fork_select(Rline,119,fork_head,fork_tail,'l');
        Prospect_See = fork_head;
      }
      else if(right.jump_point[0] && !right.jump_point_state[0])
      {
        row = right.jump_point[0]-5;
        matchline(119,border[119][1],row,border[row][1],Rline);
        Prospect_See = row;
      }
    }
  }
  /**********************右环形*****************************/
  else if(circle_flag <= RCIRCLE)///////////////////
  {
    if(circle_flag == RIGHT || circle_flag == RCIRCLE)
    {
      if(fork_flag)
      {
        if(left.inf_point>fork_head)      cir_start = left.inf_point+3;// && left.inf_point<80
        else                              cir_start = 119;
        cir_end = fork_head-3;
        CIRHAN_mark = cir_end;
        while((Lline[cir_start]-Lline[cir_end])/(cir_start-cir_end)*1.0 > (border[cir_end+3][3]-border[cir_end][3])/3.0 && cir_end>fork_tail && (border[cir_end+3][3]-border[cir_end][3])<=0)      cir_end--;
        fork_select(Lline,cir_start,cir_end,fork_tail,'r');
      }
      else
      {
        if(left.inf_point && left.inf_point>CIRHAN_mark)      {cir_start=left.inf_point;cir_end=left.inf_point;}
        else                    {cir_start = 119;cir_end = 119;}
        while(Lline[cir_end] != 0 && cir_end > track_end)     cir_end --;
        while(Lline[cir_end] == 0 && cir_end > track_end)     cir_end --;
        while((Lline[cir_start]-Lline[cir_end])/(cir_start-cir_end)*1.0 > (Lline[cir_end+3]-Lline[cir_end])/3.0 && cir_end>track_end && (Lline[cir_end+3]-Lline[cir_end])<=0)      cir_end--;
        if(Lline[cir_end]<=Lline[cir_start])
        {
          cir_end = cir_start;
          while(Rline[cir_end] != 159 && cir_end > track_end)     cir_end --;
          while(Rline[cir_end] == 159 && cir_end > track_end)     cir_end --;
          matchline(cir_start,Lline[cir_start],cir_end+3,Rline[cir_end+3],Lline);
          ExtendTheTrack(cir_end, 0, Lline);
          row = cir_end+3;
          while(row > 0)        {Rline[row] = 159;row --;}
        }
        else 
          matchline(cir_start,Lline[cir_start],cir_end,Lline[cir_end],Lline);
      }
      Prospect_See = cir_end;
    }
    else if(circle_flag == RCIRCLE_IN)
    {
      if(fork_flag)// && fork_head>25
      {
        CIRHAN_mark = 120;
        for(row=fork_tail;row<=fork_head;row++)
          Lline[row] = border[row][3];
        matchline(119,Lline[119],fork_head-1,border[fork_head-1][3],Lline);
        cir_end = fork_head;
      }
      else if(CIRHAN_mark==120 && !fork_flag && right.jump_point[0]>30)
      {
        if(!right.jump_point_state[0])  cir_end = right.jump_point[0];
        else if(!right.jump_point_state[1] && right.jump_point[1]>30)     cir_end = right.jump_point[1];
        matchline(119,Lline[119],cir_end,Rline[cir_end],Lline);
        ExtendTheTrack(cir_end+3, 0, Lline);
        row = cir_end+3;
        while(row>0)  {Rline[row] = 159; row --;}
      }
      Prospect_See = cir_end;
    }
    else if(circle_flag == RCIRCLE_OUT)
    {
      CIRHAN_mark = -1;
      if(fork_flag)
      {
        Prospect_See = fork_head;
        fork_select(Lline,119,fork_head,fork_tail,'r');
      }
      else if(left.jump_point[0] && !left.jump_point_state[0])
      {
        row = left.jump_point[0]-5;
        matchline(119,border[119][0],row,border[row][0],Lline);
        Prospect_See = row;
      }
    }
  }
}
void Circle_flag_handle(int8 fork_flag,uint8 fork_head,uint8 fork_tail)
{
  if(circle_flag <= LCIRCLE)//////////////////左环形
  {
    if(circle_flag == LEFT)
    {
      if(MaxMin(119,100,Rline,'a','v')==159 && fork_head-fork_tail<10 && (left.inf_point<fork_head || left.inf_point>105) )
        circle_flag = LCIRCLE;
    }
    else if(circle_flag == LCIRCLE)
    {
      if(MaxMin(119,99,Rline,'a','v')<159)
        circle_flag = LCIRCLE_IN;
    }
    else if(circle_flag == LCIRCLE_IN)
    {
      if(MaxMin(119,115,Rline,'a','v')==159 && fork_head>40)
        circle_flag = LCIRCLE_OUT;
    }
    else if(circle_flag == LCIRCLE_OUT)
    {
      if(MaxMin(119,99,Rline,'a','v')<159)// && MaxMin(119,99,Lline,'i','v')>0  && (Lline[99]-Lline[119])*(Rline[99]-Rline[119])<0 ) 
      {
          PTE2_OUT = 0;
          circle_flag = NUCIRCLE;
      }
    }
  }
  else
  {
   if(circle_flag == RIGHT)
   {
     if(MaxMin(119,100,Lline,'i','v')==0 && fork_head-fork_tail<10 && (right.inf_point<fork_head || right.inf_point>105) )
          circle_flag = RCIRCLE;
   }
    else if(circle_flag == RCIRCLE)
    {
      if(MaxMin(119,99,Lline,'i','v')>0)
          circle_flag = RCIRCLE_IN;
    }
    else if(circle_flag == RCIRCLE_IN)
    {
      if(MaxMin(119,115,Lline,'a','v')==0 && fork_head>40)
          circle_flag = RCIRCLE_OUT;
    }
    else if(circle_flag == RCIRCLE_OUT)
    {
      if(MaxMin(119,99,Lline,'i','v')>0)// && MaxMin(119,99,Rline,'a','v')<159 && (Lline[99]-Lline[119])*(Rline[99]-Rline[119])<0 )
      {
          PTE2_OUT = 0;
          circle_flag = NUCIRCLE;
      }
    }
  }
}