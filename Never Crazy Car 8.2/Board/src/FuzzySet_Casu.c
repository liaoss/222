#include "FuzzySet_Casu.h"
#include "include.h"

const int16 QMAX = 110 ;       //大于440的值将被当成 440
const int16 QMIN = -110 ;       //小于PMIN的值将被当成      赛道越简单，速度越快，否则越是复杂就越慢
const int16 RMAX = 45;       //这个值表示图像后RMAX行空的多少，空的越多，速度越慢
const int16 RMIN = 0;       //      //
const int16 TMAX = 98;      //两个参数都统一到0~100的值（归一化）
const int16 TMIN = 2 ;
const int16 OMAX = 100;     //语言值的满幅值

static int16 PMAX = 50;
static int16 PMIN = 3; 
              // 0  1   2   3   4   5   6
//int16 PYY[7] = { 2, 18, 34, 50, 66, 82, 98 };     //统一到最大值； 最小值；
int16 DYY[7] = { 2, 18, 34, 50, 66, 82, 98 };
int16 PYY[13] = { 2, 10, 18, 26, 34, 42, 50, 58, 66, 74, 82, 90, 98 };     //统一到最大值； 最小值；
//int16 DYY[13] = { 2, 10, 18, 26, 34, 42, 50, 58, 66, 74, 82, 90, 98 };     //
/*输出量U语言值特征点    0    1    2    3    4    5    6 */
 //数组里面的数值为速度
//正跑速度表

//int16 UYY1[7] =     {  -250, -130, -30, 0, 30, 130, 250 };
//int16 UYY2[7] =     {  -220, -120, -30, 0, 30, 120, 220 };
int16 UYY1[13] ={-350,-290,-240,-180, -130, -70, 0, 70, 130, 180, 240, 290 ,350};//{-330,-275,-220,-165, -110, -55, 0, 55, 110, 165, 220, 275 ,330};
float UYY_P1[13] ={0.82, 0.80, 0.79, 0.78, 0.74, 0.69, 0.64, 0.69, 0.74, 0.78, 0.79, 0.80 ,0.82};
//0.82, 0.81, 0.80, 0.76, 0.72, 0.70, 0.68, 0.70, 0.72, 0.76, 0.80, 0.81 ,0.82
//0.83, 0.81, 0.80, 0.79, 0.75, 0.70, 0.65, 0.70, 0.75, 0.79, 0.80, 0.81 ,0.83
//0.85, 0.83, 0.81, 0.80, 0.72, 0.68, 0.65, 0.68, 0.72, 0.80, 0.81, 0.83 ,0.85
float UYY_D1[13] ={3.6, 3.2, 2.8, 2.4, 1.2, 1.0, 1.0, 1.0, 1.2, 2.4, 2.8, 3.2 ,3.6};

//uint16 rulu[7][7] =
//{
////    /*P*/  //  0,  1,  2,  3,  4,  5,  6    /*D*/
//  /* 0 */                   {  1,  1,  1,  1,  0,  0,  0,} , //U语言数组里面的下标
//  /* 1 */                   {  2,  2,  2,  1,  1,  1,  1,} ,
//  /* 2 */                   {  3,  3,  2,  2,  2,  2,  2,} ,
//  /* 3 */                   {  3,  3,  3,  3,  3,  3,  3,} ,
//  /* 4 */                   {  3,  3,  4,  4,  4,  4,  4,} ,
//  /* 5 */                   {  4,  4,  4,  5,  5,  5,  5,} ,
//  /* 6 */                   {  5,  5,  5,  5,  6,  6,  6,}
//};

uint16 rule_Casu[13][7] =
{
//    /*P*/                //  0,  1,  2,  3,  4,  5,  6 /*D*/
  /* 0 */                   {  4,  1,  0,  0,  0,  0,  0,} , //U语言数组里面的下标
  /* 1 */                   {  4,  1,  1,  0,  0,  0,  0,} ,
  /* 2 */                   {  5,  2,  1,  1,  0,  0,  0,} ,
  /* 3 */                   {  5,  3,  2,  1,  1,  0,  0,} ,
  /* 4 */                   {  6,  4,  3,  2,  1,  1,  0,} ,
  /* 5 */                   {  6,  5,  4,  3,  2,  1,  1,} ,
  /* 6 */                   {  6,  6,  6,  6,  6,  6,  6,} ,
  /* 7 */                   {  6,  7,  8,  9,  10, 11, 11,} , //U语言数组里面的下标
  /* 8 */                   {  6,  8,  9,  10, 11, 11, 12,} ,
  /* 9 */                   {  7,  9,  10, 11, 11, 12, 12,} ,
  /* 10 */                  {  7,  10, 11, 11, 12, 12, 12,} ,
  /* 11 */                  {  8,  11, 11, 12, 12, 12, 12,} ,
  /* 12 */                  {  8,  11, 12, 12, 12, 12, 12,}
};
uint16  rule_steer_P[13][7]=
{
//    /*P*/                //  0,  1,  2,  3,  4,  5,  6 /*D*/
  /* 0 */                   {  2,  1,  0,  0,  0,  0,  0,} , //U语言数组里面的下标
  /* 1 */                   {  4,  2,  0,  0,  0,  0,  0,} ,
  /* 2 */                   {  5,  2,  0,  0,  0,  0,  0,} ,
  /* 3 */                   {  6,  4,  2,  0,  0,  0,  0,} ,
  /* 4 */                   {  6,  6,  3,  2,  0,  0,  0,} ,
  /* 5 */                   {  6,  6,  4,  3,  1,  0,  0,} ,
  /* 6 */                   {  6,  6,  6,  6,  6,  6,  6,} ,
  /* 7 */                   {  6,  6,  8,  9, 11, 12, 12,} ,  //U语言数组里面的下标
  /* 8 */                   {  6,  6,  9,  10, 12, 12, 12,} ,
  /* 9 */                   {  6,  8,  10, 12, 12, 12, 12,} ,
  /* 10 */                  {  7,  10, 12, 12, 12, 12, 11,} ,
  /* 11 */                  {  8,  10, 12, 12, 12, 12, 12,} ,
  /* 12 */                  {  10, 11, 12, 12, 12, 12, 12,}
};
uint16  rule_steer_D[13][7]=
{
//    /*P*/                //  0,  1,  2,  3,  4,  5,  6 /*D*/
  /* 0 */                   {  0,  0,  0,  0,  0,  0,  0,} , //U语言数组里面的下标
  /* 1 */                   {  1,  0,  0,  0,  0,  0,  0,} ,
  /* 2 */                   {  2,  1,  0,  0,  0,  0,  0,} ,
  /* 3 */                   {  3,  2,  1,  0,  0,  0,  0,} ,
  /* 4 */                   {  4,  3,  2,  1,  0,  0,  0,} ,
  /* 5 */                   {  5,  4,  3,  2,  1,  0,  0,} ,
  /* 6 */                   {  6,  6,  6,  6,  6,  6,  6,} ,
  /* 7 */                   {  7,  8,  9,  10, 11, 12, 12,} ,  //U语言数组里面的下标
  /* 8 */                   {  8,  9,  10, 11, 12, 12, 12,} ,
  /* 9 */                   {  9,  10, 11, 12, 12, 12, 12,} ,
  /* 10 */                  {  10, 11, 12, 12, 12, 12, 12,} ,
  /* 11 */                  {  11, 12, 12, 12, 12, 12, 12,} ,
  /* 12 */                  {  12, 12, 12, 12, 12, 12, 12,}
};

int16 *UYY=UYY1;  //这样可以选择不用的U语言数组
float *UYY_P=UYY_P1;
float *UYY_D=UYY_D1;

/*  P 代表赛道复杂程度
D 表示前瞻
模糊控制算法通过这两个跟定量以及规则表给出当前应该输出的差速
*/
int16 FuzzySet_Casu(int16 P, int16 D)          //模糊运算引擎，返回速度值
{
   float  U;       /*偏差，以及输出值的精确量 */
   uint16 PF[2];
   uint16 DF[2];
   //uint16 UF[4];  /*偏差，偏差微分以及输出值的隶属度PF[1]是P的隶属度，PF[0]是隶属度的补集 */
   int16 Pn = 0, Dn = 0;
   int16 Un[4];
  // int32 temp1,temp2;
   float temp1,temp2;
   if(P < QMIN)
     P = QMIN;
   else
     if(P > QMAX)
       P = QMAX;

   P = (int16)((double)(P - QMIN) / (QMAX - QMIN) * (TMAX - TMIN) + TMIN); //归一化到TMIN ~ TMAX

   if(D < RMIN)
     D = RMIN;
   else
     if( D > RMAX)
       D = RMAX;

   D = (int16)((double)(D - RMIN) / (RMAX - RMIN) * (TMAX - TMIN) + TMIN) ;    //归一化到TMIN ~ TMAX

   /*隶属度的确定*/
   /*根据PD的指定语言获得有效的隶属度*/

   if(P > PYY[0] && P < PYY[12])
   {
        if (P <= PYY[1])
        {
          Pn = 1;
          PF[0] = (uint16)(OMAX * ((float)(PYY[1] - P) / (PYY[1] - PYY[0])));
        }
        else if (P <= PYY[2])
        {
          Pn = 2;
          PF[0] = (uint16)(OMAX * ((float)(PYY[2] - P) / (PYY[2] - PYY[1])));
        }
        else if (P <= PYY[3])
        {
          Pn = 3;
          PF[0] = (uint16)(OMAX * ((float)(PYY[3] - P) / (PYY[3] - PYY[2])));
        }
        else if (P <= PYY[4])
        {
          Pn = 4;
          PF[0] = (uint16)(OMAX * ((float)(PYY[4] - P) / (PYY[4] - PYY[3])));
        }
        else if (P <= PYY[5])
        {
          Pn = 5;
          PF[0] = (uint16)(OMAX * ((float)(PYY[5] - P) / (PYY[5] - PYY[4])));
        }
        else if (P <= PYY[6])
        {
          Pn = 6;
          PF[0] = (uint16)(OMAX * ((float)(PYY[6] - P) / (PYY[6] - PYY[5])));
        }
        else if (P <= PYY[7])
        {
          Pn = 7;
          PF[0] = (uint16)(OMAX * ((float)(PYY[7] - P) / (PYY[7] - PYY[6])));
        }
        else if (P <= PYY[8])
        {
          Pn = 8;
          PF[0] = (uint16)(OMAX * ((float)(PYY[8] - P) / (PYY[8] - PYY[7])));
        }
        else if (P <= PYY[9])
        {
          Pn = 9;
          PF[0] = (uint16)(OMAX * ((float)(PYY[9] - P) / (PYY[9] - PYY[8])));
        }
        else if (P <= PYY[10])
        {
          Pn = 10;
          PF[0] = (uint16)(OMAX * ((float)(PYY[10] - P) / (PYY[10] - PYY[9])));
        }
        else if (P <= PYY[11])
        {
          Pn = 11;
          PF[0] = (uint16)(OMAX * ((float)(PYY[11] - P) / (PYY[11] - PYY[10])));
        }
        else if (P <= PYY[12])
        {
          Pn = 12;
          PF[0] = (uint16)(OMAX * ((float)(PYY[12] - P) / (PYY[12] - PYY[11])));
        }
   }
   else if (P <= PYY[0])
   {
        Pn = 1;
        PF[0] = (uint16)(OMAX);
    }
   else if (P >= PYY[12])
   {
        Pn = 12;
        PF[0] = 0;
   }

   PF[1] = (uint16)(OMAX - PF[0]);


      if (D > DYY[0] && D < DYY[6])
      {
        if (D <= DYY[1])
        {
          Dn = 1;
          DF[0] = (uint16)(OMAX * ((float)(DYY[1] - D) / (DYY[1] - DYY[0])));
        }
        else if (D <= DYY[2])
        {
          Dn = 2;
          DF[0] = (uint16)(OMAX * ((float)(DYY[2] - D) / (DYY[2] - DYY[1])));
        }
        else if (D <= DYY[3])
        {
          Dn = 3;
          DF[0] = (uint16)(OMAX * ((float)(DYY[3] - D) / (DYY[3] - DYY[2])));
        }
        else if (D <= DYY[4])
        {
          Dn = 4;
          DF[0] = (uint16)(OMAX * ((float)(DYY[4] - D) / (DYY[4] - DYY[3])));
        }
        else if (D <= DYY[5])
        {
          Dn = 5;
          DF[0] = (uint16)(OMAX * ((float)(DYY[5] - D) / (DYY[5] - DYY[4])));
        }
        else if (D <= DYY[6])
        {
          Dn = 6;
          DF[0] = (uint16)(OMAX * ((float)(DYY[6] - D) / (DYY[6] - DYY[5])));
        }
      }
      else if (D <= DYY[0])
      {
        Dn = 1;
        DF[0] = (uint16)(OMAX);
      }
      else if (D >= DYY[6])
      {
        Dn = 6;
        DF[0] = 0;
      }
      DF[1] = (uint16)(OMAX - DF[0]);

      Un[0] = rule_Casu[Pn - 1][ Dn - 1];
      Un[1] = rule_Casu[Pn][ Dn - 1];
      Un[2] = rule_Casu[Pn - 1][ Dn];
      Un[3] = rule_Casu[Pn][ Dn];
      Un[0] = UYY[Un[0]];
      Un[1] = UYY[Un[1]];
      Un[2] = UYY[Un[2]];
      Un[3] = UYY[Un[3]];
     /*（双线性差值法反模糊）*/
     temp1=((float)DF[1]/OMAX*(Un[2]-Un[0])+Un[0]);
     temp2=((float)DF[1]/OMAX*(Un[3]-Un[1])+Un[1]);
     U=((float)PF[1]/OMAX*(temp2-temp1)+temp1);
     //U = (int16)(temp1 / temp2);
      return (int16)U;
}
float FuzzySet_steer_P(int16 P, int16 D)
{
   float U;       /*偏差，以及输出值的精确量 */
   uint16 PF[2];
   uint16 DF[2];
   //uint16 UF[4];  /*偏差，偏差微分以及输出值的隶属度PF[1]是P的隶属度，PF[0]是隶属度的补集 */
   int16 Pn = 0, Dn = 0;
   float Un[4];// int32 temp1,temp2;
   float temp1,temp2;
   if(P < QMIN)
     P = QMIN;
   else
     if(P > QMAX)
       P = QMAX;

   P = (int16)((double)(P - QMIN) / (QMAX - QMIN) * (TMAX - TMIN) + TMIN); //归一化到TMIN ~ TMAX

   if(D < PMIN)
     D = PMIN;
   else
     if( D > PMAX)
       D = PMAX;

   D = (int16)((double)(D - PMIN) / (PMAX - PMIN) * (TMAX - TMIN) + TMIN) ;    //归一化到TMIN ~ TMAX

   /*隶属度的确定*/
   /*根据PD的指定语言获得有效的隶属度*/

   if(P > PYY[0] && P < PYY[12])
   {
        if (P <= PYY[1])
        {
          Pn = 1;
          PF[0] = (uint16)(OMAX * ((float)(PYY[1] - P) / (PYY[1] - PYY[0])));
        }
        else if (P <= PYY[2])
        {
          Pn = 2;
          PF[0] = (uint16)(OMAX * ((float)(PYY[2] - P) / (PYY[2] - PYY[1])));
        }
        else if (P <= PYY[3])
        {
          Pn = 3;
          PF[0] = (uint16)(OMAX * ((float)(PYY[3] - P) / (PYY[3] - PYY[2])));
        }
        else if (P <= PYY[4])
        {
          Pn = 4;
          PF[0] = (uint16)(OMAX * ((float)(PYY[4] - P) / (PYY[4] - PYY[3])));
        }
        else if (P <= PYY[5])
        {
          Pn = 5;
          PF[0] = (uint16)(OMAX * ((float)(PYY[5] - P) / (PYY[5] - PYY[4])));
        }
        else if (P <= PYY[6])
        {
          Pn = 6;
          PF[0] = (uint16)(OMAX * ((float)(PYY[6] - P) / (PYY[6] - PYY[5])));
        }
        else if (P <= PYY[7])
        {
          Pn = 7;
          PF[0] = (uint16)(OMAX * ((float)(PYY[7] - P) / (PYY[7] - PYY[6])));
        }
        else if (P <= PYY[8])
        {
          Pn = 8;
          PF[0] = (uint16)(OMAX * ((float)(PYY[8] - P) / (PYY[8] - PYY[7])));
        }
        else if (P <= PYY[9])
        {
          Pn = 9;
          PF[0] = (uint16)(OMAX * ((float)(PYY[9] - P) / (PYY[9] - PYY[8])));
        }
        else if (P <= PYY[10])
        {
          Pn = 10;
          PF[0] = (uint16)(OMAX * ((float)(PYY[10] - P) / (PYY[10] - PYY[9])));
        }
        else if (P <= PYY[11])
        {
          Pn = 11;
          PF[0] = (uint16)(OMAX * ((float)(PYY[11] - P) / (PYY[11] - PYY[10])));
        }
        else if (P <= PYY[12])
        {
          Pn = 12;
          PF[0] = (uint16)(OMAX * ((float)(PYY[12] - P) / (PYY[12] - PYY[11])));
        }
   }
   else if (P <= PYY[0])
   {
        Pn = 1;
        PF[0] = (uint16)(OMAX);
    }
   else if (P >= PYY[12])
   {
        Pn = 12;
        PF[0] = 0;
   }

   PF[1] = (uint16)(OMAX - PF[0]);


      if (D > DYY[0] && D < DYY[6])
      {
        if (D <= DYY[1])
        {
          Dn = 1;
          DF[0] = (uint16)(OMAX * ((float)(DYY[1] - D) / (DYY[1] - DYY[0])));
        }
        else if (D <= DYY[2])
        {
          Dn = 2;
          DF[0] = (uint16)(OMAX * ((float)(DYY[2] - D) / (DYY[2] - DYY[1])));
        }
        else if (D <= DYY[3])
        {
          Dn = 3;
          DF[0] = (uint16)(OMAX * ((float)(DYY[3] - D) / (DYY[3] - DYY[2])));
        }
        else if (D <= DYY[4])
        {
          Dn = 4;
          DF[0] = (uint16)(OMAX * ((float)(DYY[4] - D) / (DYY[4] - DYY[3])));
        }
        else if (D <= DYY[5])
        {
          Dn = 5;
          DF[0] = (uint16)(OMAX * ((float)(DYY[5] - D) / (DYY[5] - DYY[4])));
        }
        else if (D <= DYY[6])
        {
          Dn = 6;
          DF[0] = (uint16)(OMAX * ((float)(DYY[6] - D) / (DYY[6] - DYY[5])));
        }
      }
      else if (D <= DYY[0])
      {
        Dn = 1;
        DF[0] = (uint16)(OMAX);
      }
      else if (D >= DYY[6])
      {
        Dn = 6;
        DF[0] = 0;
      }
      DF[1] = (uint16)(OMAX - DF[0]);

      Un[0] = (float)rule_steer_P[Pn - 1][ Dn - 1];
      Un[1] = (float)rule_steer_P[Pn][ Dn - 1];
      Un[2] = (float)rule_steer_P[Pn - 1][ Dn];
      Un[3] = (float)rule_steer_P[Pn][ Dn];
      Un[0] = UYY_P[(uint16)Un[0]];
      Un[1] = UYY_P[(uint16)Un[1]];
      Un[2] = UYY_P[(uint16)Un[2]];
      Un[3] = UYY_P[(uint16)Un[3]];
     /*（双线性差值法反模糊）*/
     temp1=((float)DF[1]/OMAX*(Un[2]-Un[0])+Un[0]);
     temp2=((float)DF[1]/OMAX*(Un[3]-Un[1])+Un[1]);
     U=((float)PF[1]/OMAX*(temp2-temp1)+temp1);//U = (int16)(temp1 / temp2);
      return U;

}
float FuzzySet_steer_D(int16 P, int16 D)
{
   float U;       /*偏差，以及输出值的精确量 */
   uint16 PF[2];
   uint16 DF[2];
   //uint16 UF[4];  /*偏差，偏差微分以及输出值的隶属度PF[1]是P的隶属度，PF[0]是隶属度的补集 */
   int16 Pn = 0, Dn = 0;
   float Un[4];// int32 temp1,temp2;
   float temp1,temp2;
   if(P < QMIN)
     P = QMIN;
   else
     if(P > QMAX)
       P = QMAX;

   P = (int16)((double)(P - QMIN) / (QMAX - QMIN) * (TMAX - TMIN) + TMIN); //归一化到TMIN ~ TMAX

   if(D < RMIN)
     D = RMIN;
   else
     if( D > RMAX)
       D = RMAX;

   D = (int16)((double)(D - RMIN) / (RMAX - RMIN) * (TMAX - TMIN) + TMIN) ;    //归一化到TMIN ~ TMAX

   /*隶属度的确定*/
   /*根据PD的指定语言获得有效的隶属度*/

   if(P > PYY[0] && P < PYY[12])
   {
        if (P <= PYY[1])
        {
          Pn = 1;
          PF[0] = (uint16)(OMAX * ((float)(PYY[1] - P) / (PYY[1] - PYY[0])));
        }
        else if (P <= PYY[2])
        {
          Pn = 2;
          PF[0] = (uint16)(OMAX * ((float)(PYY[2] - P) / (PYY[2] - PYY[1])));
        }
        else if (P <= PYY[3])
        {
          Pn = 3;
          PF[0] = (uint16)(OMAX * ((float)(PYY[3] - P) / (PYY[3] - PYY[2])));
        }
        else if (P <= PYY[4])
        {
          Pn = 4;
          PF[0] = (uint16)(OMAX * ((float)(PYY[4] - P) / (PYY[4] - PYY[3])));
        }
        else if (P <= PYY[5])
        {
          Pn = 5;
          PF[0] = (uint16)(OMAX * ((float)(PYY[5] - P) / (PYY[5] - PYY[4])));
        }
        else if (P <= PYY[6])
        {
          Pn = 6;
          PF[0] = (uint16)(OMAX * ((float)(PYY[6] - P) / (PYY[6] - PYY[5])));
        }
        else if (P <= PYY[7])
        {
          Pn = 7;
          PF[0] = (uint16)(OMAX * ((float)(PYY[7] - P) / (PYY[7] - PYY[6])));
        }
        else if (P <= PYY[8])
        {
          Pn = 8;
          PF[0] = (uint16)(OMAX * ((float)(PYY[8] - P) / (PYY[8] - PYY[7])));
        }
        else if (P <= PYY[9])
        {
          Pn = 9;
          PF[0] = (uint16)(OMAX * ((float)(PYY[9] - P) / (PYY[9] - PYY[8])));
        }
        else if (P <= PYY[10])
        {
          Pn = 10;
          PF[0] = (uint16)(OMAX * ((float)(PYY[10] - P) / (PYY[10] - PYY[9])));
        }
        else if (P <= PYY[11])
        {
          Pn = 11;
          PF[0] = (uint16)(OMAX * ((float)(PYY[11] - P) / (PYY[11] - PYY[10])));
        }
        else if (P <= PYY[12])
        {
          Pn = 12;
          PF[0] = (uint16)(OMAX * ((float)(PYY[12] - P) / (PYY[12] - PYY[11])));
        }
   }
   else if (P <= PYY[0])
   {
        Pn = 1;
        PF[0] = (uint16)(OMAX);
    }
   else if (P >= PYY[12])
   {
        Pn = 12;
        PF[0] = 0;
   }

   PF[1] = (uint16)(OMAX - PF[0]);


      if (D > DYY[0] && D < DYY[6])
      {
        if (D <= DYY[1])
        {
          Dn = 1;
          DF[0] = (uint16)(OMAX * ((float)(DYY[1] - D) / (DYY[1] - DYY[0])));
        }
        else if (D <= DYY[2])
        {
          Dn = 2;
          DF[0] = (uint16)(OMAX * ((float)(DYY[2] - D) / (DYY[2] - DYY[1])));
        }
        else if (D <= DYY[3])
        {
          Dn = 3;
          DF[0] = (uint16)(OMAX * ((float)(DYY[3] - D) / (DYY[3] - DYY[2])));
        }
        else if (D <= DYY[4])
        {
          Dn = 4;
          DF[0] = (uint16)(OMAX * ((float)(DYY[4] - D) / (DYY[4] - DYY[3])));
        }
        else if (D <= DYY[5])
        {
          Dn = 5;
          DF[0] = (uint16)(OMAX * ((float)(DYY[5] - D) / (DYY[5] - DYY[4])));
        }
        else if (D <= DYY[6])
        {
          Dn = 6;
          DF[0] = (uint16)(OMAX * ((float)(DYY[6] - D) / (DYY[6] - DYY[5])));
        }
      }
      else if (D <= DYY[0])
      {
        Dn = 1;
        DF[0] = (uint16)(OMAX);
      }
      else if (D >= DYY[6])
      {
        Dn = 6;
        DF[0] = 0;
      }
      DF[1] = (uint16)(OMAX - DF[0]);

      Un[0] = (float)rule_steer_D[Pn - 1][ Dn - 1];
      Un[1] = (float)rule_steer_D[Pn][ Dn - 1];
      Un[2] = (float)rule_steer_D[Pn - 1][ Dn];
      Un[3] = (float)rule_steer_D[Pn][ Dn];
      Un[0] = UYY_D[(uint16)Un[0]];
      Un[1] = UYY_D[(uint16)Un[1]];
      Un[2] = UYY_D[(uint16)Un[2]];
      Un[3] = UYY_D[(uint16)Un[3]];
     /*（双线性差值法反模糊）*/
     temp1=((float)DF[1]/OMAX*(Un[2]-Un[0])+Un[0]);
     temp2=((float)DF[1]/OMAX*(Un[3]-Un[1])+Un[1]);
     U=((float)PF[1]/OMAX*(temp2-temp1)+temp1);
     //U = (int16)(temp1 / temp2);
      return U;

}