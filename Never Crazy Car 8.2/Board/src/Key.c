#include "Key.h"
#include "include.h"


uint8 KeyValue=5;

int8 Key_Test()
{
  if(!key1)
  {
    beep();
    KeyValue=1;
    while(!key1);
  }
  else if(!key2)
  {
    beep();
    KeyValue=2;
    while(!key2);
  }
  else if(!key3)
  {
    beep();
    KeyValue=3;
    while(!key3);
  }
  else if(!key4)
  {
    beep();
    KeyValue=4;
    while(!key4);
  }
  else
    KeyValue=5;
  return KeyValue;
}

//void Display()
//{
//    LED_P6x8Str(0,0,"voltage :");
//    LED_PrintsignValueF4(58, 0,camer.error );
//      
//}