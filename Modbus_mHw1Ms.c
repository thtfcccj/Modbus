/*******************************************************************************

        多个Modbus时,所需硬件定时器(RTU模式)在有1Ms定时任务中的实现

*******************************************************************************/


#include "Modbus_mHw1Ms.h"
#include <string.h>

//-----------------------------初始化函数-----------------------------
void Modbus_mHw1Ms_Init(struct _Modbus_mHw1Ms *pHw1Ms,
                        struct _Modbus *pModbus)
{
  pHw1Ms->pModbus = pModbus;  
  pHw1Ms->Ms = 0;
  Modbus_mHw1Ms_UpdateOv(pHw1Ms, 9600);//不设置默认9600
}

//-------------------------由波特率更新溢出值函数-----------------------------
void Modbus_mHw1Ms_UpdateOv(struct _Modbus_mHw1Ms *pHw1Ms,
                            unsigned long Buad)//波特率
{
  //公式为: 字符数 * (1000ms / (波特率 / 数据位数))
  //根据Modbus计算3.5个字符(以5个字符为准)间隔,,按每个字符10bit计算
  pHw1Ms->Ov = (unsigned long)(5 * (1000 * 10)) / Buad;
  if(pHw1Ms->Ov < 2) pHw1Ms->Ov = 2;
}

//-------------------------------任务函数-------------------------------
//放入1ms或<1ms进程或中断内
void Modbus_mHw1Ms_Task(struct _Modbus_mHw1Ms *pHw1Ms)
{
  if(!pHw1Ms->Ms) return;
  pHw1Ms->Ms--;
  if(pHw1Ms->Ms) return;
  //定时到了
  Modbus_RtuHwTimerOv(pHw1Ms->pModbus);
}

/*******************************************************************************
                        MODBUS Hw相关回调函数实现
*******************************************************************************/

//-----------------------硬件定时器启动并复位函数-----------------------------
//工作在RTU模式时，需由外部实现3.5个字符复位功能。
//若Usart直接支持，此函数须定义为空
void Modbus_cbHwTimerReset(struct _Modbus *pModbus)
{
  struct _Modbus_mHw1Ms *pHw1Ms = Modbus_mHw1Ms_cbpGet(pModbus);
  if(pHw1Ms != NULL) pHw1Ms->Ms = pHw1Ms->Ov;
}

//------------------------------硬件定时器停止函数-----------------------------
//工作在RTU模式时，需由外部实现3.5个字符复位功能。
//若Usart直接支持，此函数须定义为空
void Modbus_cbHwTimerStop(struct _Modbus *pModbus)
{
  struct _Modbus_mHw1Ms *pHw1Ms = Modbus_mHw1Ms_cbpGet(pModbus);
  if(pHw1Ms != NULL) pHw1Ms->Ms = 0;
}

