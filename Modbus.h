/******************************************************************
//						Modbus接口头文件	
//	
//作为Modbus从机时，RTU模式使用一次性收完模式， ASC采用单个收发模式
//主机或从机模式均适用
//此模块在原Mosbus基础上，在标准化的UsartDev上书写与设计
//注:此模块不负责Usart的通讯参数设置
****************************************************************/
#ifndef __MODBUS_H
#define __MODBUS_H

/************************************************************************
                             相关配置
************************************************************************/

//定义USART硬件是否支持数据超时完成,若支持，硬件在超时后自动结束,
//否则Modbus层将在收发到每个数后手动触发上层定时器完成。
//注：RTU模式,使用辅助定时器时,不应支持,否则数据可能会被断开。
//#define MODBUS_SUPPORT_USART_OV_FINAL   

//定义在从机在等待接收到第一个数前的超时时间，
//若长时间没有收到数据，MODBUS将重新启动以防止异常死锁
//注:主机模式没有此功能
#define MODBUS_SLAVE_WAIT_OV   200


//定义在ASC模式时，内部定时器超时结束值
#define MODBUS_ASC_OV_FINAL   10
  
//定义在RTU模式时，等待一帧数据的最长时间
#define MODBUS_RTU_OV_FINAL   20



/*********************************************************************
                             相关结构
***********************************************************************/

//Modbus工作状态定义
enum _Modbus_eState
{
	Modbus_eState_Idie			  = 0,	//处于空闲态,等待起始条件
	Modbus_eState_RdWait		  = 1,	//在读等待状态，等待起始条件
	Modbus_eState_RdReady		  = 2,	//起始条件已建立,准备接收数据状态
	Modbus_eState_RdData		  = 3,  //符合本机地址,接收数据状态
	Modbus_eState_RdDataEnd	      = 4,  //已接收完成,置为数据尾
	Modbus_eState_RdDataDone	  = 5,	//准备接收数据完成,分析数据阶段
	Modbus_eState_WrData		  = 6,	//发送数据状态,回应主机数据
	Modbus_eState_WrFinal		  = 7	  //数据发送完成但还需完成结束标志			
};

//主结构
#include "UsartDev.h"
struct _Modbus{
  //基本部分：
  enum _Modbus_eState eState;    //Modbus状态机
  unsigned char Flag;            //相关标志,见定义
  unsigned char Adr;              //当前通讯地址
  unsigned char Timer;             //定时器值
  //底层数据收发相关:
  void *pVoid;                         //回调函数可能需要的指针
  struct _UsartDev *pUsartDev;   //挂接的底层设备
  unsigned char *pBuf;           //数据缓冲区,接收发送通用
  unsigned short Count;          //数据缓冲区大小
  unsigned short Len;            //有效数据长度
  //回调函数
  signed char (*Notify)(struct _Modbus *pModbus,
                unsigned char ErrFlag); //标关标志,见Flag错误定义
};

//其中，相关标志Flag定义为：
#define MODBUS_ASC         0x80   //ASC标志，否则工作在RTU模式
#define MODBUS_HOST        0x40  //主机标志，否则工作在从机模式
#define MODBUS_ROUTER      0x20  //路由模式，此模式任何地址均收
//运行状态控制位:
#define MODBUS_RUN_MASK    0x1F  //具体定义为:
#define MODBUS_ZERO_ADR    0x10  //零地址标志
#define MODBUS_ERR_MASK    0x0F  //错误标志,具体定义为:
#define MODBUS_ERR_OTHER   0x08  //其它错误标志
#define MODBUS_ERR_ADR     0x04  //地址错误标志
#define MODBUS_ERR_DATA    0x02  //数据有错误标志
#define MODBUS_ERR_STATE   0x01  //状态机错误标志

/******************************************************************************
                             相关函数
******************************************************************************/


//--------------------------初始化函数------------------------------------
void Modbus_Init(struct _Modbus *pModbus, //未初始化的对像
                 unsigned char Flag,      //标关标志
                 void *pVoid,                    //回调函数可能需要的指针
                 struct _UsartDev *pUsartDev,   //挂接的已初始化底层设备
                 unsigned char *pBuf,           //数据缓冲区
                 unsigned short Count,          //数据缓冲区大小
                 //回调函数,可能会在中断里调用,ErrFlag见定义
                 signed char (*Notify)(struct _Modbus *pModbus,
                                       unsigned char ErrFlag));
//ErrFlag定义为:
//0: 正常，或发送起始通报
//0xff 超时通报
//0xfe 按收到首个数通报  
//0xfd 按收到首个异常数通报(ASC模式时非:)                               
//<16: 按收过程错误类型，见MODBUS_ERR_OTHER定义

//----------------------发送数据完成中断回调函数------------------------
//此函数可用于恢复MODBUS通讯
signed char Modbus_cbSendInt(void *pvUsartDev);

//-------------------------重设定地址函数------------------------
//void Modbus_SetAdr(struct _Modbus *pModbus,
//                  unsigned char Adr);
#define Modbus_SetAdr(pmodbus,adr) do{(pmodbus)->Adr = adr;}while(0)

//-------------------------重配置函数------------------------
//调用前应置停止状态
//void Modbus_ReCfg(struct _Modbus *pModbus,
//                  unsigned char Flag);
#define Modbus_ReCfg(pmodbus,flag) \
  do{(pmodbus)->Flag = (flag) & (MODBUS_ASC | MODBUS_HOST);}while(0)
    
//-----------------------重置ASC与RTU函数------------------------
//调用前应置停止状态
#define Modbus_SetAsc(pmodbus) do{(pmodbus)->Flag |= MODBUS_ASC;}while(0)
#define Modbus_SetRtu(pmodbus) do{(pmodbus)->Flag &= ~MODBUS_ASC;}while(0)

//-------------------------收数据启动函数------------------------
//返回是否启动成功
signed char  Modbus_RcvStart(struct _Modbus *pModbus);

//----------------------发送数据启动函数----------------------------
//返回是否启动成功
signed char Modbus_SendStart(struct _Modbus *pModbus,
                             unsigned char Adr,        //要发送的设备地址
                             unsigned char *pData,     //要发送的数据指针
                             unsigned short Len,       //数据长度
                             unsigned char IsDisCheck);//是否禁止校验

//---------------------判断是否成功接收完成函数----------------------------
//返回是否启动成功
//enum _Modbus_eState Modbus_IsRdDataDone(struct _Modbus *pModbus)
#define Modbus_IsRdDataDone(pModbus) \
    ((pModbus)->eState == Modbus_eState_RdDataDone)

//----------------------强制停止函数------------------------
//强制停止并置为空闲状态
void Modbus_Stop(struct _Modbus *pModbus);

//----------------------收完数据解码函数------------------------
//返回数据是否错误
signed char Modbus_RcvDataDecode(struct _Modbus *pModbus);

//----------------------得到接收数据函数------------------------
//数据解码正确时才能调用此函数
//返回数据格式为：地址0+功能码1+数据...
unsigned char *Modbus_pGetRcvData(struct _Modbus *pModbus);

//----------------------Modbus得到接收数据长度函数------------------------
//数据解码正确时才能调用此函数
//含地址位置大小
//unsigned short Modbus_GetRcvLen(struct _Modbus *pModbus);
#define Modbus_GetRcvLen(pmodbus) ((pmodbus)->Len)

//-----------------Modbus得到数据发送处理缓冲区函数------------------------
//使用内建缓冲区时调用此函数
//返回缓冲区位置及大小
unsigned char *Modbus_pGetSendBuf(struct _Modbus *pModbus,
                                  unsigned short *pCount); //返回可用缓冲区大小

//-------------------------任务函数----------------------------
//此代码放入任务中以自动恢复从机错误
//注作为主机时需与主机处于同一进程,也可由主机程序自行处理
void Modbus_Task(struct _Modbus *pModbus);

//-------------------------Rtu模式硬件定时器溢出处理函数----------------------------
//工作在RTU模式时，需由外部实现3.5个字符复位功能。
//若Usart直接支持，此函数可不调用
void Modbus_RtuHwTimerOv(struct _Modbus *pModbus);

/******************************************************************************
                           回调函数
******************************************************************************/
//#include "ModbusSlave.h"

//-------------------------由底层UsartDev得到Modbus对像-----------------------
//struct _Modbus *Modbus_cbGet(const struct _UsartDev *pUsartDev);
#define Modbus_cbGet(usartDev) ((struct _Modbus *)((usartDev)->pVoid))
//#define Modbus_cbGet(pUsartDev)  ((struct _Modbus *)0)//测试时
//#define Modbus_cbGet(pUsartDev)  (&ModbusSlave.Modbus)

//-----------------------硬件定时器启动并复位函数-----------------------------
//工作在RTU模式时，需由外部实现3.5个字符复位功能。
//若Usart直接支持，此函数须定义为空
void Modbus_cbHwTimerReset(struct _Modbus *pModbus);
//#define Modbus_cbHwTimerReset(pModbus)  do{}while(0)

//------------------------------硬件定时器停止函数-----------------------------
//工作在RTU模式时，需由外部实现3.5个字符复位功能。
//若Usart直接支持，此函数须定义为空
void Modbus_cbHwTimerStop(struct _Modbus *pModbus);
//#define Modbus_cbHwTimerStop(pModbus)  do{}while(0)

//-------------------------通报开始接收数据函数-----------------------------
//可用于实现RTS的IO控制
#ifdef SUPPORT_MUTI_USART
  void Modbus_cbRcvStartNotify(struct _Modbus *pModbus);
#else //只支持1个时直接用Ioctrl实现:
  #include "IoCtrl.h"
  #define Modbus_cbRcvStartNotify(pModbus)  do{ClrRS485RTS();}while(0)
#endif

//-------------------------通报开始发送数据函数-----------------------------
//可用于实现RTS的IO控制
#ifdef SUPPORT_MUTI_USART
  void Modbus_cbSendStartNotify(struct _Modbus *pModbus);
#else //只支持1个时直接用Ioctrl实现:
  #define Modbus_cbSendStartNotify(pModbus)  do{SetRS485RTS();}while(0)
#endif
  
#endif //#ifndef __MODBUS_H

