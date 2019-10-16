/******************************************************************
//
//						Modbus对像实现
//
****************************************************************/

#include "Modbus.h"
#include  "NetData.h"  //校验等数据处理
#include  <string.h>

//--------------------------初始化函数------------------------------------
void Modbus_Init(struct _Modbus *pModbus, //未初始化的对像
                 unsigned char Flag,      //标关标志
                 void *pVoid,                         //回调函数可能需要的指针
                 struct _UsartDev *pUsartDev,   //挂接的已初始化底层设备
                 unsigned char *pBuf,           //数据缓冲区
                 unsigned short Count,          //数据缓冲区大小
                 //回调函数,可能会在中断里调用
                 signed char (*Notify)(struct _Modbus *pModbus,
                                       unsigned char ErrFlag))
{
  memset(pModbus, 0, sizeof(struct _Modbus));
  pModbus->Flag = (Flag & ~MODBUS_RUN_MASK);
  pModbus->pVoid = pVoid;
  pModbus->pUsartDev = pUsartDev;
  pModbus->pBuf = pBuf;  
  pModbus->Count = Count; 
  pModbus->Notify = Notify;   
  Modbus_cbHwTimerStop(pModbus); //先停止
  pModbus->pUsartDev->SendEndInt = Modbus_cbSendInt;//先挂载发送
}

//----------------------RTU模式接收数据中断回调函数------------------------
//此函数结合外部的硬件定时器实现状态机与数据的收发
static signed char Modbus_cbRtuRcvInt(void *pvUsartDev)
{
  struct _UsartDev *pUsartDev = (struct _UsartDev*)pvUsartDev;
  struct _Modbus *pModbus = Modbus_cbGet(pUsartDev);
  unsigned char Flag;
  
  switch(pModbus->eState){
  //在等待状态时，收到第一个数据为地址
  case Modbus_eState_RdWait:{
    #ifdef MODBUS_SUPPORT_USART_OV_FINAL
      //重新置为快速接收模式(此时已收到一个数据了)
      UsartDev_RcvStart(pModbus->pUsartDev,
                        pModbus->pBuf + 1,
                        (pModbus->Count - 1) | 0x8000,
                        Modbus_cbRtuRcvInt);
      pModbus->eState = Modbus_eState_RdDataEnd;  //预置接收数据完成状态 
    #else //继续当前接收
      pModbus->eState = Modbus_eState_RdData;    
    #endif
    Modbus_cbHwTimerReset(pModbus);//通知定时器复位
    pModbus->Notify(pModbus,0xfe); //首个数通报
    return 0; //继续接收
  }
  //接收数据状态,非MODBUS_SUPPORT_USART_OV_FINAL进入
  case Modbus_eState_RdData:
    Modbus_cbHwTimerReset(pModbus);//通知定时器复位
    return 0; //继续接收
  //接收完数据状态及其它状态:
  //case Modbus_eState_RdDataEnd:
  default:{ //其它状态
    break;
  }
  }//end   switch(pModbus->eState){
  
  //状态错误或接收数据完成处理：
  Modbus_cbHwTimerStop(pModbus);  //通知定时器停止 
  //检查错误
  Flag = 0;//假定正确
  if(pModbus->eState != Modbus_eState_RdDataEnd)//状态机错误
    Flag = MODBUS_ERR_STATE;
  else if(pUsartDev->Flag & USART_DEV_RCV_ERR)//数据错误
    Flag = MODBUS_ERR_DATA;
  else if(*pModbus->pBuf == 0) //广播地址
    Flag = MODBUS_ZERO_ADR;
  else if(pModbus->Flag & MODBUS_ROUTER){} //路由模式无条件收数据
  else if(*pModbus->pBuf != pModbus->Adr)//不是希望的地址 
    Flag = MODBUS_ERR_ADR;
  //接收完成处理:
  #ifdef MODBUS_SUPPORT_USART_OV_FINAL
    pModbus->Len = pUsartDev->RcvLen + 1;//接收到的个数,因重新启动接收了,需加上地址位
  #else
    pModbus->Len = pUsartDev->RcvLen;
  #endif
  if(Flag & MODBUS_ERR_MASK)
    pModbus->eState = Modbus_eState_Idie; //错误时预置空闲状态
  else 
    pModbus->eState = Modbus_eState_RdDataDone; //接收数据完成
  pModbus->Flag |= Flag;
  
  //错误或数据收完了
  return pModbus->Notify(pModbus,Flag); //通报
  
  //return -1;//停止
}

//----------------------ASC模式接收数据中断回调函数------------------------
//此函数结合内部的定时器实现状态机与数据的收发
static signed char Modbus_cbAscRcvInt(void *pvUsartDev)
{
  struct _UsartDev *pUsartDev = (struct _UsartDev*)pvUsartDev;
  struct _Modbus *pModbus = Modbus_cbGet(pUsartDev);
  unsigned char *pRcvBuf = pUsartDev->pRcvBuf;
  
  unsigned short CurPos = pUsartDev->RcvLen - 1;
  unsigned char RcvData = *(pRcvBuf + CurPos);//读取最后一个数据
  unsigned char Flag = 0;    

  pModbus->Timer = MODBUS_ASC_OV_FINAL; //内部定时器复位
  
  //无条件去掉最高位,在适用于STM32等不支持7数据位
  *(pRcvBuf + CurPos) &= ~0x80; 
  RcvData &= ~0x80; 
  
  //接收到前导字符且数据未接收完成时,强制复位,准备接收数据  
  if(RcvData == ':'){
      pModbus->eState = Modbus_eState_RdReady;
      *pRcvBuf = ':';
      pUsartDev->RcvLen = 1;
      pModbus->Notify(pModbus,0xfe); //首个数正确通报
      return 0;//继续接收
  }
  //状态机处理:

  switch(pModbus->eState){
  //等待数据状态收到非":"
  case Modbus_eState_RdWait: 
    pUsartDev->RcvLen = 0;//丢掉,重新开始接收
    //pModbus->Notify(pModbus,0xfd); //首个数通报,首个数异常
    return 0;//继续接收    
   //接收到起始位,准备接收地址   
  case Modbus_eState_RdReady:{
    if(CurPos == 2){//判断一二两位
      unsigned char CurAdr = (*(pRcvBuf + 2) - '0') | //地址低位ASC
                             (( *(pRcvBuf + 1) - '0') << 4); //地址高位ASC
      if(CurAdr == 0) //广播地址
        Flag = MODBUS_ZERO_ADR;
      else if(pModbus->Flag & MODBUS_ROUTER){} //路由模式无条件收数据
      else if(CurAdr != pModbus->Adr)//不是希望的地址 
        Flag = MODBUS_ERR_ADR;
      //无错误时转换状态机到接收数据状态
      if(!(Flag & MODBUS_ERR_MASK)){
        pModbus->eState = Modbus_eState_RdData;
      }
    }
    else if(CurPos != 1)
      Flag = MODBUS_ERR_OTHER;//其它错误
    break;
  }
  //数据接收状态
  case Modbus_eState_RdData:
    if(CurPos >= pModbus->Count)
      Flag = MODBUS_ERR_DATA;//数据溢出
    //地址符合时接收到回车字符,到达数据尾
    else if(RcvData == 0x0d){
      if(CurPos >= 6)
        pModbus->eState = Modbus_eState_RdDataEnd;
      else 
        Flag = MODBUS_ERR_OTHER;//其它错误,数据区数据不够,重新开始
    }
    break;
  //接收到换行字符
  case Modbus_eState_RdDataEnd:
    if(RcvData != 0x0a)
      Flag = MODBUS_ERR_OTHER;//其它错误,换行字符错误
    else{//数据正确
      //读取数据完成,交由主程序处理
      pModbus->eState = Modbus_eState_RdDataDone;
      pModbus->Len = CurPos + 1;//接收到的个数
      pModbus->Timer = 0;  //定时器停止
    }
    break;
  default: //状态机错误
    Flag = MODBUS_ERR_STATE;
    break;
  }
  
  if(Flag & MODBUS_ERR_MASK)//错误时强制结束
    pModbus->eState = Modbus_eState_Idie; //错误时预置空闲状态
  else if(pModbus->eState != Modbus_eState_RdDataDone)
    return 0;//没有收完时继续接收
    
  //错误或数据收完了
  return pModbus->Notify(pModbus,Flag); //通报
  
  //return -1;//停止
}

//------------------------收数据启动函数------------------------
//返回是否启动成功
signed char  Modbus_RcvStart(struct _Modbus *pModbus)
{
  Modbus_Stop(pModbus);//先强制停止
  Modbus_cbRcvStartNotify(pModbus);//通报开始接收数据
  pModbus->Flag &= ~MODBUS_RUN_MASK;//标志清零
  
  if(pModbus->Flag & MODBUS_ASC){
    UsartDev_RcvStart(pModbus->pUsartDev,
                      pModbus->pBuf,
                      pModbus->Count,
                      Modbus_cbAscRcvInt);                   
  }
  else{
    UsartDev_RcvStart(pModbus->pUsartDev,
                      pModbus->pBuf,
                      pModbus->Count,
                      Modbus_cbRtuRcvInt);
    Modbus_cbHwTimerStop(pModbus);  //通知定时器先停止等待第一个数
  }
  if(!(pModbus->Flag & MODBUS_HOST))//从机模式时置定时器防止异常死锁
    pModbus->Timer = 200;
  else pModbus->Timer = 0; //主机模式依靠外部实现等待超时
  
  pModbus->eState = Modbus_eState_RdWait;   //置为等待接收状态
  return 0;
}

//----------------------强制停止函数------------------------
//强制停止并置为空闲状态
void Modbus_Stop(struct _Modbus *pModbus)
{
  //被劫持(其它设备如WIFI正在配置，未进入正常模式)时直接退出
  if(pModbus->pUsartDev->SendEndInt == Modbus_cbSendInt){
    UsartDev_RcvStop(pModbus->pUsartDev);
    Modbus_cbRcvStartNotify(pModbus);//强制预置接收数据状态
    Modbus_cbHwTimerStop(pModbus);
    UsartDev_SendStop(pModbus->pUsartDev);
  }
  pModbus->eState = Modbus_eState_Idie;
}

//----------------------发送数据完成中断回调函数------------------------
//此函数在发送数据完成后调用
static signed char Modbus_cbSendInt(void *pvUsartDev)
{
  struct _UsartDev *pUsartDev = (struct _UsartDev*)pvUsartDev;
  struct _Modbus *pModbus = Modbus_cbGet(pUsartDev);
  pModbus->eState = Modbus_eState_WrFinal;//置为写数据完成状态
  
  //假定为全部全送正确
  pModbus->Notify(pModbus,0);//发送里调用,发送通报
  Modbus_cbRcvStartNotify(pModbus);//强制预置接收数据状态
  
  Modbus_RcvStart(pModbus); //预置为接收模式接收数据
  return 1;
}

//----------------------发送数据启动函数----------------------------
//返回是否启动成功
signed char Modbus_SendStart(struct _Modbus *pModbus,
                             unsigned char Adr,        //要发送的设备地址
                             unsigned char *pData,     //要发送的数据指针
                             unsigned short Len,       //数据长度
                             unsigned char IsDisCheck) //是否禁止校验
{
  unsigned char *pBuf;
  unsigned short CRC16;
  
  //被劫持(其它设备如WIFI正在配置，未进入正常模式)时直接退出
  if(pModbus->pUsartDev->SendEndInt != Modbus_cbSendInt) 
    return -1;

  Modbus_Stop(pModbus);//先强制停止
  pBuf = pModbus->pBuf;

  if(pModbus->Flag & MODBUS_HOST) //主机模式时更新地址
    pModbus->Adr = Adr;
  if(pModbus->Flag & MODBUS_ASC){//ASC格式时
    if((Len << 1) > (pModbus->Count - 7)) return -1;    //发送数据过长
    //若不在原缓冲区里(:和地址各占一位)，则先copy到该位置
    if(pData != (pBuf + 2)) memcpy(pBuf + 2,pData,Len);
    *pBuf++ = ':'; 
    *pBuf = Adr;
    Len += 1; //包含地址了
    *(pBuf + Len) = GetLRC(pBuf,Len);   //得到LRC数据校验码,含地址
    Len += 1;//包含LRC校验码了
    Len = DataToAsc(pBuf,pBuf,Len); //转换为ASCII,留开始字符与地址位置
    //准备校验码与生成开始结束标志
    pBuf += Len;      //到达结束位置了
    *pBuf++ = 0x0d;
    *pBuf++ = 0x0a;//++为计算个数
    pModbus->Len = pBuf - pModbus->pBuf;
  }
  else{ //RSC工作模式
    if((Len + 3) > pModbus->Count) return -1;    //发送数据过长
    //若不在原缓冲区里(地址占一位)，则先copy到该位置
    if(pData != (pBuf + 1)) memcpy(pBuf + 1,pData,Len);
    *pBuf = Adr;
    Len += 1; //地址位
    if(!IsDisCheck){//没有禁止增加校验码时
      CRC16 = GetCRC16(pBuf,Len); //得到CRC16校验码
      pBuf += Len;
      *pBuf++ = (unsigned char)(CRC16 >> 8);//CRC高位
      *pBuf = (unsigned char)(CRC16 & 0xff);//CRC低位
      pModbus->Len = Len + 2; //CRC16占位
    }
    else pModbus->Len = Len;
  }
  //启动发送
  Modbus_cbSendStartNotify(pModbus);//通报开始发送数据
  UsartDev_SendStart(pModbus->pUsartDev,
                     pModbus->pBuf,
                     pModbus->Len | 0x8000,//工作于快速模式
                     Modbus_cbSendInt);
  
  pModbus->eState = Modbus_eState_WrData;//置为写数据状态
  return 0;
}


//----------------------收完数据解码函数------------------------
//返回数据是否错误
signed char Modbus_RcvDataDecode(struct _Modbus *pModbus)
{
  unsigned char *pBuf = pModbus->pBuf;
  unsigned short Len = pModbus->Len;
  unsigned short Crc;

  if(pModbus->Flag & MODBUS_ASC){//ASC格式时
    if(Len < 4) return -2;//异常错误
    pBuf++;
    Len = AscToData(pBuf,pBuf,Len - 3); //转换为ASCII转换为数据,不含开始与结束标志
    Len--;
    if(*(pBuf + Len) != GetLRC(pBuf,Len)) return -1;   //LRC数据校验码错误
  }
  //RSC工作模式
  else{
    if(Len < 2) return -2;//异常错误
    Len-= 2;//CRC点两位
    Crc = (unsigned short)(*(pBuf + Len) << 8) | 
      (unsigned short)(*(pBuf + Len + 1));
    if(Crc != GetCRC16(pBuf,Len)) return -1;   //CRC数据校验码错误
  }
  pModbus->Len = Len;
  return 0;
}

//----------------------得到接收数据函数------------------------
//数据解码正确时才能调用此函数
//返回数据格式为：地址0+功能码1+数据...
unsigned char *Modbus_pGetRcvData(struct _Modbus *pModbus)
{
  if(pModbus->Flag & MODBUS_ASC)//ASC格式时
    return pModbus->pBuf + 1;
  //RTU模式时
  return pModbus->pBuf;
}

//-----------------Modbus得到数据发送处理缓冲区函数------------------------
//使用内建缓冲区时调用此函数
//返回缓冲区位置及大小
unsigned char *Modbus_pGetSendBuf(struct _Modbus *pModbus,
                                  unsigned short *pCount) //返回可用缓冲区大小
{
  if(pModbus->Flag & MODBUS_ASC){//ASC格式时,数据减半并除去相关控制位置
    *pCount = ((pModbus->Count) >> 1) - 8;//:,地址，LRCH,LRCL
    return pModbus->pBuf + 2;     //解码前,":'和地址占位
  }
  //RTU模式时
  *pCount = pModbus->Count - 2;//仅CRC16占位
  return pModbus->pBuf + 1;    //地址占位  
}

//-------------------------任务函数函数----------------------------
//此代码放入任务中以自动恢复从机错误
//注作为主机时需与主机处于同一进程,也可由主机程序自行处理
void Modbus_Task(struct _Modbus *pModbus)
{
  //被劫持(其它设备如WIFI正在配置，未进入正常模式)时直接退出
  if(pModbus->pUsartDev->SendEndInt != Modbus_cbSendInt) 
    return;
  
  if(!(pModbus->Timer)) return;//停止状态
  pModbus->Timer--;
  if(pModbus->Timer) return;//定时值未到
  
  //定时值到了
  UsartDev_RcvStop(pModbus->pUsartDev);  //先停止通讯
  pModbus->Flag |= MODBUS_ERR_OTHER;//置错误标志
  pModbus->Notify(pModbus,0xff);//超时通报
  
  if(pModbus->Flag & MODBUS_HOST){//主机模式时 
    pModbus->eState = Modbus_eState_Idie;//置空闲状态
  }
  else{//从机模式时
    Modbus_RcvStart(pModbus); //重新接收数据    
  }
}

//-------------------------Rtu模式硬件定时器溢出处理函数----------------------------
void Modbus_RtuHwTimerOv(struct _Modbus *pModbus)
{
  //被劫持(其它设备如WIFI正在配置，未进入正常模式)时直接退出
  if(pModbus->pUsartDev->SendEndInt != Modbus_cbSendInt) 
    return;
  
  if(pModbus->eState != Modbus_eState_RdData){//不在接收数据中,状态机异常
    pModbus->eState = Modbus_eState_Idie;//转由函数处理
  }
  else{//接收数据完成
    pModbus->eState = Modbus_eState_RdDataEnd;
    UsartDev_RcvStop(pModbus->pUsartDev); //接收停止      
  }
  Modbus_cbRtuRcvInt(pModbus->pUsartDev);
}

