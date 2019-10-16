/******************************************************************
//
//						Modbus����ʵ��
//
****************************************************************/

#include "Modbus.h"
#include  "NetData.h"  //У������ݴ���
#include  <string.h>

//--------------------------��ʼ������------------------------------------
void Modbus_Init(struct _Modbus *pModbus, //δ��ʼ���Ķ���
                 unsigned char Flag,      //��ر�־
                 void *pVoid,                         //�ص�����������Ҫ��ָ��
                 struct _UsartDev *pUsartDev,   //�ҽӵ��ѳ�ʼ���ײ��豸
                 unsigned char *pBuf,           //���ݻ�����
                 unsigned short Count,          //���ݻ�������С
                 //�ص�����,���ܻ����ж������
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
  Modbus_cbHwTimerStop(pModbus); //��ֹͣ
  pModbus->pUsartDev->SendEndInt = Modbus_cbSendInt;//�ȹ��ط���
}

//----------------------RTUģʽ���������жϻص�����------------------------
//�˺�������ⲿ��Ӳ����ʱ��ʵ��״̬�������ݵ��շ�
static signed char Modbus_cbRtuRcvInt(void *pvUsartDev)
{
  struct _UsartDev *pUsartDev = (struct _UsartDev*)pvUsartDev;
  struct _Modbus *pModbus = Modbus_cbGet(pUsartDev);
  unsigned char Flag;
  
  switch(pModbus->eState){
  //�ڵȴ�״̬ʱ���յ���һ������Ϊ��ַ
  case Modbus_eState_RdWait:{
    #ifdef MODBUS_SUPPORT_USART_OV_FINAL
      //������Ϊ���ٽ���ģʽ(��ʱ���յ�һ��������)
      UsartDev_RcvStart(pModbus->pUsartDev,
                        pModbus->pBuf + 1,
                        (pModbus->Count - 1) | 0x8000,
                        Modbus_cbRtuRcvInt);
      pModbus->eState = Modbus_eState_RdDataEnd;  //Ԥ�ý����������״̬ 
    #else //������ǰ����
      pModbus->eState = Modbus_eState_RdData;    
    #endif
    Modbus_cbHwTimerReset(pModbus);//֪ͨ��ʱ����λ
    pModbus->Notify(pModbus,0xfe); //�׸���ͨ��
    return 0; //��������
  }
  //��������״̬,��MODBUS_SUPPORT_USART_OV_FINAL����
  case Modbus_eState_RdData:
    Modbus_cbHwTimerReset(pModbus);//֪ͨ��ʱ����λ
    return 0; //��������
  //����������״̬������״̬:
  //case Modbus_eState_RdDataEnd:
  default:{ //����״̬
    break;
  }
  }//end   switch(pModbus->eState){
  
  //״̬��������������ɴ���
  Modbus_cbHwTimerStop(pModbus);  //֪ͨ��ʱ��ֹͣ 
  //������
  Flag = 0;//�ٶ���ȷ
  if(pModbus->eState != Modbus_eState_RdDataEnd)//״̬������
    Flag = MODBUS_ERR_STATE;
  else if(pUsartDev->Flag & USART_DEV_RCV_ERR)//���ݴ���
    Flag = MODBUS_ERR_DATA;
  else if(*pModbus->pBuf == 0) //�㲥��ַ
    Flag = MODBUS_ZERO_ADR;
  else if(pModbus->Flag & MODBUS_ROUTER){} //·��ģʽ������������
  else if(*pModbus->pBuf != pModbus->Adr)//����ϣ���ĵ�ַ 
    Flag = MODBUS_ERR_ADR;
  //������ɴ���:
  #ifdef MODBUS_SUPPORT_USART_OV_FINAL
    pModbus->Len = pUsartDev->RcvLen + 1;//���յ��ĸ���,����������������,����ϵ�ַλ
  #else
    pModbus->Len = pUsartDev->RcvLen;
  #endif
  if(Flag & MODBUS_ERR_MASK)
    pModbus->eState = Modbus_eState_Idie; //����ʱԤ�ÿ���״̬
  else 
    pModbus->eState = Modbus_eState_RdDataDone; //�����������
  pModbus->Flag |= Flag;
  
  //���������������
  return pModbus->Notify(pModbus,Flag); //ͨ��
  
  //return -1;//ֹͣ
}

//----------------------ASCģʽ���������жϻص�����------------------------
//�˺�������ڲ��Ķ�ʱ��ʵ��״̬�������ݵ��շ�
static signed char Modbus_cbAscRcvInt(void *pvUsartDev)
{
  struct _UsartDev *pUsartDev = (struct _UsartDev*)pvUsartDev;
  struct _Modbus *pModbus = Modbus_cbGet(pUsartDev);
  unsigned char *pRcvBuf = pUsartDev->pRcvBuf;
  
  unsigned short CurPos = pUsartDev->RcvLen - 1;
  unsigned char RcvData = *(pRcvBuf + CurPos);//��ȡ���һ������
  unsigned char Flag = 0;    

  pModbus->Timer = MODBUS_ASC_OV_FINAL; //�ڲ���ʱ����λ
  
  //������ȥ�����λ,��������STM32�Ȳ�֧��7����λ
  *(pRcvBuf + CurPos) &= ~0x80; 
  RcvData &= ~0x80; 
  
  //���յ�ǰ���ַ�������δ�������ʱ,ǿ�Ƹ�λ,׼����������  
  if(RcvData == ':'){
      pModbus->eState = Modbus_eState_RdReady;
      *pRcvBuf = ':';
      pUsartDev->RcvLen = 1;
      pModbus->Notify(pModbus,0xfe); //�׸�����ȷͨ��
      return 0;//��������
  }
  //״̬������:

  switch(pModbus->eState){
  //�ȴ�����״̬�յ���":"
  case Modbus_eState_RdWait: 
    pUsartDev->RcvLen = 0;//����,���¿�ʼ����
    //pModbus->Notify(pModbus,0xfd); //�׸���ͨ��,�׸����쳣
    return 0;//��������    
   //���յ���ʼλ,׼�����յ�ַ   
  case Modbus_eState_RdReady:{
    if(CurPos == 2){//�ж�һ����λ
      unsigned char CurAdr = (*(pRcvBuf + 2) - '0') | //��ַ��λASC
                             (( *(pRcvBuf + 1) - '0') << 4); //��ַ��λASC
      if(CurAdr == 0) //�㲥��ַ
        Flag = MODBUS_ZERO_ADR;
      else if(pModbus->Flag & MODBUS_ROUTER){} //·��ģʽ������������
      else if(CurAdr != pModbus->Adr)//����ϣ���ĵ�ַ 
        Flag = MODBUS_ERR_ADR;
      //�޴���ʱת��״̬������������״̬
      if(!(Flag & MODBUS_ERR_MASK)){
        pModbus->eState = Modbus_eState_RdData;
      }
    }
    else if(CurPos != 1)
      Flag = MODBUS_ERR_OTHER;//��������
    break;
  }
  //���ݽ���״̬
  case Modbus_eState_RdData:
    if(CurPos >= pModbus->Count)
      Flag = MODBUS_ERR_DATA;//�������
    //��ַ����ʱ���յ��س��ַ�,��������β
    else if(RcvData == 0x0d){
      if(CurPos >= 6)
        pModbus->eState = Modbus_eState_RdDataEnd;
      else 
        Flag = MODBUS_ERR_OTHER;//��������,���������ݲ���,���¿�ʼ
    }
    break;
  //���յ������ַ�
  case Modbus_eState_RdDataEnd:
    if(RcvData != 0x0a)
      Flag = MODBUS_ERR_OTHER;//��������,�����ַ�����
    else{//������ȷ
      //��ȡ�������,������������
      pModbus->eState = Modbus_eState_RdDataDone;
      pModbus->Len = CurPos + 1;//���յ��ĸ���
      pModbus->Timer = 0;  //��ʱ��ֹͣ
    }
    break;
  default: //״̬������
    Flag = MODBUS_ERR_STATE;
    break;
  }
  
  if(Flag & MODBUS_ERR_MASK)//����ʱǿ�ƽ���
    pModbus->eState = Modbus_eState_Idie; //����ʱԤ�ÿ���״̬
  else if(pModbus->eState != Modbus_eState_RdDataDone)
    return 0;//û������ʱ��������
    
  //���������������
  return pModbus->Notify(pModbus,Flag); //ͨ��
  
  //return -1;//ֹͣ
}

//------------------------��������������------------------------
//�����Ƿ������ɹ�
signed char  Modbus_RcvStart(struct _Modbus *pModbus)
{
  Modbus_Stop(pModbus);//��ǿ��ֹͣ
  Modbus_cbRcvStartNotify(pModbus);//ͨ����ʼ��������
  pModbus->Flag &= ~MODBUS_RUN_MASK;//��־����
  
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
    Modbus_cbHwTimerStop(pModbus);  //֪ͨ��ʱ����ֹͣ�ȴ���һ����
  }
  if(!(pModbus->Flag & MODBUS_HOST))//�ӻ�ģʽʱ�ö�ʱ����ֹ�쳣����
    pModbus->Timer = 200;
  else pModbus->Timer = 0; //����ģʽ�����ⲿʵ�ֵȴ���ʱ
  
  pModbus->eState = Modbus_eState_RdWait;   //��Ϊ�ȴ�����״̬
  return 0;
}

//----------------------ǿ��ֹͣ����------------------------
//ǿ��ֹͣ����Ϊ����״̬
void Modbus_Stop(struct _Modbus *pModbus)
{
  //���ٳ�(�����豸��WIFI�������ã�δ��������ģʽ)ʱֱ���˳�
  if(pModbus->pUsartDev->SendEndInt == Modbus_cbSendInt){
    UsartDev_RcvStop(pModbus->pUsartDev);
    Modbus_cbRcvStartNotify(pModbus);//ǿ��Ԥ�ý�������״̬
    Modbus_cbHwTimerStop(pModbus);
    UsartDev_SendStop(pModbus->pUsartDev);
  }
  pModbus->eState = Modbus_eState_Idie;
}

//----------------------������������жϻص�����------------------------
//�˺����ڷ���������ɺ����
static signed char Modbus_cbSendInt(void *pvUsartDev)
{
  struct _UsartDev *pUsartDev = (struct _UsartDev*)pvUsartDev;
  struct _Modbus *pModbus = Modbus_cbGet(pUsartDev);
  pModbus->eState = Modbus_eState_WrFinal;//��Ϊд�������״̬
  
  //�ٶ�Ϊȫ��ȫ����ȷ
  pModbus->Notify(pModbus,0);//���������,����ͨ��
  Modbus_cbRcvStartNotify(pModbus);//ǿ��Ԥ�ý�������״̬
  
  Modbus_RcvStart(pModbus); //Ԥ��Ϊ����ģʽ��������
  return 1;
}

//----------------------����������������----------------------------
//�����Ƿ������ɹ�
signed char Modbus_SendStart(struct _Modbus *pModbus,
                             unsigned char Adr,        //Ҫ���͵��豸��ַ
                             unsigned char *pData,     //Ҫ���͵�����ָ��
                             unsigned short Len,       //���ݳ���
                             unsigned char IsDisCheck) //�Ƿ��ֹУ��
{
  unsigned char *pBuf;
  unsigned short CRC16;
  
  //���ٳ�(�����豸��WIFI�������ã�δ��������ģʽ)ʱֱ���˳�
  if(pModbus->pUsartDev->SendEndInt != Modbus_cbSendInt) 
    return -1;

  Modbus_Stop(pModbus);//��ǿ��ֹͣ
  pBuf = pModbus->pBuf;

  if(pModbus->Flag & MODBUS_HOST) //����ģʽʱ���µ�ַ
    pModbus->Adr = Adr;
  if(pModbus->Flag & MODBUS_ASC){//ASC��ʽʱ
    if((Len << 1) > (pModbus->Count - 7)) return -1;    //�������ݹ���
    //������ԭ��������(:�͵�ַ��ռһλ)������copy����λ��
    if(pData != (pBuf + 2)) memcpy(pBuf + 2,pData,Len);
    *pBuf++ = ':'; 
    *pBuf = Adr;
    Len += 1; //������ַ��
    *(pBuf + Len) = GetLRC(pBuf,Len);   //�õ�LRC����У����,����ַ
    Len += 1;//����LRCУ������
    Len = DataToAsc(pBuf,pBuf,Len); //ת��ΪASCII,����ʼ�ַ����ַλ��
    //׼��У���������ɿ�ʼ������־
    pBuf += Len;      //�������λ����
    *pBuf++ = 0x0d;
    *pBuf++ = 0x0a;//++Ϊ�������
    pModbus->Len = pBuf - pModbus->pBuf;
  }
  else{ //RSC����ģʽ
    if((Len + 3) > pModbus->Count) return -1;    //�������ݹ���
    //������ԭ��������(��ַռһλ)������copy����λ��
    if(pData != (pBuf + 1)) memcpy(pBuf + 1,pData,Len);
    *pBuf = Adr;
    Len += 1; //��ַλ
    if(!IsDisCheck){//û�н�ֹ����У����ʱ
      CRC16 = GetCRC16(pBuf,Len); //�õ�CRC16У����
      pBuf += Len;
      *pBuf++ = (unsigned char)(CRC16 >> 8);//CRC��λ
      *pBuf = (unsigned char)(CRC16 & 0xff);//CRC��λ
      pModbus->Len = Len + 2; //CRC16ռλ
    }
    else pModbus->Len = Len;
  }
  //��������
  Modbus_cbSendStartNotify(pModbus);//ͨ����ʼ��������
  UsartDev_SendStart(pModbus->pUsartDev,
                     pModbus->pBuf,
                     pModbus->Len | 0x8000,//�����ڿ���ģʽ
                     Modbus_cbSendInt);
  
  pModbus->eState = Modbus_eState_WrData;//��Ϊд����״̬
  return 0;
}


//----------------------�������ݽ��뺯��------------------------
//���������Ƿ����
signed char Modbus_RcvDataDecode(struct _Modbus *pModbus)
{
  unsigned char *pBuf = pModbus->pBuf;
  unsigned short Len = pModbus->Len;
  unsigned short Crc;

  if(pModbus->Flag & MODBUS_ASC){//ASC��ʽʱ
    if(Len < 4) return -2;//�쳣����
    pBuf++;
    Len = AscToData(pBuf,pBuf,Len - 3); //ת��ΪASCIIת��Ϊ����,������ʼ�������־
    Len--;
    if(*(pBuf + Len) != GetLRC(pBuf,Len)) return -1;   //LRC����У�������
  }
  //RSC����ģʽ
  else{
    if(Len < 2) return -2;//�쳣����
    Len-= 2;//CRC����λ
    Crc = (unsigned short)(*(pBuf + Len) << 8) | 
      (unsigned short)(*(pBuf + Len + 1));
    if(Crc != GetCRC16(pBuf,Len)) return -1;   //CRC����У�������
  }
  pModbus->Len = Len;
  return 0;
}

//----------------------�õ��������ݺ���------------------------
//���ݽ�����ȷʱ���ܵ��ô˺���
//�������ݸ�ʽΪ����ַ0+������1+����...
unsigned char *Modbus_pGetRcvData(struct _Modbus *pModbus)
{
  if(pModbus->Flag & MODBUS_ASC)//ASC��ʽʱ
    return pModbus->pBuf + 1;
  //RTUģʽʱ
  return pModbus->pBuf;
}

//-----------------Modbus�õ����ݷ��ʹ�����������------------------------
//ʹ���ڽ�������ʱ���ô˺���
//���ػ�����λ�ü���С
unsigned char *Modbus_pGetSendBuf(struct _Modbus *pModbus,
                                  unsigned short *pCount) //���ؿ��û�������С
{
  if(pModbus->Flag & MODBUS_ASC){//ASC��ʽʱ,���ݼ��벢��ȥ��ؿ���λ��
    *pCount = ((pModbus->Count) >> 1) - 8;//:,��ַ��LRCH,LRCL
    return pModbus->pBuf + 2;     //����ǰ,":'�͵�ַռλ
  }
  //RTUģʽʱ
  *pCount = pModbus->Count - 2;//��CRC16ռλ
  return pModbus->pBuf + 1;    //��ַռλ  
}

//-------------------------����������----------------------------
//�˴���������������Զ��ָ��ӻ�����
//ע��Ϊ����ʱ������������ͬһ����,Ҳ���������������д���
void Modbus_Task(struct _Modbus *pModbus)
{
  //���ٳ�(�����豸��WIFI�������ã�δ��������ģʽ)ʱֱ���˳�
  if(pModbus->pUsartDev->SendEndInt != Modbus_cbSendInt) 
    return;
  
  if(!(pModbus->Timer)) return;//ֹͣ״̬
  pModbus->Timer--;
  if(pModbus->Timer) return;//��ʱֵδ��
  
  //��ʱֵ����
  UsartDev_RcvStop(pModbus->pUsartDev);  //��ֹͣͨѶ
  pModbus->Flag |= MODBUS_ERR_OTHER;//�ô����־
  pModbus->Notify(pModbus,0xff);//��ʱͨ��
  
  if(pModbus->Flag & MODBUS_HOST){//����ģʽʱ 
    pModbus->eState = Modbus_eState_Idie;//�ÿ���״̬
  }
  else{//�ӻ�ģʽʱ
    Modbus_RcvStart(pModbus); //���½�������    
  }
}

//-------------------------RtuģʽӲ����ʱ�����������----------------------------
void Modbus_RtuHwTimerOv(struct _Modbus *pModbus)
{
  //���ٳ�(�����豸��WIFI�������ã�δ��������ģʽ)ʱֱ���˳�
  if(pModbus->pUsartDev->SendEndInt != Modbus_cbSendInt) 
    return;
  
  if(pModbus->eState != Modbus_eState_RdData){//���ڽ���������,״̬���쳣
    pModbus->eState = Modbus_eState_Idie;//ת�ɺ�������
  }
  else{//�����������
    pModbus->eState = Modbus_eState_RdDataEnd;
    UsartDev_RcvStop(pModbus->pUsartDev); //����ֹͣ      
  }
  Modbus_cbRtuRcvInt(pModbus->pUsartDev);
}

