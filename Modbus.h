/******************************************************************
//						Modbus�ӿ�ͷ�ļ�	
//	
//��ΪModbus�ӻ�ʱ��RTUģʽʹ��һ��������ģʽ�� ASC���õ����շ�ģʽ
//������ӻ�ģʽ������
//��ģ����ԭMosbus�����ϣ��ڱ�׼����UsartDev����д�����
//ע:��ģ�鲻����Usart��ͨѶ��������
****************************************************************/
#ifndef __MODBUS_H
#define __MODBUS_H

/************************************************************************
                             �������
************************************************************************/

//����USARTӲ���Ƿ�֧�����ݳ�ʱ���,��֧�֣�Ӳ���ڳ�ʱ���Զ�����,
//����Modbus�㽫���շ���ÿ�������ֶ������ϲ㶨ʱ����ɡ�
//ע��RTUģʽ,ʹ�ø�����ʱ��ʱ,��Ӧ֧��,�������ݿ��ܻᱻ�Ͽ���
//#define MODBUS_SUPPORT_USART_OV_FINAL   

//�����ڴӻ��ڵȴ����յ���һ����ǰ�ĳ�ʱʱ�䣬
//����ʱ��û���յ����ݣ�MODBUS�����������Է�ֹ�쳣����
//ע:����ģʽû�д˹���
#define MODBUS_SLAVE_WAIT_OV   200


//������ASCģʽʱ���ڲ���ʱ����ʱ����ֵ
#define MODBUS_ASC_OV_FINAL   10
  
//������RTUģʽʱ���ȴ�һ֡���ݵ��ʱ��
#define MODBUS_RTU_OV_FINAL   20



/*********************************************************************
                             ��ؽṹ
***********************************************************************/

//Modbus����״̬����
enum _Modbus_eState
{
	Modbus_eState_Idie			  = 0,	//���ڿ���̬,�ȴ���ʼ����
	Modbus_eState_RdWait		  = 1,	//�ڶ��ȴ�״̬���ȴ���ʼ����
	Modbus_eState_RdReady		  = 2,	//��ʼ�����ѽ���,׼����������״̬
	Modbus_eState_RdData		  = 3,  //���ϱ�����ַ,��������״̬
	Modbus_eState_RdDataEnd	      = 4,  //�ѽ������,��Ϊ����β
	Modbus_eState_RdDataDone	  = 5,	//׼�������������,�������ݽ׶�
	Modbus_eState_WrData		  = 6,	//��������״̬,��Ӧ��������
	Modbus_eState_WrFinal		  = 7	  //���ݷ�����ɵ�������ɽ�����־			
};

//���ṹ
#include "UsartDev.h"
struct _Modbus{
  //�������֣�
  enum _Modbus_eState eState;    //Modbus״̬��
  unsigned char Flag;            //��ر�־,������
  unsigned char Adr;              //��ǰͨѶ��ַ
  unsigned char Timer;             //��ʱ��ֵ
  //�ײ������շ����:
  void *pVoid;                         //�ص�����������Ҫ��ָ��
  struct _UsartDev *pUsartDev;   //�ҽӵĵײ��豸
  unsigned char *pBuf;           //���ݻ�����,���շ���ͨ��
  unsigned short Count;          //���ݻ�������С
  unsigned short Len;            //��Ч���ݳ���
  //�ص�����
  signed char (*Notify)(struct _Modbus *pModbus,
                unsigned char ErrFlag); //��ر�־,��Flag������
};

//���У���ر�־Flag����Ϊ��
#define MODBUS_ASC         0x80   //ASC��־����������RTUģʽ
#define MODBUS_HOST        0x40  //������־���������ڴӻ�ģʽ
#define MODBUS_ROUTER      0x20  //·��ģʽ����ģʽ�κε�ַ����
//����״̬����λ:
#define MODBUS_RUN_MASK    0x1F  //���嶨��Ϊ:
#define MODBUS_ZERO_ADR    0x10  //���ַ��־
#define MODBUS_ERR_MASK    0x0F  //�����־,���嶨��Ϊ:
#define MODBUS_ERR_OTHER   0x08  //���������־
#define MODBUS_ERR_ADR     0x04  //��ַ�����־
#define MODBUS_ERR_DATA    0x02  //�����д����־
#define MODBUS_ERR_STATE   0x01  //״̬�������־

/******************************************************************************
                             ��غ���
******************************************************************************/


//--------------------------��ʼ������------------------------------------
void Modbus_Init(struct _Modbus *pModbus, //δ��ʼ���Ķ���
                 unsigned char Flag,      //��ر�־
                 void *pVoid,                    //�ص�����������Ҫ��ָ��
                 struct _UsartDev *pUsartDev,   //�ҽӵ��ѳ�ʼ���ײ��豸
                 unsigned char *pBuf,           //���ݻ�����
                 unsigned short Count,          //���ݻ�������С
                 //�ص�����,���ܻ����ж������,ErrFlag������
                 signed char (*Notify)(struct _Modbus *pModbus,
                                       unsigned char ErrFlag));
//ErrFlag����Ϊ:
//0: ������������ʼͨ��
//0xff ��ʱͨ��
//0xfe ���յ��׸���ͨ��  
//0xfd ���յ��׸��쳣��ͨ��(ASCģʽʱ��:)                               
//<16: ���չ��̴������ͣ���MODBUS_ERR_OTHER����

//----------------------������������жϻص�����------------------------
//�˺��������ڻָ�MODBUSͨѶ
signed char Modbus_cbSendInt(void *pvUsartDev);

//-------------------------���趨��ַ����------------------------
//void Modbus_SetAdr(struct _Modbus *pModbus,
//                  unsigned char Adr);
#define Modbus_SetAdr(pmodbus,adr) do{(pmodbus)->Adr = adr;}while(0)

//-------------------------�����ú���------------------------
//����ǰӦ��ֹͣ״̬
//void Modbus_ReCfg(struct _Modbus *pModbus,
//                  unsigned char Flag);
#define Modbus_ReCfg(pmodbus,flag) \
  do{(pmodbus)->Flag = (flag) & (MODBUS_ASC | MODBUS_HOST);}while(0)
    
//-----------------------����ASC��RTU����------------------------
//����ǰӦ��ֹͣ״̬
#define Modbus_SetAsc(pmodbus) do{(pmodbus)->Flag |= MODBUS_ASC;}while(0)
#define Modbus_SetRtu(pmodbus) do{(pmodbus)->Flag &= ~MODBUS_ASC;}while(0)

//-------------------------��������������------------------------
//�����Ƿ������ɹ�
signed char  Modbus_RcvStart(struct _Modbus *pModbus);

//----------------------����������������----------------------------
//�����Ƿ������ɹ�
signed char Modbus_SendStart(struct _Modbus *pModbus,
                             unsigned char Adr,        //Ҫ���͵��豸��ַ
                             unsigned char *pData,     //Ҫ���͵�����ָ��
                             unsigned short Len,       //���ݳ���
                             unsigned char IsDisCheck);//�Ƿ��ֹУ��

//---------------------�ж��Ƿ�ɹ�������ɺ���----------------------------
//�����Ƿ������ɹ�
//enum _Modbus_eState Modbus_IsRdDataDone(struct _Modbus *pModbus)
#define Modbus_IsRdDataDone(pModbus) \
    ((pModbus)->eState == Modbus_eState_RdDataDone)

//----------------------ǿ��ֹͣ����------------------------
//ǿ��ֹͣ����Ϊ����״̬
void Modbus_Stop(struct _Modbus *pModbus);

//----------------------�������ݽ��뺯��------------------------
//���������Ƿ����
signed char Modbus_RcvDataDecode(struct _Modbus *pModbus);

//----------------------�õ��������ݺ���------------------------
//���ݽ�����ȷʱ���ܵ��ô˺���
//�������ݸ�ʽΪ����ַ0+������1+����...
unsigned char *Modbus_pGetRcvData(struct _Modbus *pModbus);

//----------------------Modbus�õ��������ݳ��Ⱥ���------------------------
//���ݽ�����ȷʱ���ܵ��ô˺���
//����ַλ�ô�С
//unsigned short Modbus_GetRcvLen(struct _Modbus *pModbus);
#define Modbus_GetRcvLen(pmodbus) ((pmodbus)->Len)

//-----------------Modbus�õ����ݷ��ʹ�����������------------------------
//ʹ���ڽ�������ʱ���ô˺���
//���ػ�����λ�ü���С
unsigned char *Modbus_pGetSendBuf(struct _Modbus *pModbus,
                                  unsigned short *pCount); //���ؿ��û�������С

//-------------------------������----------------------------
//�˴���������������Զ��ָ��ӻ�����
//ע��Ϊ����ʱ������������ͬһ����,Ҳ���������������д���
void Modbus_Task(struct _Modbus *pModbus);

//-------------------------RtuģʽӲ����ʱ�����������----------------------------
//������RTUģʽʱ�������ⲿʵ��3.5���ַ���λ���ܡ�
//��Usartֱ��֧�֣��˺����ɲ�����
void Modbus_RtuHwTimerOv(struct _Modbus *pModbus);

/******************************************************************************
                           �ص�����
******************************************************************************/
//#include "ModbusSlave.h"

//-------------------------�ɵײ�UsartDev�õ�Modbus����-----------------------
//struct _Modbus *Modbus_cbGet(const struct _UsartDev *pUsartDev);
#define Modbus_cbGet(usartDev) ((struct _Modbus *)((usartDev)->pVoid))
//#define Modbus_cbGet(pUsartDev)  ((struct _Modbus *)0)//����ʱ
//#define Modbus_cbGet(pUsartDev)  (&ModbusSlave.Modbus)

//-----------------------Ӳ����ʱ����������λ����-----------------------------
//������RTUģʽʱ�������ⲿʵ��3.5���ַ���λ���ܡ�
//��Usartֱ��֧�֣��˺����붨��Ϊ��
void Modbus_cbHwTimerReset(struct _Modbus *pModbus);
//#define Modbus_cbHwTimerReset(pModbus)  do{}while(0)

//------------------------------Ӳ����ʱ��ֹͣ����-----------------------------
//������RTUģʽʱ�������ⲿʵ��3.5���ַ���λ���ܡ�
//��Usartֱ��֧�֣��˺����붨��Ϊ��
void Modbus_cbHwTimerStop(struct _Modbus *pModbus);
//#define Modbus_cbHwTimerStop(pModbus)  do{}while(0)

//-------------------------ͨ����ʼ�������ݺ���-----------------------------
//������ʵ��RTS��IO����
#ifdef SUPPORT_MUTI_USART
  void Modbus_cbRcvStartNotify(struct _Modbus *pModbus);
#else //ֻ֧��1��ʱֱ����Ioctrlʵ��:
  #include "IoCtrl.h"
  #define Modbus_cbRcvStartNotify(pModbus)  do{ClrRS485RTS();}while(0)
#endif

//-------------------------ͨ����ʼ�������ݺ���-----------------------------
//������ʵ��RTS��IO����
#ifdef SUPPORT_MUTI_USART
  void Modbus_cbSendStartNotify(struct _Modbus *pModbus);
#else //ֻ֧��1��ʱֱ����Ioctrlʵ��:
  #define Modbus_cbSendStartNotify(pModbus)  do{SetRS485RTS();}while(0)
#endif
  
#endif //#ifndef __MODBUS_H

