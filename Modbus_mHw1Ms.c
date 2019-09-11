/*******************************************************************************

        ���Modbusʱ,����Ӳ����ʱ��(RTUģʽ)����1Ms��ʱ�����е�ʵ��

*******************************************************************************/


#include "Modbus_mHw1Ms.h"
#include <string.h>

//-----------------------------��ʼ������-----------------------------
void Modbus_mHw1Ms_Init(struct _Modbus_mHw1Ms *pHw1Ms,
                        struct _Modbus *pModbus)
{
  pHw1Ms->pModbus = pModbus;  
  pHw1Ms->Ms = 0;
  Modbus_mHw1Ms_UpdateOv(pHw1Ms, 9600);//������Ĭ��9600
}

//-------------------------�ɲ����ʸ������ֵ����-----------------------------
void Modbus_mHw1Ms_UpdateOv(struct _Modbus_mHw1Ms *pHw1Ms,
                            unsigned long Buad)//������
{
  //��ʽΪ: �ַ��� * (1000ms / (������ / ����λ��))
  //����Modbus����3.5���ַ�(��5���ַ�Ϊ׼)���,,��ÿ���ַ�10bit����
  pHw1Ms->Ov = (unsigned long)(5 * (1000 * 10)) / Buad;
  if(pHw1Ms->Ov < 2) pHw1Ms->Ov = 2;
}

//-------------------------------������-------------------------------
//����1ms��<1ms���̻��ж���
void Modbus_mHw1Ms_Task(struct _Modbus_mHw1Ms *pHw1Ms)
{
  if(!pHw1Ms->Ms) return;
  pHw1Ms->Ms--;
  if(pHw1Ms->Ms) return;
  //��ʱ����
  Modbus_RtuHwTimerOv(pHw1Ms->pModbus);
}

/*******************************************************************************
                        MODBUS Hw��ػص�����ʵ��
*******************************************************************************/

//-----------------------Ӳ����ʱ����������λ����-----------------------------
//������RTUģʽʱ�������ⲿʵ��3.5���ַ���λ���ܡ�
//��Usartֱ��֧�֣��˺����붨��Ϊ��
void Modbus_cbHwTimerReset(struct _Modbus *pModbus)
{
  struct _Modbus_mHw1Ms *pHw1Ms = Modbus_mHw1Ms_cbpGet(pModbus);
  if(pHw1Ms != NULL) pHw1Ms->Ms = pHw1Ms->Ov;
}

//------------------------------Ӳ����ʱ��ֹͣ����-----------------------------
//������RTUģʽʱ�������ⲿʵ��3.5���ַ���λ���ܡ�
//��Usartֱ��֧�֣��˺����붨��Ϊ��
void Modbus_cbHwTimerStop(struct _Modbus *pModbus)
{
  struct _Modbus_mHw1Ms *pHw1Ms = Modbus_mHw1Ms_cbpGet(pModbus);
  if(pHw1Ms != NULL) pHw1Ms->Ms = 0;
}

