/******************************************************************
//						������ص����ݴ�����	
//	
****************************************************************/
#ifndef __NET_DATA_H
#define __NET_DATA_H

//--------------------------�õ�LRC�������߳���⣩����------------------
//���صõ���LRC����
unsigned char GetLRC(unsigned char *pBuf,  //����֡
                     unsigned short Len);    //����֡����

//-------------------------------ASCII�ַ�ת��Ϊ���ݺ���------------------
//����ת��������ݳ���
unsigned short AscToData(unsigned char *pDest, //ת�ƺ��Ŀ�ĵ�
                         unsigned char *pSorce,//����Դ
                         unsigned short Len);   //���ݳ���

//-------------------------------����ת��ΪASCII�ַ�����---------------------
//����ת��������ݳ���
unsigned short DataToASC(unsigned char *pDest, //ת�ƺ��Ŀ�ĵ�
                         unsigned char *pSorce,//����Դ
                         unsigned short Len);   //���ݳ���


//-----------------------�õ�CRC16��ѭ���߳���⣩����-------------------
//���صõ���CRC16����
unsigned short GetCRC16(unsigned char *pBuf,  //����֡
                        unsigned short Len);    //����֡����

#endif

