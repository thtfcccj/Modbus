/******************************************************************
//						网络相关的数据处理函数	
//	
****************************************************************/
#ifndef __NET_DATA_H
#define __NET_DATA_H

//--------------------------得到LRC（纵向冗长检测）函数------------------
//返回得到的LRC数据
unsigned char GetLRC(unsigned char *pBuf,  //数据帧
                     unsigned short Len);    //数据帧长度

//-------------------------------ASCII字符转换为数据函数------------------
//返回转换后的数据长度
unsigned short AscToData(unsigned char *pDest, //转移后的目的地
                         unsigned char *pSorce,//数据源
                         unsigned short Len);   //数据长度

//-------------------------------数据转换为ASCII字符函数---------------------
//返回转换后的数据长度
unsigned short DataToASC(unsigned char *pDest, //转移后的目的地
                         unsigned char *pSorce,//数据源
                         unsigned short Len);   //数据长度


//-----------------------得到CRC16（循环冗长检测）函数-------------------
//返回得到的CRC16数据
unsigned short GetCRC16(unsigned char *pBuf,  //数据帧
                        unsigned short Len);    //数据帧长度

#endif

