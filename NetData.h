/*******************************************************************************

						        网络相关的数据编解码及校验函数集		

*******************************************************************************/
#ifndef __NET_DATA_H
#define __NET_DATA_H

/*******************************************************************************
						               数据编码相关
下述函数输入流可等于输出流
*******************************************************************************/

//-------------------------------ASCII字符转换为数据函数----------------------
//如：字符串"6d4b"转换后数据流为：0x6d,0x4b,Len为源数据长度,
//返回转换后的数据长度
unsigned short AscToData(unsigned char *pDest, //转移后的目的地
                          const unsigned char *pSorce,//数据源
                          unsigned short Len);   //数据长度

//-----------------------ASCII字符转换为数据函数2-------------------------------
//此为AscToData()变种，返回数据流结束位置
unsigned char *pAscToData(unsigned char *pDist, 
                           const char *pSorce,
                           unsigned short Len);

//-------------------------------数据转换为ASCII字符函数---------------------
//如：数据流0x6d,0x4b转换为ASC为："6d4b",Len为源数据长度
//返回转换后的数据长度
unsigned short DataToAsc(unsigned char *pDest, //转移后的目的地
                          const unsigned char *pSorce,//数据源
                          unsigned short Len);   //数据长度

//--------------------数据转换为ASCII字符函数2--------------------------------
//此为DataToAsc()变种，返回字符串结束位置
char *pDataToAsc(char *pDist, 
                 const unsigned char *pSorce, 
                 unsigned short Len);

//----------------------------ASC数字转换为Bcd数字函数--------------------------
//从最高位开始，若长度若为奇数，则首位Bcd码填充0,
//如：字符串"130"转换后数据流为：0x01,0x30
//调用此函数需确保长度范围内为asc数字, 否则将不是BCD值，返回Bcd结束位置
unsigned char *pAscNumToBcd(const char *pStr, 
                            unsigned char Len,
                            unsigned char *pBcd);

//----------------------------Bcd数字转换为ASC数字函数--------------------------
//如：0x01,0x30转换为字符串后"0130"
//调用此函数需确保长度范围内为BCD码, 否则字符串为异常值，返回字符结束位置
char *pBcdNumToAsc(const unsigned char *pBcd,
                   unsigned char Len,                
                   char *pStr);

/*******************************************************************************
						                      校验相关		
*******************************************************************************/

//--------------------------得到LRC（纵向冗长检测）函数-------------------------
//可供MODBUS ASC协用， 返回得到的LRC数据
unsigned char GetLRC(const unsigned char *pBuf,  //数据帧
                      unsigned short Len);         //数据帧长度

//-----------------------得到CRC16（循环冗长检测）函数--------------------------
//可供MODBUS RTU协用， 返回得到的CRC16数据
unsigned short GetCRC16(const unsigned char *pBuf,  //数据帧
                         unsigned short Len);         //数据帧长度

//------------------得到CRC8（循环冗长检测）函数--带查找表----------------------
//可供各种CRC8校验方式使用，返回得到的CRC8数据反码值
//使用查表法，因多项式系数不同，可分别带入不同的表(NULL为默认0x131表)
//注：多项式系数主要有：
//CRC-8       x8+x5+x4+1              0x31（0x131）
//CRC-8       x8+x2+x1+1              0x07（0x107）
//CRC-8       x8+x6+x3+x2+1           0x4D（0x14D）
//CRC-8       x8+x6+x4+x3+x2+x1       0x5E（0x15E）
unsigned short GetCRC8_Tbl(const unsigned char *pBuf,  //数据帧
                            unsigned short Len,         //数据帧长度
                            const unsigned char *pTbl);//查找表

//-----------------------得到CRC8-多项式为0x131函数--------------------------
//此函数多项式系数为：x8+x5+x4+1              0x31（0x131）
unsigned short GetCRC8_131(const unsigned char *pBuf,  //数据帧
                            unsigned short Len);         //数据帧长度

#endif

