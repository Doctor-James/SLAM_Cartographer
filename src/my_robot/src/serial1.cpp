#include <ros/ros.h>
#include <serial/serial.h>
#include "serial.h"
#include <iostream>
//串口类
serial::Serial ser;
#define PC_RECVBUF_SIZE 13
 
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
//发送左右轮速控制速度共用体
union sendData
{
	short d;
	unsigned char data[2];
}x,y,w;

//接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union receiveData
{
	short d;
	unsigned char data[2];
}X_VelNow,Y_VelNow,W_VelNow,angleNow;

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
int serialInit1()
{
    //打开串口
  try {
    ser.setPort("/dev/ttyUSB1");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }
 
  if (ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }  
}
void Serialclose(){

    //关闭串口
    ser.close();
}
/********************************************************
函数功能：将对机器人的左右轮子控制速度，打包发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void writeSpeed(short V_x, short V_y,short V_w,unsigned char ctrlFlag)
{
    unsigned char buf[13] = {0};//
    int i, length = 0;

    x.d  = V_x;//mm/s
    y.d = V_y;
    w.d = V_w;
    
    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 7;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = x.data[i];  //buf[3] buf[4]
        buf[i + 5] = y.data[i]; //buf[5] buf[6]
        buf[i + 7] = w.data[i]; //buf[7] buf[8]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[9]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[10]
    buf[3 + length + 1] = ender[0];     //buf[11]
    buf[3 + length + 2] = ender[1];     //buf[12]

    // 通过串口下发数据
    ser.write(buf, 13);
}
/********************************************************
函数功能：从下位机读取数据
入口参数：机器人左轮轮速、右轮轮速、角度，预留控制位
出口参数：bool
********************************************************/
unsigned char ReceiveBuffer[5000]={0};
unsigned char SaveBuffer[26];//接受双缓存区

bool readSpeed(double &V_x_Actual,double &V_y_Actual,double &V_w_Actual,double &Angle)
{
    char i, length = 0;
    unsigned char checkSum;
    
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    int n = ser.available();
    if (n) {
        printf("n=%d\n",n);
        ser.read(ReceiveBuffer, n);

        memcpy(&SaveBuffer[PC_RECVBUF_SIZE],&ReceiveBuffer[n-PC_RECVBUF_SIZE],PC_RECVBUF_SIZE);		//把ReceiveBuffer[0]地址拷贝到SaveBuffer[13], 依次拷贝13个, 把这一次接收的存到数组后方
    //接收到数据后处理
    //=========================================================        
        short k=0;
        short PackPoint;
        memcpy(&SaveBuffer[PC_RECVBUF_SIZE],&ReceiveBuffer[0],PC_RECVBUF_SIZE);		//把ReceiveBuffer[0]地址拷贝到SaveBuffer[13], 依次拷贝13个, 把这一次接收的存到数组后方
        for(PackPoint=0;PackPoint<PC_RECVBUF_SIZE;PackPoint++)		//先处理前半段数据(在上一周期已接收完成)
        {
            if(SaveBuffer[PackPoint]==header[0] && SaveBuffer[PackPoint + 1]== header[1]) //包头检测
            {	
                short dataLength  = SaveBuffer[PackPoint + 2]    ;
                unsigned char checkSum = getCrc8(&SaveBuffer[PackPoint], 3 + dataLength);
                    // 检查信息校验值
                if (checkSum == SaveBuffer[PackPoint +3 + dataLength]) //SaveBuffer[PackPoint开始的校验位]
                {
                    //说明数据核对成功，开始提取数据
                    for(k = 0; k < 2; k++)
                        {
                            X_VelNow.data[k]  = SaveBuffer[PackPoint + k + 3]; //SaveBuffer[3]  SaveBuffer[4]
                            Y_VelNow.data[k] = SaveBuffer[PackPoint + k + 5]; //SaveBuffer[5]  SaveBuffer[6]
                            W_VelNow.data[k]  = SaveBuffer[PackPoint + k + 7]; //SaveBuffer[7]  SaveBuffer[8]
                            angleNow.data[k]  = SaveBuffer[PackPoint + k + 9]; //SaveBuffer[9]  SaveBuffer[10]
                        }				
                        
                        //速度赋值操作
                        V_x_Actual  =(int)(X_VelNow.d);
                        V_y_Actual =(int)(Y_VelNow.d);
                        V_w_Actual =(int)W_VelNow.d;
                        Angle   = angleNow.d;
        
                    
                    
                }
            }	
        memcpy(&SaveBuffer[0],&SaveBuffer[PC_RECVBUF_SIZE],PC_RECVBUF_SIZE);		//把SaveBuffer[13]地址拷贝到SaveBuffer[0], 依次拷贝13个，把之前存到后面的数据提到前面，准备处理
        }
    }
    return true;
}




/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
