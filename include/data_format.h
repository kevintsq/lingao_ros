#ifndef LINGAO_DATA_FORMAT_H
#define LINGAO_DATA_FORMAT_H

#define Max_Data_Lengh 1024

#include <string.h>

#define HEAD_FLAG1 0xFE
#define HEAD_FLAG2 0xEF


class FormatTools
{
public: 
    //同地址大小端交換
    static void EndianSwap(void *pData, char typeSize)
    {
        if(NULL == pData){
		    return;
	    }

        char cnt = typeSize /2;
        char *end = (char *)pData + typeSize - 1;
        char *start = (char *)pData;

        unsigned char tmp;
        for (int i = 0; i < cnt; i++)
        {
            tmp       = *(start + i);
            *(start + i) = *(end -i);
            *(end -i)    = tmp;
        }
    }

    static void EndianSwap(void *dest, const void *src,  char typeSize, int arraySize)
    {
        if(NULL == dest || NULL == src){
		    return;
	    }

        for(int i=0; i<arraySize/typeSize; i++)
        {
            EndianSwap(dest, src, typeSize);
            dest = (char *)dest + typeSize;
            src = (char *)src + typeSize;
        }
    }

    static void EndianSwap(void *dest, const void *src,  char typeSize)
    {
        char *psrc;
        char *pdst;
    
        if(NULL == dest || NULL == src){
		    return;
	    }

        pdst = (char *)dest;
        psrc = (char *)src + typeSize -1;

        for (int i = 0; i < typeSize; i++)
        {
            *(pdst +i) = *(psrc -i);
        }
    }

};

enum Message_Id_Enum
{
    MSG_ID_NULL = 0,            //无
    MSG_ID_SET_VELOCITY = 1,    //设置线速度、角速度
    MSG_ID_GET_VELOCITY = 2,    //获取线速度、角速度
    MSG_ID_GET_VOLTAGE  = 3,    //获取电压，电量数据
    MSG_ID_GET_IMU = 4,         //获取IMU数据
    
};


#pragma pack(1)  //指定字节对齐
//线速度结构体
struct Data_Format_Liner
{
    float v_liner_x;    //车体线速度x, 单位m/s
    float v_liner_y;
    float v_angular_z;

    Data_Format_Liner(const double& vx=.0, const double& vy=.0, const double& wz=.0)
        : v_liner_x(vx), v_liner_y(vy), v_angular_z(wz){}

    void set(const double& vx=.0, const double& vy=.0, const double& wz=.0)
    {
        v_liner_x = vx; v_liner_y = vy; v_angular_z = wz;
    }
    void EndianSwapSet(const void *src)
    {
        FormatTools::EndianSwap(this, src, sizeof(v_liner_x), sizeof(Data_Format_Liner));
    }
};

//IMU数据结构体
struct Data_Format_IMU
{
    float imu_accx;    //IMU 加速度
    float imu_accy;
    float imu_accz;

    float imu_angx;
    float imu_angy;
    float imu_angz;

    // float imu_magx;
    // float imu_magy;
    // float imu_magz;

    void EndianSwapSet(const void *src)
    {
        FormatTools::EndianSwap(this, src, sizeof(imu_accx), sizeof(Data_Format_IMU));
    }
};

//电池数据结构体
struct Data_Format_BAT
{
    int16_t bat_voltage;    //电压 mv
    unsigned char bat_percentage;  //电量百分比

    void EndianSwapSet(const void *src)
    {
        FormatTools::EndianSwap(this, src, sizeof(bat_voltage), sizeof(Data_Format_BAT));
        bat_percentage = *((char *)src + sizeof(bat_voltage));
    }
};
#pragma pack()  //取消指定对齐，恢复缺省对齐

struct MessageHead_st{
    unsigned char flag1;
    unsigned char flag2;
    unsigned char data_length;
    unsigned char msg_id;

    MessageHead_st(){
        flag1 = HEAD_FLAG1;
        flag2 = HEAD_FLAG2;
        msg_id = 0;
        data_length = 0;
    }
};

struct MessageFormat_st{
    MessageHead_st head;
    unsigned char data[Max_Data_Lengh];
    unsigned char check;


    MessageFormat_st(unsigned char _msg_id, unsigned char* _data = 0, unsigned char _data_Lengh = 0)
    {
        head.msg_id = _msg_id;
        head.data_length = _data_Lengh + 1;

        if(_data!=0 && _data_Lengh !=0)
        {
            memcpy(data, _data, _data_Lengh);
        }

        unsigned char* _send_buff = (unsigned char*)this;
        unsigned char _check = 0;

        for (int i=0; i<(4 + _data_Lengh); i++) {
            _check += _send_buff[i];
        }

        _send_buff[4 + _data_Lengh] = _check;

    }

    MessageFormat_st(MessageHead_st _head)
    {
        head = _head;
    }

};


#endif // LINGAO_DATA_FORMAT_H
