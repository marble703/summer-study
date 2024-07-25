#pragma once

typedef struct {
    unsigned short Start = 0x0D00; //起始位     0~1
    unsigned short MessageType;    //消息类型   2~3
    unsigned int DataID;           //数据ID     4~7
    unsigned int DataTotalLength;   //数据总长度 8~11
    unsigned int Offset;           //偏移量     12~15
    unsigned int DataLength;        //数据长度   16~19
    // 数据
    unsigned char Data[10218];     //数据       20~10237
    //数据尾部
    unsigned short End = 0x0721;    //数据尾部   10238~10239
} MessageBuffer;

enum MessageType {
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

// MessageType 为 IMAGE_MSG 时的 Data 数据结构
typedef struct{
    unsigned int Length;
    unsigned int Width;
    unsigned int Type;
    unsigned char ImageData[10218 - 12];
} ImageData;

// MessageType 为 CAMERA_INFO
typedef struct{
    double CameraMaterix[9];
    double DistortionCoefficients[5];
} CameraInfoData;

// MessageType 为 Transform
typedef struct{
    double Translation[3];
    double Roation[4];
} TransformData;

// MessageType 为 TRANSFORM_REQUEST
typedef struct{
    char From[10218 / 2];
    char To[10218 / 2];
}TransformRequestData;

/**
 * @brief 解码数据，存储在指定结构体
 * @param buffer 数据缓存区
 * @param message 数据结构体
 */
void classifyMessage(unsigned char* buffer, MessageBuffer& message) {
    memcpy(&message.Start, buffer, sizeof(message.Start));
    memcpy(&message.MessageType, buffer + 2, sizeof(message.MessageType));
    memcpy(&message.DataID, buffer + 4, sizeof(message.DataID));
    memcpy(&message.DataTotalLength, buffer + 8, sizeof(message.DataTotalLength));
    memcpy(&message.Offset, buffer + 12, sizeof(message.Offset));
    memcpy(&message.DataLength, buffer + 16, sizeof(message.DataLength));
    memcpy(&message.Data, buffer + 20, message.DataLength);
    memcpy(&message.End, buffer + 20 + message.DataLength, sizeof(message.End));
}