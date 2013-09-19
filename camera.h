#include <stdint.h>

extern void SendResetCmd();
extern void SendTakePhotoCmd();
extern void SendReadDataCmd();
extern void takePicture();
extern int  getPicture(uint8_t *img);
extern void initCamera();
extern void BaudRate();
extern void StopTakePhotoCmd();
extern void setPicSize();
