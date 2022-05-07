#ifndef __SKIDER_PACKAGE_H__
#define __SKIDER_PACKAGE_H__

#include "struct_typedef.h"
#include "main.h"


#define SKIDER_CONTROLMODE_SCM  0x01
#define SKIDER_GIMBAL_ENABLE    0x01
#define SKIDER_SHOOTER_ENABLE   0x01

// TEST COMMAND : 55 01 18 AA 66 01 01 3C 00 64 00 3C 00 00 00 14 00 64 00 14 00 64 00 14 00 64 00 88

typedef __PACKED_STRUCT{
    uint8_t     _SOF_;
    uint8_t     Mode;
    uint8_t     DataFrameSize;
    uint8_t     _EOF_;
} SkiderControlFrame_Header_t;

typedef __PACKED_STRUCT{
    uint8_t     _SOF_;
    uint8_t     GimbalState;
    uint8_t     ShooterState;
    uint16_t    YawKp;
    int16_t     YawCommand;
    uint16_t    PitchKp;
    int16_t     PitchCommand;
    uint16_t    RotorKp;
    int16_t     RotorCommand;
    uint16_t    AmmolKp;
    int16_t     AmmolCommand;
    uint16_t    AmmorKp;
    int16_t     AmmorCommand;
    uint8_t     _EOF_;
} SkiderControlFrame_SCM_t;

typedef __PACKED_STRUCT
{
    uint8_t     _SOF_;
    uint8_t     Gyro[6];
    uint8_t     Accl[6];
    uint8_t     Sbus[18];
    uint8_t     _EOF_;
} SkiderFeedbackFrame_t;







#endif


