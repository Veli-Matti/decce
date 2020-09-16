#include <stdbool.h>

typedef int mcu_actuator;
typedef int int32_t;
typedef unsigned int uint32_t;
typedef enum
{
    pwmAccPhaseConcave = 0,
    pwmAccPhaseLinear,
    pwmAccPhaseConvex
} pwmAccPhase;

typedef struct
{
    int targetPos;
    int32_t targetSpeed;
    uint32_t maxAcceleration;
    uint32_t period;
    pwmAccPhase phase;
    int32_t jerk;
    int32_t jerkFreq;
    // Just for prototyping. Take this from pwmInfo
    int32_t currentSpeed;
    float initialSpeed;
    bool setPosition;
    bool debugTakeStep;
    // Just for prototyping v2
    uint32_t breakZone;
    uint32_t breakTime;

} accMoveDirective;

typedef enum
{
    MCU_ERROR_NONE              = 0,
    MCU_ERROR_NOT_SUPPORTED     = 1,
    MCU_ERROR_NOT_FOUND         = 2,
    MCU_ERROR_TIMEOUT           = 3,
    MCU_ERROR_OVERFLOW          = 4,
    MCU_ERROR_UNDERFLOW         = 5,
    MCU_ERROR_NO_MEMORY         = 6,
    MCU_ERROR_BUSY              = 7,
    MCU_ERROR_NOT_INITIALIZED   = 8,
    MCU_ERROR_PARAMETER         = 9,
    MCU_ERROR_NO_ACCESS         = 10,
    MCU_ERROR_CANCEL            = 11,
    MCU_ERROR_NOT_ENABLED       = 12,
    MCU_ERROR_NOT_ALLOWED       = 13,
    MCU_ERROR_UDP_PEER_LOST     = 14,
    MCU_ERROR_GENERAL           = 99
} mcu_error;