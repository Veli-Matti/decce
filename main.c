#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "main.h"

int addInIncrements();
static void doAdjust(int maxAcc, int jerkFreq, int targetSpeed);

int addInIncrements() {
    time_t seconds;
    for (int i = 0; i < 10; i++) {

        seconds = time(NULL);
        printf("%d, secs = %d\n", i, seconds);
        fflush(stdout);
        usleep(1000000);
    }
}



int main()
{
    printf("Spped step simulator!\n");
    doAdjust(500, 10, 1000);
    return 0;
}

static mcu_error mcu_pwmAccCalcNextSpeedIterConcaveConvex(const mcu_actuator act,
        accMoveDirective * directive, uint32_t time_ms, int32_t * speedStep)
{
    float v0 = (float)directive->initialSpeed;
    float phaseTime = (float)time_ms/(float)1000.00;
    float jerk = (float)directive->jerk;

    const float a0 = 0;
    float speed = v0 + a0 * phaseTime + (jerk / (float)2.00) *
            (phaseTime * phaseTime);

    if (speed < 0) {
        speed = 0;
    }
    *speedStep = (int32_t)speed;
    return MCU_ERROR_NONE;
}

void doAdjust(int maxAcc, int jerkFreq, int targetSpeed)
{
    accMoveDirective directive;
    directive.breakTime = 0;
    directive.breakZone = 0;
    directive.currentSpeed = 0;
    directive.initialSpeed = 0;
    directive.jerk = maxAcc / 4;
    directive.jerkFreq = jerkFreq;
    directive.maxAcceleration = maxAcc;
    directive.period = 0;
    directive.phase = pwmAccPhaseConcave;
    directive.targetPos = 15000000;
    directive.targetSpeed = targetSpeed;


    uint32_t time_ms = 0;
    int32_t speedStep = 0;

    int32_t delay_us = 1000000 / jerkFreq;
    int32_t delay_ms = delay_us / 1000;

    bool accelerate = directive.currentSpeed < targetSpeed;
    bool stop = false;

    while (!stop) {

        mcu_error error = mcu_pwmAccCalcNextSpeedIterConcaveConvex(
            0, &directive, time_ms, &speedStep);

        // Update the speed
        directive.currentSpeed += speedStep;
        time_ms += delay_ms;
        printf("SpeedStep: %d, CurretnSpeed: %d, Time (ms): %d\n",
            speedStep, directive.currentSpeed,time_ms);
        fflush(stdout);

        // Check that we wont exceed thelimits
        if (accelerate) {
            stop = directive.currentSpeed > targetSpeed;
        } else {
            stop = directive.currentSpeed < targetSpeed;
        }
        usleep(delay_us);
    }

}
