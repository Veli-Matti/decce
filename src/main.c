#include <SDL2/SDL.h>
#include <stdio.h>

#include <unistd.h>
#include <time.h>
#include "include/main.h"

#define POINTS_COUNT 4

static SDL_Point points[POINTS_COUNT] = {
    {320, 200},
    {300, 240},
    {340, 240},
    {320, 200}
};

int addInIncrements();
static void doAdjust(int maxAcc, int jerkFreq, int targetSpeed);
static SDL_Window *sdl_init();


int addInIncrements() {
    time_t seconds;
    for (int i = 0; i < 10; i++) {

        seconds = time(NULL);
        printf("%d, secs = %d\n", i, (int)seconds);
        fflush(stdout);
        usleep(1000000);
    }
    return 0;
}



int main()
{

    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    SDL_bool done = SDL_FALSE;

    // Init SDL2
    if (SDL_Init(SDL_INIT_VIDEO) == 0) {
        // Create target window and renderer
        if (SDL_CreateWindowAndRenderer(640, 480, 0, &window, &renderer) == 0) {
            // Set the color for lines etc
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
            SDL_RenderClear(renderer);

            SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        }
    }

    printf("Speed step simulator!\n");


    doAdjust(500, 10, 1000);

    // Draw the lines
    SDL_RenderDrawLines(renderer, points, POINTS_COUNT);
    SDL_RenderPresent(renderer);

    SDL_Event event;

    while (!done) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                done = SDL_TRUE;
            }
        }
    }

    if (renderer) {
        SDL_DestroyRenderer(renderer);
    }
    if (window) {
        SDL_DestroyWindow(window);
    }

    SDL_Quit();
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
        printf("SpeedStep: %d, CurrentSpeed: %d, Time (ms): %d\n",
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
