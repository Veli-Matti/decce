#include <SDL2/SDL.h>
#include <stdio.h>

#include <unistd.h>
#include <time.h>
#include "include/smcp.h"

#define AREA_WIDTH_X 1000
#define AREA_WIDTH_Y 1000
#define SCALE_FACTOR_X 0.5
#define SCALE_FACTOR_Y 0.5

#define POINTS_BASELINE_Y 200

static void doAdjust(SDL_Renderer *renderer, int maxAcc, int jerkFreq, int targetSpeed);

static SDL_Point resolvePoint(int x_val, int y_val);

int main()
{
    printf("Speed step simulator!\n");

    // Init SDL2
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    SDL_bool done = SDL_FALSE;

    if (SDL_Init(SDL_INIT_VIDEO) == 0) {
        // Create target window and renderer
        if (SDL_CreateWindowAndRenderer(AREA_WIDTH_X, AREA_WIDTH_Y, 0, &window, &renderer) == 0) {
            // Set the color for lines etc
            // ... clear the drwing area (set white)
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
            SDL_RenderClear(renderer);
            // Set the color for pen
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
            // Set the scale factor
            SDL_RenderSetScale(renderer, SCALE_FACTOR_X, SCALE_FACTOR_Y);
        }
    }

    // The actual operations
    doAdjust(renderer, 500, 10, 1000);

    // Leave the window open until 'X' is clicked
    SDL_Event event;
    while (!done) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                done = SDL_TRUE;
            }
        }
    }

    // Damp down SDL
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

void doAdjust(SDL_Renderer *renderer, int maxAcc, int jerkFreq, int targetSpeed)
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

    int iter = 0;
    SDL_Point points[50];

    while (!stop) {

        mcu_error error = mcu_pwmAccCalcNextSpeedIterConcaveConvex(
            0, &directive, time_ms, &speedStep);

        // Update the speed
        directive.currentSpeed += speedStep;
        time_ms += delay_ms;

        SDL_Point point = resolvePoint(time_ms, directive.currentSpeed);
        points[iter] = point;

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
        iter++;
    }
    // Draw the curve
    SDL_RenderDrawLines(renderer, points, iter);
    SDL_RenderPresent(renderer);

}

SDL_Point resolvePoint(int x_val, int y_val)
{
    SDL_Point retval;
    retval.x = x_val;
    retval.y = (AREA_WIDTH_Y - y_val) / SCALE_FACTOR_Y;
    return retval;
}
