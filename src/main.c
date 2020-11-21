#include <SDL2/SDL.h>
#include <stdio.h>

#include <unistd.h>
#include <time.h>
#include "smcp.h"
#include "main.h"


#define AREA_WIDTH_X 1000
#define AREA_WIDTH_Y 1000
#define SCALE_FACTOR_X 0.1
#define SCALE_FACTOR_Y 0.1

#define POINTS_BASELINE_Y 200
#define MIN_SPEED_UMS  0
#define MAX_SPEED_UMS  1000


static uint32_t doAdjust(SDL_Renderer *renderer, adjustParams * args);

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
            SDL_SetWindowTitle(window, "Speed change in time scale");

            // Set the color for lines etc
            // ... clear the drwing area (set white)
            SDL_SetRenderDrawColor(renderer, 211, 211, 211, SDL_ALPHA_OPAQUE);
            SDL_RenderClear(renderer);
            SDL_RenderPresent(renderer);

            // Set the scale factor
            SDL_RenderSetScale(renderer, SCALE_FACTOR_X, SCALE_FACTOR_Y);
            usleep(1000);
        }
    }

    // The actual operations
    // ... Acceleration
    SDL_SetRenderDrawColor(renderer, 0, 100, 0, SDL_ALPHA_OPAQUE);

    adjustParams args;
    args.adjFreq = 10;
    args.initSpeed = MIN_SPEED_UMS;
    args.maxAcc = 500;
    args.targetSpeed = MAX_SPEED_UMS;
    args.runtime_ms = 0;
    uint32_t convex_ms = doAdjust(renderer, &args);

    // ... Decceleration
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);

    args.initSpeed = args.targetSpeed;
    args.targetSpeed = MIN_SPEED_UMS;
    args.runtime_ms = convex_ms;
    uint32_t concave_ms = doAdjust(renderer, &args);

    printf("RUNTIMES (ms). Convex: %d, Concave: %d\n", convex_ms, concave_ms);
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
    float vh = (float)directive->initialSpeed;
    float phaseTime = (float)time_ms/(float)1000.00;
    float jerk = (float)directive->jerk;

    // v(t) = vh + as * j * (t^2 / 2)
    float speed = vh + jerk * (phaseTime * phaseTime) / 2.00;

    *speedStep = (int32_t)speed;
    return MCU_ERROR_NONE;
}

uint32_t doAdjust(SDL_Renderer *renderer, adjustParams * args)
{
    accMoveDirective directive;

    bool accelerate = args->initSpeed < args->targetSpeed;

    directive.breakTime = 0;
    directive.breakZone = 0;
    directive.currentSpeed = args->initSpeed;
    directive.initialSpeed = args->initSpeed;
    directive.jerk = accelerate ? (args->maxAcc / 4) : -(args->maxAcc / 4);
    directive.jerkFreq = args->adjFreq;
    directive.maxAcceleration = args->maxAcc;
    directive.period = 0;
    directive.phase = pwmAccPhaseConcave;
    directive.targetPos = 15000000;
    directive.targetSpeed = args->targetSpeed;

    static uint32_t total_time_ms = 0;
    uint32_t runtime_ms = 0;
    int32_t speedStep = 0;

    int32_t delay_us = 1000000 / args->adjFreq;
    int32_t delay_ms = delay_us / 1000;
    int32_t time_ms = 0;
    bool stop = false;

    int iter = 0;
    SDL_Point points[100];

    do {
        if (args->runtime_ms) {
            time_ms = args->runtime_ms - runtime_ms;
        } else {
            time_ms = runtime_ms;
        }

        // Get the next speed step
        mcu_error error = mcu_pwmAccCalcNextSpeedIterConcaveConvex(
            0, &directive, time_ms, &speedStep);

        // Update the speed
        if (accelerate) {
            directive.currentSpeed = speedStep;
        } else {
            directive.currentSpeed = args->initSpeed - speedStep;
        }

        uint32_t scaledTimeMs;
        // Check that we wont exceed the limits
        if (accelerate) {
            stop = directive.currentSpeed >= args->targetSpeed;
            scaledTimeMs = time_ms;
        } else {
            stop = directive.currentSpeed <= args->targetSpeed;
            scaledTimeMs = time_ms + 200;
        }
        if (stop) {
            directive.currentSpeed = args->targetSpeed;
        }

        // Print the data
        SDL_Point point = resolvePoint(scaledTimeMs, directive.currentSpeed);
        points[iter] = point;

        printf("[%d] SpeedStep: %d, CurrentSpeed: %d, Total time (ms): %d\n",
            iter, speedStep, directive.currentSpeed, total_time_ms);
        fflush(stdout);

        if (!stop) {
            // usleep(delay_us);
            total_time_ms += delay_ms;
            runtime_ms += delay_ms;
        }
        iter++;

    } while (!stop);

    // Draw the curve
    SDL_RenderDrawLines(renderer, points, iter);
    SDL_RenderPresent(renderer);
    usleep(100000);
    return runtime_ms;
}

SDL_Point resolvePoint(int x_val, int y_val)
{
    SDL_Point retval;
    retval.x = x_val;
    retval.y = (AREA_WIDTH_Y - y_val) / SCALE_FACTOR_Y;
    return retval;
}
