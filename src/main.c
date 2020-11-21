#include <SDL2/SDL.h>
#include <stdio.h>

#include <unistd.h>
#include <time.h>
#include "smcp.h"
#include "main.h"


#define AREA_WIDTH_X 1000
#define AREA_WIDTH_Y 1000
#define SCALE_FACTOR_X 0.3
#define SCALE_FACTOR_Y 0.1

#define POINTS_BASELINE_Y 200
#define MIN_SPEED_UMS  0
#define MAX_SPEED_UMS  1000


static adjustSum doAdjust(SDL_Renderer *renderer, adjustParams * args);

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
            usleep(10000);
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
    adjustSum convex_summary = doAdjust(renderer, &args);

    // ... Decceleration
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);

    args.initSpeed = args.targetSpeed - convex_summary.lastStep;
    args.targetSpeed = MIN_SPEED_UMS;
    args.runtime_ms = convex_summary.runTime - 100;  // FIXME. Hard coded. One step less since the current impl. handles full steps only
    adjustSum concave_summary = doAdjust(renderer, &args);

    printf("RUNTIMES (ms). Convex: %d, Concave: %d\n", convex_summary.runTime, concave_summary.runTime);
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
    //float vh = (float)directive->initialSpeed;
    float phaseTime = (float)time_ms/(float)1000.00;
    float jerk = (float)directive->jerk;
    //float speed = vh + jerk * (phaseTime * phaseTime) / 2.00;
    float speed = jerk * (phaseTime * phaseTime) / 2.00;

    *speedStep = (int32_t)speed;
    return MCU_ERROR_NONE;
}

adjustSum doAdjust(SDL_Renderer *renderer, adjustParams * args)
{
    adjustSum retval;
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
    int32_t speedDelta = 0;

    int32_t delay_us = 1000000 / args->adjFreq;
    int32_t delay_ms = delay_us / 1000;
    int32_t time_ms = 0;
    bool stop = false;

    int iter = 0;
    SDL_Point points[100];
    memset((char*)&points, '\0', sizeof(points));

    do {
        if (args->runtime_ms) {
            time_ms = args->runtime_ms - runtime_ms;
        } else {
            time_ms = runtime_ms;
        }

        // Get the next speed step
        mcu_error error = mcu_pwmAccCalcNextSpeedIterConcaveConvex(
            0, &directive, time_ms, &speedDelta);

        // Update the current speed
        directive.currentSpeed += speedDelta;

        
        // Check that we wont exceed the limits
        if (accelerate) {
            stop = directive.currentSpeed >= args->targetSpeed;
        } else {
            stop = directive.currentSpeed <= args->targetSpeed;
        }
        // Scale the x-axis a bit - just to see the diff between phases
        uint32_t scaledTimeMs = accelerate ? time_ms : time_ms + 200;;
        if (stop) {            
            uint32_t prevSpeed = directive.currentSpeed;
            directive.currentSpeed = args->targetSpeed;
            retval.lastStep = directive.currentSpeed - prevSpeed;
        }

        // Print the data
        SDL_Point point = resolvePoint(scaledTimeMs, directive.currentSpeed);
        points[iter] = point;

        printf("[%d] SpeedDelta: %d, CurrentSpeed: %d, Total time (ms): %d\n",
            iter, speedDelta, directive.currentSpeed, total_time_ms);
        fflush(stdout);
        iter++;
    
        if (!stop) {
            total_time_ms += delay_ms;
        }
        runtime_ms += delay_ms;

    } while (!stop);

    // Draw the curve
    SDL_RenderDrawLines(renderer, points, iter);
    SDL_RenderPresent(renderer);
    usleep(1000000);
    retval.runTime = runtime_ms;
    return retval;
}

SDL_Point resolvePoint(int x_val, int y_val)
{
    SDL_Point retval;
    retval.x = x_val;
    retval.y = (AREA_WIDTH_Y - y_val) / SCALE_FACTOR_Y;
    return retval;
}
