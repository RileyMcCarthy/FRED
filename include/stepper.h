#include <stdint.h>
#include <stdbool.h>
#include "planner.h"

#define STEPPER_COG_SIZE 1000

typedef struct stepper_t {

    // Cog Parameters
    uint8_t cogid;
    long cog_stack[STEPPER_COG_SIZE]; // Cog memory
    uint8_t step_pin;
    uint8_t dir_pin;

    // Motion Planning
    double v_goal;
    double v_max;
    double a_max;
    
    // PID Controller
    double kp;
    double ki;
    double kd;
    double e[2];
    int periodms;
    
    // Will read some feedback and return the error
    int (*error)();

    // Stepper state (absolute)
    int position; // current position in steps (absolute)
    int velocity; // steps/s (current speed)
} StepperPID;

bool stepper_start(Stepper *stepper, int step_pin, int dir_pin);
void stepper_callback(Stepper *stepper, int periodms, int (*error)());