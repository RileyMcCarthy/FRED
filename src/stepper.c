#include "stepper.h"
#include <propeller2.h>
#include <math.h>

// lookup how to convert to integer math
static double x_next(double x_current, double v_current, double a_max, double dt)
{
    return x_current + v_current*dt + 0.5*a_max*dt*dt;
}

static double v_next(double v_goal, double v_current, double a_max, double dt)
{
    double a = a_max;
    if (v_goal < v_current)
    {
        a = -a_max;
    }
    double v = v_current + a*dt;
    if (a > 0 && v > v_goal)
    {
        v = v_goal;
    }
    else if (a < 0 && v < v_goal)
    {
        v = v_goal;
    }
    return v;
}

// Should move stepper to setpoint using Trajectory Planner
static void stepper_cog(void *arg)
{
    Stepper *stepper = (Stepper *)arg;
    int lastus = _getus();
    while (1)
    {
        int currentus = _getus();
        double dt = (currentus-lastus)/1000000.0;
        lastus = currentus;

        stepper->velocity = v_next(stepper->v_goal, stepper->velocity, stepper->a_max, dt);
        double  = x_next(stepper->position, stepper->velocity, stepper->a_max, dt);
        // Need to use different variables for current position and trajectory position
        int step = false;
        if (x > stepper->position)
        {
            _pinl(stepper->dir_pin);
            stepper->position++;
            step=true;
        }
        else if (x < stepper->position)
        { 
            _pinl(stepper->dir_pin);
            stepper->position--;
            step=true;
        }

        //printf("step: %d,%d\n", _getus()-startx,x);
        if (step)
        {
            _pinl(stepper->step_pin);
            _waitus(1);
            _pinh(stepper->step_pin);
            _waitus(1);
        }
    }
}

bool stepper_start(Stepper *stepper, int step_pin, int dir_pin)
{
    stepper->step_pin = step_pin;
    stepper->dir_pin = dir_pin;

    stepper->v_goal = 0;

    stepper->kp = 0.0;
    stepper->ki = 0.0;
    stepper->kd = 0.0;
    stepper->p[0] = 0.0;
    stepper->p[1] = 0.0;
    stepper->error = NULL;
    stepper->periodms = 0;

    stepper->position = 0;
    stepper->velocity = 0;
    stepper->cogid = _cogstart_C(stepper_cog, stepper, &(stepper->cog_stack)[0], sizeof(long) * STEPPER_COG_SIZE);
    if (stepper->cogid != -1)
    {
      return true;
    }
    return false;
}

void stepper_callback(Stepper *stepper, int periodms, int (*error)())
{
    stepper->error = error;
    stepper->periodms = periodms;
}
