#include "fred.h"
#include <stdint.h>
#include <stdio.h>
#include <propeller2.h>
#include <stepper.h>
#define P2_TARGET_MHZ 300
#include <sys/p2es_clock.h>

typedef struct __using("ssd130x.spin2") Display;

static void test_display()
{  
    int width = 128;
    int height = 64;
    int freq = 100000;
    uint8_t display_buffer[128*64/8];
    Display display;
    display.startx(OLED_SCK_PIN,OLED_SDA_PIN, -1, freq, 0, width, height, display_buffer);
    display.clear();
    display.preset_128x64();
   // display.putchar("Hello, world!");
}

static void test_relays()
{
    int start_pin = RELAY1_PIN;
    while(1)
    {
        for (int i = 0; i < 4; i++)
        {
            int pin = start_pin+i;
            _pinl(pin);
            _waitms(100);
            _pinh(pin);
            _waitms(100);
        }
    }
}

static test_adc()
{
    _pinstart(POT1_PIN, p_adc_1x | p_adc, 13, 0);
    while(1)
    {
         uint32_t value = _rdpin(POT1_PIN);
         printf("ADC value: %d\n", value);
         _waitms(1000);
    }
}

static void test_stepper()
{
    _pinl(STEPPER1_DIR_PIN);
    _pinh(STEPPER4_DIR_PIN);
    while(1)
    {
            _pinl(STEPPER1_STEP_PIN);
            _pinl(STEPPER4_STEP_PIN);
            _waitus(2000);
            _pinh(STEPPER1_STEP_PIN);
            _pinh(STEPPER4_STEP_PIN);
            _waitus(2000);
    }
}

static void test_stepper_pot()
{
    _pinl(STEPPER4_DIR_PIN);
    _pinh(STEPPER2_DIR_PIN);

    _pinstart(POT1_PIN, p_adc_1x | p_adc, 13, 0);
    int max_adc = 10000;
    int min_adc = 4000;

    int max_speed = 1000;
    int min_speed = 1;

    while(1)
    {
        int value = _rdpin(POT1_PIN);
        if (value < min_adc)
        {
            value = min_adc;
           //printf("Error value to low\n");
        }
        uint32_t delay = (value-min_adc)*(max_speed)/(max_adc-min_adc);
        //printf("Value: %d, Delay; %d\n",value,delay);
        _pinl(STEPPER4_STEP_PIN);
        _waitus(delay/2);
        _pinh(STEPPER4_STEP_PIN);
        _waitus(delay/2);
    }
}

static void test_setpoint_pot()
{
    Stepper stepper1;
    //stepper_start(&stepper1, STEPPER3_STEP_PIN, STEPPER3_DIR_PIN);

    Stepper stepper2;
    //stepper_start(&stepper2, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);

    _pinstart(POT1_PIN, p_adc_1x | p_adc, 13, 0);
    _pinstart(POT2_PIN, p_adc_1x | p_adc, 13, 0);
    int max_adc = 12000;
    int min_adc = 2000;

    int setpoint2 = 8400+(8400 - 8230)/2;
    int setpoint1 = 8222+(9570 - 8222)/2 + 500;
    
    int kp1=100;
    double ki1 = 0.0;
    int integral1 = 0;

    int kp2=50;
    double ki2 = 0.1;
    int integral2 = 0;
    // setpoint1_max = 9570, setpoint2_max = 8222, setpoint 1 
    // setpoint1_min = 8400, setpoint2_min = 8230
    _pinl(STEPPER3_DIR_PIN);
    while(1)
    {
        int value1 = _rdpin(POT1_PIN);
        int error1 = (setpoint1-value1);
        integral1 += error1;
        int pid1 = kp1*error1;

        int wait_time = 94400 - pid1;
        if (error1 < 0)
        {
            printf("Error1 to high: %d\n",error1);
            continue;
        }
        if (wait_time < 2000)
        {
            wait_time = 2000;
        }
        //printf("Error1: %d",error1);
        wait_time /= 2000;

        _pinl(STEPPER3_STEP_PIN);
        _waitus(wait_time);
        _pinh(STEPPER3_STEP_PIN);
        _waitus(wait_time);
        _pinl(STEPPER3_STEP_PIN);

        //stepper_setpoint(&stepper1, -pid1);
        //printf("Value1: %d, error1: %d, pid: %d, wait: %d\n",value1,error1,pid1,wait_time);

        int value2 = _rdpin(POT2_PIN);
        int error2 = setpoint2-value2;
        integral2 += error2;
        int pid2 = kp2*error2 + ki2*integral2;
        //stepper_setpoint(&stepper2, pid2);
        //printf("Value1: %d, Value2: %d\n",value1,value2);
        //_waitms(pid1);
    }


    stepper_setpoint(&stepper1,100500);
    _waitms(1000);
    stepper_setpoint(&stepper1,-10000);
}

static void test_planner()
{
    Trajectory move;
    trajectory_init(&move);
    trajectory_velocity_plan(&move, 1000,0);
    int offset = 0;
    for (int i=0;i<200;i++)
    {
        double t = (i-offset)/100.0;
        trajectory_velocity_next(&move, t);
        printf("%f,%f,%f\n", i/100.0, move.x,move.v);
        if (i==100)
        {
            trajectory_update(&move, -1000);
            offset = i;
        }
    }
}

static void test_stepper_cog()
{
    Stepper stepper1;
    stepper_start(&stepper1, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);
    //printf("Stepper1 cog id: %d\n", 1);
    stepper_setpoint(&stepper1,100500);
}


// TODO:
// - Make steper easily change between velocity and position controll
// - Velocity control can use pwm? hard to count steps though
// - Embed PID into stepper
// - flip direction flag
// - min speed flag
static void test_setpoint_velocity()
{
    // int pwm_start(unsigned int cycleMicroseconds) _IMPL("libsimpletools/pwm.c");
    //void pwm_set(int pin, int channel, int tHigh) _IMPL("libsimpletools/pwm.c");
    //void pwm_stop(void) _IMPL("libsimpletools/pwm.c");
    Stepper stepper1;
    stepper_start(&stepper1, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);

    Stepper stepper2;
    //stepper_start(&stepper2, STEPPER4_STEP_PIN, STEPPER4_DIR_PIN);

    _pinstart(POT1_PIN, p_adc_1x | p_adc, 13, 0);
    _pinstart(POT2_PIN, p_adc_1x | p_adc, 13, 0);
    int max_adc = 12000;
    int min_adc = 2000;

    int setpoint2 = 8400+(8400 - 8230)/2;
    int setpoint1 = 8222+(9570 - 8222)/2 + 500;
    
    int kp1=100;
    double ki1 = 0.0;
    int integral1 = 0;

    int kp2=50;
    double ki2 = 0.1;
    int integral2 = 0;
    // setpoint1_max = 9570, setpoint2_max = 8222, setpoint 1 
    // setpoint1_min = 8400, setpoint2_min = 8230
    while(1)
    {
        int value1 = _rdpin(POT1_PIN);
        int error1 = (setpoint1-value1);
        integral1 += error1;
        int pid1 = kp1*error1;

        stepper_setpoint(&stepper1, pid1);
        //printf("Value1: %d, error1: %d, pid: %d, wait: %d\n",value1,error1,pid1,wait_time);

        int value2 = _rdpin(POT2_PIN);
        int error2 = setpoint2-value2;
        integral2 += error2;
        int pid2 = kp2*error2 + ki2*integral2;
        //stepper_setpoint(&stepper2, pid2);
        //printf("Value1: %d, Value2: %d\n",value1,value2);
        _waitms(100);
    }
}

//TODO
// - Create stepper cog for each stepper with a setpoint
// - Main cog will read potentiometer and set setpoint
// - Each stepper will be controlled by a pid that keeps the pot at its center value (or setpoint)

int main()
{
   //_clkset(_SETFREQ, _CLOCKFREQ);
    test_pwm();
    while(1); // loop forever
    return 0;
}