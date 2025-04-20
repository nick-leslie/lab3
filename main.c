/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include "buttons.h"
#include "driverlib/timer.h"
uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
//
//
//uint32_t gSystemClock; // [Hz] system clock frequency
//volatile uint32_t gTime = 8345; // time in hundredths of a second
//
//volatile bool should_update_draw_buffer = true;
//#define FIFO_SIZE 11        // FIFO capacity is 1 item fewer
//#define FIFO_MAX 10        // FIFO capacity is 1 item fewer
//typedef uint32_t DataType;      // FIFO data type
//volatile DataType fifo[FIFO_SIZE];  // FIFO storage array
//volatile int fifo_head = 0; // index of the first item in the FIFO
//volatile int fifo_tail = 0; // index one step past the last item
//#define NUMBER_OF_VOLT_SETTINGS 5
//volatile int volt_scale_index = 0;
//volatile bool find_rising = true;
uint32_t count_unloaded = 0;
//uint32_t count_loaded = 0;
//
//
void signal_init();
uint32_t cpu_load_count(void);

//
//char cpu_str[50];


/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();
    //do I need this
    FPUEnable();
    FPULazyStackingEnable();

    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock * 0.016); // 1 sec interval

    // hardware initialization goes here
    count_unloaded = cpu_load_count();
    signal_init();
    sample_init();
    ButtonInit();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
void signal_init() {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2,
    GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // use system clock without division
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1,roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}

void clock_tick() {
    Semaphore_post(button);
}

void task0_func(UArg arg1, UArg arg2)
{
    IntMasterEnable();

    while (true) {
        // do nothing
    }
}
