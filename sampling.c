#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <xdc/cfg/global.h>                 //header file for statically defined objects/handles
#include <ti/sysbios/BIOS.h>                //mandatory - if you call APIs like BIOS_start()
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "sysctl_pll.h"
#include "driverlib/sysctl.h"
#include "Crystalfontz128x128_ST7735.h"
#include "driverlib/fpu.h"

#include "sampling.h"

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
// index wrapping macro
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1))

volatile uint16_t draw_buffer[128];

// latest sample index
volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;
volatile int16_t gADCBuffer[ADC_BUFFER_SIZE]; // circular buffer
volatile uint32_t gADCErrors = 0; // number of missed ADC deadlines
#define VIN_RANGE 3.3
#define MAX_VOLTS 3.3
#define ADC_BITS 12
#define NUMBER_OF_VOLT_SETTINGS 5
volatile uint32_t ADC_OFFSET = (1 << ADC_BITS) /2;
int volt_scale_index = 0;
bool find_rising = true;
const char * const gVoltageScaleStr[] = {
"100 mV", "200 mV", "500 mV", " 1 V", " 2 V"
};
const float fVoltsPerDiv[] = {
0.1, 0.2, .5, 1, 2
};
float cpu_load = 0.0;
extern uint32_t count_unloaded;
uint32_t count_loaded = 0;
char cpu_str[50];

uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}

void sample_init() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3); // GPIO setup for analog input AIN3


    // initialize ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE)
    + 1; // round up
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,
    pll_divisor);

    // choose ADC1 sequence 0; disable before configuring
    ADCSequenceDisable(ADC1_BASE, 0);

    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0); // specify the "Always" trigger

    // in the 0th step, sample channel 3 (AIN3)
    // enable interrupt, and make it the end of sequence
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

    // enable the sequence. it is now sampling
    ADCSequenceEnable(ADC1_BASE, 0);

    // enable sequence 0 interrupt in the ADC1 peripheral
    ADCIntEnable(ADC1_BASE,0);

    //IntPrioritySet(INT_ADC1SS0, SAMPLING_INT_PRIORITY); // set ADC1 sequence 0 interrupt priority

    // enable ADC1 sequence 0 interrupt in int. controller
    //setup the draw buffer
//    IntEnable(INT_ADC1SS0);
    uint8_t i;
    for(i=0;i<128;++i) {
        draw_buffer[i] = 0;
    }
}



void ADC_ISR(void)
{
    // clear ADC1 sequence0 interrupt flag in the ADCISC register
    //ADCIntClear(ADC1_BASE,0);
    (ADC1_ISC_R) = (ADC1_ISC_R | 0x01);
    //check for missed intrupts
    if(ADC1_OSTAT_R & ADC_OSTAT_OV0) {
        gADCErrors++; // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0; // clear overflow condition
    }
    //get the index for sample.
    // ADC_BUFFER_WRAP is a macro that handles wrapping the buffer
    gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1);

    // read sample from the ADC1 sequence 0 FIFO using direct address access
    uint16_t sample = (uint16_t)(ADC1_SSFIFO0_R);
    gADCBuffer[gADCBufferIndex] = sample; // read from sample sequence 0
}

int RisingTrigger(void) // search for rising edge trigger
{
    int init_index =  gADCBufferIndex - (LCD_HORIZONTAL_MAX /2);
    // Step 1
    int x = init_index;/* half screen width; dont use a
    magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
    if ( gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
            gADCBuffer[ADC_BUFFER_WRAP(x-1)] < ADC_OFFSET) {
            break;
        }
    }
    // Step 3
    if (x == x_stop) { // for loop ran to the end
        x =  gADCBufferIndex - (LCD_HORIZONTAL_MAX /2); // reset x back to how it was initialized
    }
    return x;
}


int FallingTrigger(void) // search for rising edge trigger
{
    int init_index =  gADCBufferIndex - (LCD_HORIZONTAL_MAX /2);
    // Step 1
    int x = init_index;/* half screen width; dont use a
    magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2;
    for (; x > x_stop; x--) {
    if ( gADCBuffer[ADC_BUFFER_WRAP(x)] <= ADC_OFFSET &&
            gADCBuffer[ADC_BUFFER_WRAP(x-1)] > ADC_OFFSET) {
            break;
        }
    }
    // Step 3
    if (x == x_stop) { // for loop ran to the end
        x = init_index; // reset x back to how it was initialized
    }
    return x;
}


void display_task() {
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation


    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    // full-screen rectangle
    uint16_t max_width = GrContextDpyWidthGet(&sContext)-1;
    uint16_t max_height =  GrContextDpyHeightGet(&sContext)-1;

    tContext* context = &sContext;

    tRectangle rectFullScreen = {0, 0, max_width,max_height};
    while(1) {
        Semaphore_pend(display, BIOS_WAIT_FOREVER);
        GrContextForegroundSet(context, ClrBlack);
        GrRectFill(context, &rectFullScreen); // fill screen with black
        GrContextForegroundSet(context, ClrRoyalBlue); // blue line
        uint8_t i;
        for(i=4; i<max_width;i+=PIXELS_PER_DIV) {
            //this works because we are working with a square display
            GrLineDrawV(context,i,0,max_height);
            GrLineDrawH(context,0,max_width,i);
        }
        GrContextForegroundSet(context, ClrYellow); // blue line
        for(i=1;i<LCD_VERTICAL_MAX;++i) {
           int16_t prev_y = draw_buffer[i-1];
           int16_t y = draw_buffer[i];
           if(y < 0) {
               y = 0;
           }
           if(y > LCD_VERTICAL_MAX) {
               y=LCD_VERTICAL_MAX-1;
           }
           if (prev_y < 0) {
               prev_y=0;
           }
           if(prev_y > LCD_VERTICAL_MAX) {
               prev_y=LCD_VERTICAL_MAX-1;
           }
           GrLineDraw(context,i-1,prev_y,i,y);
        }
        GrStringDraw(context,gVoltageScaleStr[volt_scale_index],-1,30,0,false);
        if(find_rising == true) {
            GrStringDraw(context,"R",1,80,0,false);
        } else {
            GrStringDraw(context,"F",1,80,0,false);
        }
        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load
        sprintf(cpu_str,"CPU load %f",cpu_load);
        GrStringDraw(&sContext,cpu_str,-1,0,100,false);
        GrFlush(context); // flush the frame buffer to the LCD
    }
}
void waveform_task() {
    IntMasterEnable();
    int32_t search_index;

    while(1) {
        Semaphore_pend(waveform, BIOS_WAIT_FOREVER);
        if(find_rising == true) {
            search_index=RisingTrigger();
        } else {
            search_index=FallingTrigger();
        }
        int i;

        for(i=(LCD_HORIZONTAL_MAX/2) * -1;i<LCD_HORIZONTAL_MAX/2;++i) {
            int index = ADC_BUFFER_WRAP(search_index +i);
            int32_t sample = gADCBuffer[index];
            int draw_index = (LCD_HORIZONTAL_MAX/2)+i;
            draw_buffer[draw_index] = sample;

        }
        //I put this pend at the bottem so it runs once at start before
        Semaphore_post(processing);
    }
    //todo signal
}

void processing_task() {

    while(1) {
        Semaphore_pend(processing, BIOS_WAIT_FOREVER);
        int scale_i;

    //    copy_i=0;copy_i< LCD_HORIZONTAL_MAX/2;++i
        for(scale_i=0;scale_i< LCD_HORIZONTAL_MAX;++scale_i) {;
            uint32_t adc_bit=(1 << ADC_BITS);
            float fscale_top = (VIN_RANGE * PIXELS_PER_DIV);
            float fscale_bottem = (adc_bit * fVoltsPerDiv[volt_scale_index]);
            float fScale = fscale_top/fscale_bottem;

            int sample = (int)draw_buffer[scale_i];
            int sample_mod = (sample - ADC_OFFSET);
            float rounded = roundf(fScale * sample_mod);
            int y = LCD_VERTICAL_MAX/2 - (int)rounded;

            draw_buffer[scale_i] = y;
        }
        Semaphore_post(display);
        Semaphore_post(waveform);
    }
}

void user_input_task() {
 while(1) {
     uint8_t code;
     Mailbox_pend(buttons, &code, BIOS_WAIT_FOREVER);
     if(code == 2) {
         find_rising = !find_rising;
     }
     if(code ==5) {
         volt_scale_index++;
         if(volt_scale_index >= NUMBER_OF_VOLT_SETTINGS) {
             volt_scale_index=0;
         }
     }
     if(code == 6) {
      volt_scale_index--;
      if(volt_scale_index < 0) {
          volt_scale_index=NUMBER_OF_VOLT_SETTINGS-1;
      }
     }
     Semaphore_post(display);
 }
}
