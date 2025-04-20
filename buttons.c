/*
 * buttons.c
 *
 *  Created on: Aug 12, 2012, modified 9/8/2017
 *      Author: Gene Bogdanov
 *
 * ECE 3849 Lab button handling
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <xdc/cfg/global.h>                 //header file for statically defined objects/handles
#include <ti/sysbios/BIOS.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "sampling.h"

void ButtonInit() {
    // GPIO PJ0 and PJ1 = EK-TM4C1294XL buttons 1 and 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTH_BASE,  GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTK_BASE,  GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4 );
    GPIOPadConfigSet(GPIO_PORTD_BASE,  GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // analog input AIN13, at GPIO PD2 = BoosterPack Joystick HOR(X)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    // analog input AIN17, at GPIO PK1 = BoosterPack Joystick VER(Y)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    // initialize ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // round divisor up
    gADCSamplingRate = pll_frequency / (16 * pll_divisor); // actual sampling rate may differ from ADC_SAMPLING_RATE
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor); // only ADC0 has PLL clock divisor control

    // initialize ADC sampling sequence
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);                             // Joystick HOR(X)
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);  // Joystick VER(Y)
    ADCSequenceEnable(ADC0_BASE, 0);
}

// public globals
volatile uint32_t gButtons = 0; // debounced button state, one per bit in the lowest bits
                                // button is pressed if its bit is 1, not pressed if 0
uint32_t gJoystick[2] = {0};    // joystick coordinates
uint32_t gADCSamplingRate;      // [Hz] actual ADC sampling rate
// update the debounced button state gButtons
void ButtonDebounce(uint32_t buttons)
{
	int32_t i, mask;
	static int32_t state[BUTTON_COUNT]; // button state: 0 = released
									    // BUTTON_PRESSED_STATE = pressed
									    // in between = previous state
	for (i = 0; i < BUTTON_COUNT; i++) {
		mask = 1 << i;
		if (buttons & mask) {
			state[i] += BUTTON_STATE_INCREMENT;
			if (state[i] >= BUTTON_PRESSED_STATE) {
				state[i] = BUTTON_PRESSED_STATE;
				gButtons |= mask; // update debounced button state
			}
		}
		else {
			state[i] -= BUTTON_STATE_DECREMENT;
			if (state[i] <= 0) {
				state[i] = 0;
				gButtons &= ~mask;
			}
		}
	}
}

// sample joystick and convert to button presses
void ButtonReadJoystick(void)
{
    ADCProcessorTrigger(ADC0_BASE, 0);          // trigger the ADC sample sequence for Joystick X and Y
    while(!ADCIntStatus(ADC0_BASE, 0, false));  // wait until the sample sequence has completed
    ADCSequenceDataGet(ADC0_BASE, 0, gJoystick);// retrieve joystick data
    ADCIntClear(ADC0_BASE, 0);                  // clear ADC sequence interrupt flag

    // process joystick movements as button presses using hysteresis
    if (gJoystick[0] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 5; // joystick right in position 5
    if (gJoystick[0] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 5);

    if (gJoystick[0] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 6; // joystick left in position 6
    if (gJoystick[0] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 6);

    if (gJoystick[1] > JOYSTICK_UPPER_PRESS_THRESHOLD) gButtons |= 1 << 7; // joystick up in position 7
    if (gJoystick[1] < JOYSTICK_UPPER_RELEASE_THRESHOLD) gButtons &= ~(1 << 7);

    if (gJoystick[1] < JOYSTICK_LOWER_PRESS_THRESHOLD) gButtons |= 1 << 8; // joystick down in position 8
    if (gJoystick[1] > JOYSTICK_LOWER_RELEASE_THRESHOLD) gButtons &= ~(1 << 8);
}

// autorepeat button presses if a button is held long enough
uint32_t ButtonAutoRepeat(void)
{
    static int count[BUTTON_AND_JOYSTICK_COUNT] = {0}; // autorepeat counts
    int i;
    uint32_t mask;
    uint32_t presses = 0;
    for (i = 0; i < BUTTON_AND_JOYSTICK_COUNT; i++) {
        mask = 1 << i;
        if (gButtons & mask)
            count[i]++;     // increment count if button is held
        else
            count[i] = 0;   // reset count if button is let go
        if (count[i] >= BUTTON_AUTOREPEAT_INITIAL &&
                (count[i] - BUTTON_AUTOREPEAT_INITIAL) % BUTTON_AUTOREPEAT_NEXT == 0)
            presses |= mask;    // register a button press due to auto-repeat
    }
    return presses;
}




// ISR for scanning and debouncing buttons
void button_task(void) {
    while(1) {
    Semaphore_pend(button, BIOS_WAIT_FOREVER); // todo look into why we are blowing up the stack here
    uint32_t gpio_buttons =
            ~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0) // EK-TM4C1294XL buttons in positions 0 and 1
            | ((~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & (GPIO_PIN_1)) << 1) // s1
            | ((~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & (GPIO_PIN_6)) >> 3) // s2
            | ((~GPIOPinRead(GPIO_PORTD_BASE, 0xff) & (GPIO_PIN_4))) // select
            ;
        uint32_t old_buttons = gButtons;    // save previous button state
        ButtonDebounce(gpio_buttons);       // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();               // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();      // autorepeat presses if a button is held long enough
        //todo convert this to be a way to post new
        uint8_t code = 0;
        if (presses & 0x01) { //USR_SW_1 bit 0
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);
        }
        if (presses & 0x02) { //USR_SW_2 bit 1
            code =1;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x04) { //s1 bit 2
            code =2;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x08) { //s2 bit 3
            code = 3;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x10) { //select bit 4
            code = 4;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x20) { // right  bit 5
            code = 5;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x40) { // left  bit 6
            code = 6;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x80) { //up bit 7
            code = 7;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);

        }
        if (presses & 0x100) { //down bit 8
            code = 8;
            Mailbox_post(buttons,&code,BIOS_NO_WAIT);
        }
  }
}
