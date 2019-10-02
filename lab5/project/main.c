#include<stdint.h>
#include<stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/debug.h"
#include"driverlib/sysctl.h"
#include"driverlib/adc.h"
#include"driverlib/gpio.h"
#include"driverlib/interrupt.h"
#include "driverlib/timer.h"
#define TARGET_IS_BLIZZARD_RB1
#include"driverlib/rom.h"

#ifdef DEBUG
void__error__(char*pcFilename, uint32_t ui32Line)
{
}
#endif

uint32_t ui32ADC0Value[1];
volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;

int main()
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //peripherals for LEDs enabled
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCHardwareOversampleConfigure(ADC0_BASE, 32); //32 measurements averaged for sample. stops value from switching around too much

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); //SAMPLE SEQUENCER 3 ENABLED
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);

    //Timer 1 Configure
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() * .5));
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
    TimerEnable(TIMER1_BASE,TIMER_A);

    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntEnable(ADC0_BASE, 3);

    while(1) //read temp sensor and calculate the temp endlessly
    {
    }
}

void Timer1IntHandler(void)
{
    ADCIntClear(ADC0_BASE, 3); //clear adc conversion done flag before writing code that depends on it. change to sequence 2
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); //clear timer

    TimerLoadSet(TIMER1_BASE, TIMER_A, (SysCtlClockGet() * .5)); //.5sec
    ADCProcessorTrigger(ADC0_BASE, 3);

    while(!ADCIntStatus(ADC0_BASE, 3, false)) //wait for conversion to finish
    {
    } //if loop exited conversion is complete

    ADCSequenceDataGet(ADC0_BASE, 3, ui32ADC0Value); //gets samples from the array
    ui32TempValueC = (1475 -((2475 * ui32ADC0Value[0])) / 4096)/10;
    ui32TempValueF = ((ui32TempValueC * 9) + 160) / 5;

    if (ui32TempValueF < 70) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 2); // Turn on the red LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); // Turn off the LED
    }
    else if (ui32TempValueF > 70) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4); // Turn on the blue LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // Turn off the LED
    }
}
