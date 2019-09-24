#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/sysctl.h"
#include"driverlib/gpio.h"
uint8_t ui8PinData=4; //initialize light as blue

int main(void)
{
    uint8_t red = 2;
    uint8_t blue = 4;
    uint8_t green = 8;

    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    while (1) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, red); //RED FLASH
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, green); //GREEN FLASH
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, blue); //BLUE FLASH
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, red); //RED GREEN FLASH
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, green);
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, red); //RED BLUE FLASH
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, blue);
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, blue); //BLUE GREEN FLASH
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, green);
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, red); //RED GREEN BLUE FLASH
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, green);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, blue);
        SysCtlDelay(6666666); //delay of 0.5s
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
        SysCtlDelay(6666666); //delay of 0.5s



    }
    return 0;
}
