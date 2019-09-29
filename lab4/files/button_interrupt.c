/*
  This code was made using TivaWare 2.1.0.12573. TivaWare rights to it are all own by
Texas Instruments


  This code shows how to toggle the RED LED (PF1) of the TM4C123 Launchpad
with the left button at PF4, using Tivaware. In this one a interrupt is
used to sense the button being pressed

  Luís Afonso

*/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.c"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.c"
#include "driverlib/gpio.h"

/* 
Make sure to modify your startup_ccs.c file to add 
extern void PortFIntHandler(void);
and

    IntDefaultHandler,                      // FLASH Control
    PortFIntHandler,                      // GPIO Port F
    IntDefaultHandler,                      // GPIO Port G
*/

/*
  These defines help if you want to change the LED pin or the Button pin.
  Remember if you change to a diferent GPIO you need to enable the system
  clock on it
*/
#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_BASE GPIO_PORTF_BASE
#define RED_LED GPIO_PIN_1

#define Button_PERIPH SYSCTL_PERIPH_GPIOF
#define ButtonBase GPIO_PORTF_BASE
#define Button GPIO_PIN_4
#define ButtonInt GPIO_INT_PIN_4

volatile uint8_t value=0; 

void PortFIntHandler(){
  uint32_t status=0;

  status = GPIOIntStatus(ButtonBase,true);
  GPIOIntClear(ButtonBase,status);

  if(status & ButtonInt == ButtonInt){
    //Then there was a Button pin interrupt
    uint8_t value=0;
    
    value= GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
    
    if( value==0)
      state^=RED_LED;

    GPIOPinWrite(LED_BASE,RED_LED, state);
    
    /*
      This delay is for deboucing but since it's in a interrupt it
      should be used a better method that is faster
    */
    SysCtlDelay(7000000);
    
    
  }
  
  
  
} 






int main(void)
{
 
 //Set the clock to 80Mhz
  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

  /*
    No need to enable the button peripheral since it's the same as the LED
    in this case
  */
  SysCtlPeripheralEnable(LED_PERIPH);
  SysCtlDelay(3);
  
  /*
    Configure the switch on the left of the launchpad, GPIO_PIN_4 to a input with
    internal pull-up.
  */
  GPIOPinTypeGPIOInput(ButtonBase, Button);
  GPIOPadConfigSet(ButtonBase ,Button,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_FALLING_EDGE);
  GPIOIntRegister(GPIO_PORTF_BASE,PortFIntHandler);
  GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
  
  /*
    Configures the Red LED, GPIO_PIN_1, to output
  */
  GPIOPinTypeGPIOOutput(LED_BASE, RED_LED);
  
  
  
 /*
  A variable is created to store the return value of GPIOPinRead and then checked 
 if it's 0 or not.The button when pressed, connected  the pin to the GND so it gives 0.
 
 We use a variable "state" because in GPIOPinWrite we need to use GPIO_PIN_1 to set the pin HIGH. 
 So we XOR the state with GPIO_PIN_1 so it toggles the "state" every time the button is pressed.
 
  The SysCtlDelay of about 0.25s is a rude way to avoid bouncing. The button being mechanical 
  doesn't give a perfect 1 and 0 signal, instead, to the "eyes" of the digital pin, it bounces
  between 1 and 0 when you press it before stabilizing
*/
  uint32_t value=0;
  uint8_t state=0;
  while(1){
    value= GPIOPinRead(ButtonBase,Button);
    if( value==0)
      state^=RED_LED;

    GPIOPinWrite(LED_BASE,RED_LED, state);
    SysCtlDelay(7000000);
  }
   
}
