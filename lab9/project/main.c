#include <stdint.h>
#include <stdbool.h>
#include <math.h> //new include uses the sinf() function
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h" //new include for floating point unit support
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#ifndef M_PI
#define M_PI                    3.14159265358979323846 //pie value
#endif

#define SERIES_LENGTH 1000 //buffer length of 1000 sample points required by task 1

float gSeriesData[SERIES_LENGTH]; //an array of floats SERIES_LENGTH long

int32_t i32DataCount = 0; //counter for the while loop to operate 1000 times
int main(void)
{
    float fRadians_one; //variable that calculates the value of sine
    float fRadians_two;

    //Lazy stacking will avoid an increase of interrupt latency by skipping the stacking of floating point registers.
    ROM_FPULazyStackingEnable(); //enables lazy stacking of floating point registers s0-s15 when an interrupt is handled.

    ROM_FPUEnable(); //turns on floating point unit

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); //50MHz

    fRadians_one = ((2 * M_PI * 50) / SERIES_LENGTH); //full sine wave is 2pi radians. 2pi/series length gives radians
    fRadians_two = ((2 * M_PI * 200) / SERIES_LENGTH); //full sine wave is 2pi radians. 2pi/series length gives radians
    while(i32DataCount < SERIES_LENGTH) //calculates sine value for each of 100 values
    {
        //calculate the equation given in task 1, must use the sine loop calculation (fRadians * i32DataCount) formula for "t"
        //1.5 + 1.0*sin(2pi50t) + 0.5*cos(2pi200t)
        //store the value in a floating point array to display the graph later
        gSeriesData[i32DataCount] = 1.5 + (1.0 * sinf(fRadians_one * i32DataCount)) + (0.5 * cosf(fRadians_two * i32DataCount));
        i32DataCount++; //increment counter for while loop
    }

    while(1) //run infinitely
    {
    }
}
