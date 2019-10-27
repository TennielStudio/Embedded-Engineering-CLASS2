#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "uartstdio.h"
#include "driverlib/uart.h"
#include "math.h"
#include "IQmathLib.h"

#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 65.536
#define SAMPLE_RATE 0.01
#define dt 0.01 // 10 ms sample rate!

void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    // Angle around the X-axis
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt;
    // Angle around the Y-axis
    *roll += ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;
    // Compensate for drift with accelerometer data
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
        // Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
        // Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;

    }
}

void ConfigureUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

volatile bool g_bMPU6050Done;

tI2CMInstance g_sI2CMSimpleInst;

void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    if (ui8Status != I2CM_STATUS_SUCCESS) {}
    g_bMPU6050Done = true;
}

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());

}

void I2CMSimpleIntHandler(void)
{
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}

int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    InitI2C0();
    ConfigureUART();

    float pitch, roll, tempPitch, tempRoll;
    float fAccel[3], fGyro[3];
    float xAccel = 0, yAccel = 0, zAccel = 0;
    float xGyro = 0, yGyro = 0, zGyro = 0;
    _iq16 qAccel[3], qGyro[3];

    tMPU6050 sMPU6050;
    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done);
    //
    // Configure the MPU6050 for +/- 4 g accelerometer range.
    //

    //Settings for the Accelerometer
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050,
                           MPU6050_O_ACCEL_CONFIG, // Accelerometer configuration
                           0xFF,//~MPU6050_ACCEL_CONFIG_AFS_SEL_M, // No need to mask
                           MPU6050_ACCEL_CONFIG_AFS_SEL_4G, // Accelerometer full-scale range 4g
                           MPU6050Callback,
                           &sMPU6050);
    while (!g_bMPU6050Done);

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050,
                           MPU6050_O_GYRO_CONFIG, // Gyroscope configuration
                           0xFF, // No need to mask
                           MPU6050_GYRO_CONFIG_FS_SEL_250, // Gyro full-scale range +/- 250 degrees/sec
                           MPU6050Callback,
                           &sMPU6050);
    while (!g_bMPU6050Done);

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050,
                           MPU6050_O_PWR_MGMT_1, // Power management 1 register
                           0x00,
                           0x00,//0x02 & MPU6050_PWR_MGMT_1_DEVICE_RESET,
                           MPU6050Callback,
                           &sMPU6050);
    while (!g_bMPU6050Done);

    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050,
                           MPU6050_O_PWR_MGMT_2, // Power management 2 register
                           0x00,
                           0x00,
                           MPU6050Callback,
                           &sMPU6050);
    while (!g_bMPU6050Done);

    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //
    while (1)
    {
        //
        // Request another reading from the MPU6050.
        //

        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {

        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        // Do something with the new accelerometer and gyroscope readings.
        //
        xAccel = fAccel[0] * 1000;
        yAccel = fAccel[1] * 1000;
        zAccel = fAccel[2] * 1000;
        xGyro = fGyro[0] * 1000;
        yGyro = fGyro[1] * 1000;
        zGyro = fGyro[2] * 1000;

        short fAccelShort[3];
        fAccelShort[0] = (short)fAccel[0];
        fAccelShort[1] = (short)fAccel[1];
        fAccelShort[2] = (short)fAccel[2];

        short fGryoShort[3];
        fGryoShort[0] = (short)fGyro[0];
        fGryoShort[1] = (short)fGyro[1];
        fGryoShort[2] = (short)fGyro[2];

        qAccel[0] = _IQ16(fAccel[0]);
        qAccel[1] = _IQ16(fAccel[1]);
        qAccel[2] = _IQ16(fAccel[2]);

        qGyro[0] = _IQ16(fGyro[0]);
        qGyro[1] = _IQ16(fGyro[1]);
        qGyro[2] = _IQ16(fGyro[2]);


        //ComplementaryFilter(&qAccel[0], &qGyro[0], &pitch, &roll);

        ComplementaryFilter(&fAccelShort[0], &fGryoShort[0], &pitch, &roll);

        tempPitch = atan2f((float)fAccelShort[0], (float)fAccelShort[2]) * 180 / M_PI;
        tempRoll = atan2f((float)fAccelShort[1], (float)fAccelShort[2]) * 180 / M_PI;


        UARTprintf("Acc. X: %d | Acc. Y: %d | Acc. Z: %d\n", (int)xAccel, (int)yAccel, (int)zAccel);
        UARTprintf("Gyro. X: %d | Gyro. Y: %d | Gyro. Z: %d\n", (int)xGyro, (int)yGyro, (int)zGyro);
        //UARTprintf("Pitch: %d ", (int)tempPitch);
        //UARTprintf("Roll:  %d", (int)tempRoll );
        UARTprintf("Pitch: %d ", (int)tempPitch);
        UARTprintf("Roll:  %d", (int)tempRoll);
        UARTprintf("\n");
        //UARTprintf("test\n");
        delayMS(1000);
    }
}


