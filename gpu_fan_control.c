#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
// #include "inc/tm4c123gh6pm.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#define APP_INPUT_BUF_SIZE 128
#define PWM_FREQUENCY 25000

static char g_cInput[APP_INPUT_BUF_SIZE];
uint8_t duty_cycle = 25;


void ConfigureUART(void) {
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}


void ConfigurePWM(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (SysCtlClockGet() / PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (SysCtlClockGet() / PWM_FREQUENCY * duty_cycle / 100));

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}


int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_XTAL_16MHZ | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN);

    //
    // Enable the GPIO module.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlDelay(1);

    //
    // Configure PA1 as an output.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    ConfigureUART();
    ConfigurePWM();

    UARTprintf("PWM Control \n");
    UARTprintf("> ");

    while(1) {
        UARTprintf("> ");
        UARTgets(g_cInput,sizeof(g_cInput));
        duty_cycle = ustrtoul(g_cInput, 0, 10);
        //
        // Set the GPIO high.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

        //
        // Delay for a while.
        //
        SysCtlDelay(1000000);

        //
        // Set the GPIO low.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

        //
        // Delay for a while.
        //
        SysCtlDelay(1000000);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (SysCtlClockGet() / PWM_FREQUENCY * duty_cycle / 100));
        UARTprintf("Pulse width: %d%%\n", duty_cycle);
    }
}
