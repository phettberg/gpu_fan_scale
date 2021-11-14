#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
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
uint8_t g_dutyCycle = 15;
uint32_t g_timerValue = 0;
uint32_t g_risingEdge = 0;
uint32_t g_risingEdgeSecond = 0;
uint32_t g_fallingEdge = 0;
uint32_t g_edgeDifference = 0;
uint32_t g_periodClocks = 0;


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
    SysCtlPWMClockSet(SYSCTL_PWMDIV_32);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (SysCtlClockGet() / 32 / PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (SysCtlClockGet() / 32 / PWM_FREQUENCY * g_dutyCycle / 100));

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}


void ConfigureTimer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_1);

    GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PF1_T0CCP1);

    IntMasterEnable();

    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(TIMER0_BASE, TIMER_B, 255);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_B_CAP_TIME_UP | TIMER_CFG_SPLIT_PAIR);
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);

    IntEnable(INT_TIMER0B);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);

    TimerEnable(TIMER0_BASE, TIMER_B);
}


void Timer0BIntHandler(void) {
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) {
        g_risingEdge = TimerValueGet(TIMER0_BASE, TIMER_B);
        if (g_risingEdgeSecond) {
            g_periodClocks = g_risingEdge - g_risingEdgeSecond;
            g_risingEdgeSecond = 0;
        }
        else {
            g_risingEdgeSecond = g_risingEdge;
        }

    }

    TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);

    if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) {
        g_fallingEdge = TimerValueGet(TIMER0_BASE, TIMER_B);
        g_edgeDifference = g_fallingEdge - g_risingEdge;

    }
    // TimerEnable(TIMER0_BASE, TIMER_B);
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
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ConfigureUART();
    ConfigurePWM();
    ConfigureTimer();

    UARTprintf("PWM Control \n");

    uint32_t timePeriod = 0;
    uint32_t pulseWidth = 0;
    uint32_t dutyCycle = 0;

    while(1) {
        UARTprintf("> ");
        UARTgets(g_cInput,sizeof(g_cInput));
        g_dutyCycle = ustrtoul(g_cInput, 0, 10);

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (SysCtlClockGet() / 32 / PWM_FREQUENCY * g_dutyCycle / 100));
        UARTprintf("Duty Cycle: %d%%\n", g_dutyCycle);

        //
        // Set the GPIO high.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        //
        // Delay for a while.
        //
        SysCtlDelay(1000000);

        //
        // Set the GPIO low.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

        //
        // Delay for a while.
        //
        SysCtlDelay(1000000);

        timePeriod = g_periodClocks * 62.5 / 1000;
        pulseWidth = g_edgeDifference * 62.5 / 1000;
        // dutyCycle = pulseWidth * 100.0 / timePeriod;
        UARTprintf("W: %d P: %d DC: %d\n", pulseWidth, timePeriod, dutyCycle);
    }
}
