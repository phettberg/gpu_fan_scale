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
#define PWM_DIVIDER 16

static char g_cInput[APP_INPUT_BUF_SIZE];
static uint8_t g_dutyCycle = 15;
static uint32_t g_tachoFrequency = 50;


void ConfigureUART(void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}


void ConfigurePWM(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, (SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY * g_dutyCycle / 100));

    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (SysCtlClockGet() / PWM_DIVIDER / g_tachoFrequency));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (SysCtlClockGet() / PWM_DIVIDER / g_tachoFrequency * 50 / 100));

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
}


void ConfigureTimer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_1);

    GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PF1_T0CCP1);

    IntMasterEnable();

    TimerConfigure(TIMER0_BASE, TIMER_CFG_B_CAP_TIME_UP | TIMER_CFG_SPLIT_PAIR);
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);

    IntEnable(INT_TIMER0B);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);

    TimerEnable(TIMER0_BASE, TIMER_B);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PF4_T2CCP0);

    TimerConfigure(TIMER2_BASE, TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_SPLIT_PAIR);
    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);

    TimerEnable(TIMER2_BASE, TIMER_A);
}


/* Measuring pulse width and period of incoming PWM signal */

uint32_t g_edgeDifference = 0;
uint32_t g_periodClocks = 0;
volatile uint32_t g_dutyCycleAct = 0;

void Timer0BIntHandler(void) {
    static uint32_t risingEdge = 0;
    static uint32_t risingEdgeSecond = 0;
    static uint32_t fallingEdge = 0;

    TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);

    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) {
        risingEdge = TimerValueGet(TIMER0_BASE, TIMER_B);
        if (risingEdgeSecond) {
            g_periodClocks = risingEdge - risingEdgeSecond;
            risingEdgeSecond = 0;
            g_dutyCycleAct = (uint32_t)(g_edgeDifference * 1000.0 / g_periodClocks);
        }
        else {
            risingEdgeSecond = risingEdge;
        }
    }

    else if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) {
        fallingEdge = TimerValueGet(TIMER0_BASE, TIMER_B);
        g_edgeDifference = (fallingEdge - risingEdge)&0xFFFF;

    }
}


/* Measuring period of incoming tacho signal */

uint32_t g_tachoPeriodClocks = 0;

void Timer2AIntHandler(void) {
    static uint32_t risingEdge = 0;
    static uint32_t risingEdgeSecond = 0;

    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)) {
        risingEdge = TimerValueGet(TIMER2_BASE, TIMER_A);
        if (risingEdgeSecond) {
            g_tachoPeriodClocks = risingEdge - risingEdgeSecond;
            risingEdgeSecond = 0;
        }
        else {
            risingEdgeSecond = risingEdge;
        }

    }

    TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);
}


uint32_t getFanDutyCycle(void ) {
    return g_dutyCycleAct;
}


void setFanDutyCycle(uint32_t dutyCycle) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY * dutyCycle / 1000.0));
}


uint32_t getTachoRPM(void) {
    /* The system clock is 1 / 16MHz or 62.5ns
     * A tacho signal emits two pulses per full rotation
     */
    return (uint32_t)(1e9 / (62.5 * g_tachoPeriodClocks) * 60 / 2);
}


void setTachoSignal(uint32_t rpm) {
    /* A tacho signal emits two pulses per full rotation */
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (SysCtlClockGet() / PWM_DIVIDER / (rpm * 2.0 / 60)));

    /* Keep duty cycle at 50% */
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (SysCtlClockGet() / PWM_DIVIDER / (rpm * 2.0 / 60) * 50 / 100));
}



int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_XTAL_16MHZ | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlDelay(1);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ConfigureUART();
    ConfigurePWM();
    ConfigureTimer();

    UARTprintf("PWM Control \n");

    uint32_t timePeriod = 0;
    uint32_t pulseWidth = 0;
    uint32_t tachoPeriod = 0;
    uint32_t tachoRPM = 0;
    uint32_t dutyCycle = 0;

    while(1) {
        UARTprintf("> ");
        UARTgets(g_cInput,sizeof(g_cInput));
        dutyCycle = ustrtoul(g_cInput, 0, 10);
        // tachoRPM = ustrtoul(g_cInput, 0, 10);

        setFanDutyCycle(dutyCycle);
        // setTachoSignal(tachoRPM);

        UARTprintf("Duty Cycle: %d%%\n", dutyCycle);
        // UARTprintf("Set tacho: %drpm\n", tachoRPM);

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

        SysCtlDelay(1000000);

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

        SysCtlDelay(1000000);

        timePeriod = g_periodClocks;// * 62.5;
        pulseWidth = g_edgeDifference;// * 62.5;
        tachoPeriod = g_tachoPeriodClocks * 62.5 / 1000;
        UARTprintf("W: %d P: %d DC: %d\n", pulseWidth, timePeriod, getFanDutyCycle());
        UARTprintf("Clks: %d TP: %d RPM: %d\n", g_tachoPeriodClocks, tachoPeriod, getTachoRPM());
    }
}
