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
#define PWM_FREQUENCY 27000
#define PWM_DIVIDER 1

#define TACHO_FREQUENCY_INIT 25
#define PWM_DUTY_CYCLE_INIT 15

static char g_cInput[APP_INPUT_BUF_SIZE];


void ConfigureUART(void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTStdioConfig(0, 115200, 16000000);
}


void ConfigureTimerSysTick(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / 1000 - 1);

    IntEnable(INT_TIMER3A);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER3_BASE, TIMER_A);
}


volatile uint32_t g_millis = 0;

void Timer3AIntHandler(void) {
    uint32_t status = 0;

    status = TimerIntStatus(TIMER3_BASE, true);
    TimerIntClear(TIMER3_BASE, status);

    g_millis++;
}


void delayMS(uint32_t millis) {
    uint32_t startTime = g_millis;
    while ((g_millis - startTime) < millis);
}


void ConfigurePWMFan(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, (SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY * PWM_DUTY_CYCLE_INIT / 100));

    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}


void ConfigureTimerPWMTacho(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);

    GPIOPinConfigure(GPIO_PC5_WT0CCP1);
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_5);

    TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM);

    TimerLoadSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / TACHO_FREQUENCY_INIT);
    TimerMatchSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / TACHO_FREQUENCY_INIT / 2);

    TimerEnable(WTIMER0_BASE, TIMER_B);
}


void ConfigureTimerMeasurements(void) {
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

volatile uint32_t g_dutyCycleAct = 0;

static uint32_t previousPeriod = 0;
static uint32_t previousPulsewidth = 0;

void Timer0BIntHandler(void) {
    static uint32_t startTime = 0;
    static uint32_t stopTime = 0;
    uint32_t captureRegister = 0;

    TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);

    captureRegister = TimerValueGet(TIMER0_BASE, TIMER_B);

    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)) {
        previousPeriod = (captureRegister - startTime) & 0xFFFFFF;
        previousPulsewidth = (stopTime - startTime) & 0xFFFFFF;
        startTime = captureRegister;
        g_dutyCycleAct = previousPulsewidth * 1000.0 / previousPeriod;
    }
    else {
        stopTime = captureRegister;
    }
}


/* Measuring period of incoming tacho signal */

uint32_t g_tachoPeriodClocks = 0;

void Timer2AIntHandler(void) {
    static uint32_t startTime = 0;
    uint32_t captureRegister = 0;

    TimerIntClear(TIMER2_BASE, TIMER_CAPA_EVENT);

    captureRegister = TimerValueGet(TIMER2_BASE, TIMER_A);

    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)) {
        g_tachoPeriodClocks = (captureRegister - startTime) & 0xFFFFFF;
        startTime = captureRegister;
    }
}


uint32_t getGPUDutyCycle(void ) {
    if (g_dutyCycleAct < 35 || g_dutyCycleAct > 950) return 950;
    return g_dutyCycleAct;
}


void setFanDutyCycle(uint32_t dutyCycle) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY * dutyCycle / 1000.0));
}


uint32_t getFanTachoRPM(void) {
    /* The system clock is 1 / 20MHz or 25ns
     * A tacho signal emits two pulses per full rotation
     */
    uint32_t rpm = (1e9 / (25.0 * g_tachoPeriodClocks) * 60 / 2);
    if (rpm < 500 || rpm > 3800) return 500;
    return rpm;
}


void setGPUTachoRPM(uint32_t rpm) {
    /* A tacho signal emits two pulses per full rotation */
    TimerLoadSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / (rpm * 2.0 / 60) - 1);

    /* Keep duty cycle at 50% */
    TimerMatchSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / ((rpm * 2.0 / 60) - 1) * 50 / 100);
}


void development_pwm(void) {
    uint32_t dutyCycle = 0;

    UARTprintf("> ");
    UARTgets(g_cInput,sizeof(g_cInput));
    dutyCycle = ustrtoul(g_cInput, 0, 10);

    setFanDutyCycle(dutyCycle);

    UARTprintf("SysClk: %d\n", SysCtlClockGet());
    UARTprintf("Duty Cycle: %d%%\n", dutyCycle);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    SysCtlDelay(100000);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    SysCtlDelay(100000);

    for (uint8_t i=0; i<20; i++) {
        UARTprintf("W: %d P: %d DC: %d\n", previousPulsewidth, previousPeriod, getGPUDutyCycle());
        SysCtlDelay(26666);
    }
    UARTprintf("--------------------------");
}


void development_tacho(void) {
    uint32_t tachoRPM = 0;

    UARTprintf("> ");
    UARTgets(g_cInput,sizeof(g_cInput));

    tachoRPM = ustrtoul(g_cInput, 0, 10);

    setGPUTachoRPM(tachoRPM);

    UARTprintf("SysClk: %d\n", SysCtlClockGet());
    UARTprintf("Set tacho: %drpm\n", tachoRPM);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    SysCtlDelay(1000);

    for (uint8_t i=0; i<50; i++) {
        UARTprintf("Clks: %d  CR: RPM: %d\n", g_tachoPeriodClocks, getFanTachoRPM());
        SysCtlDelay(266666);
    }
    UARTprintf("--------------------------");
}


const uint16_t DCtoRPMlookup[13][2] = {
    {282, 1480},
    {290, 1530},
    {330, 1700},
    {376, 1870},
    {420, 2040},
    {469, 2210},
    {517, 2380},
    {573, 2550},
    {634, 2725},
    {695, 2890},
    {761, 3060},
    {829, 3240},
    {910, 3400}
};

uint16_t calcRPMfromDC(uint16_t dc) {
    uint8_t i=0;

    if (dc < DCtoRPMlookup[0][0]) return 1300;
    if (dc > DCtoRPMlookup[12][0]) return 3450;

    for (i=0; i<12; i++) {
        if (dc <= DCtoRPMlookup[i+1][0]) break;
    }
    return DCtoRPMlookup[i][1] + (dc - DCtoRPMlookup[i][0]) *
        (DCtoRPMlookup[i+1][1] - DCtoRPMlookup[i][1]) / (DCtoRPMlookup[i+1][0] - DCtoRPMlookup[i][0]);
}


#define MEAN(name, val, count) (((uint32_t)(name)*(count-1)+(uint32_t)(val))/count)

typedef enum {
    STANDARD = 0, SILENT, FALLBACK
} state_t;

void app(void) {
    /* Duty Cycle is in ‰ */

    static uint8_t greenLED = GPIO_PIN_3;
    static uint32_t measuredDCfromGPU = 0;
    static uint32_t measuredDCfromGPUmean = 0;
    static uint32_t measuredTachofromFan = 0;
    static uint32_t calculatedTachoFromGPU = 0;
    int32_t tachoDeviation = 0;

    static state_t state = STANDARD;

    measuredDCfromGPU = getGPUDutyCycle();
    measuredDCfromGPUmean = MEAN(measuredDCfromGPUmean, measuredDCfromGPU, 8);
    measuredTachofromFan = getFanTachoRPM();
    calculatedTachoFromGPU = calcRPMfromDC(measuredDCfromGPU);
    tachoDeviation = calculatedTachoFromGPU - measuredTachofromFan;

    UARTprintf("%d: ", state);

    switch(state) {
        case STANDARD:
            setFanDutyCycle(measuredDCfromGPUmean);
            if (tachoDeviation > 500) {
                state = FALLBACK;
            }
            else if (measuredDCfromGPU < 310) {
                state = SILENT;
            }
            break;
        case SILENT:
            setFanDutyCycle(150);
            if (measuredTachofromFan < 500) {
                state = FALLBACK;
            }
            else if (measuredDCfromGPU > 350) {
                state = STANDARD;
            }
            break;
        case FALLBACK:
        default:
            if (tachoDeviation < 100 && measuredTachofromFan > 600) {
                state = STANDARD;
            }
            else {
                setFanDutyCycle(900);
            }
            break;
    }

    setGPUTachoRPM(calculatedTachoFromGPU);

    UARTprintf("%d‰ %d‰ %drpm %drpm %d\n", measuredDCfromGPU, measuredDCfromGPUmean,
        measuredTachofromFan, calculatedTachoFromGPU, tachoDeviation);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, greenLED ^= 0xFF);

    delayMS(500);
}


int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlDelay(1);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ConfigureUART();
    ConfigureTimerSysTick();
    ConfigurePWMFan();
    ConfigureTimerPWMTacho();
    ConfigureTimerMeasurements();

    UARTprintf("PWM Control \n");

    while(1) {
        // development_pwm();
        // development_tacho();
        app();
    }
}
