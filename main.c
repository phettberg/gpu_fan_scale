#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "utils/ustdlib.h"
#include "drivers/uart.h"
#include "drivers/timer.h"


static char g_cInput[APP_INPUT_BUF_SIZE];


void DevelopmentPWM(void) {
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
        UARTprintf("DC: %d\n", getGPUDutyCycle());
        SysCtlDelay(26666);
    }
    UARTprintf("--------------------------");
}


void DevelopmentTacho(void) {
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
        UARTprintf("RPM: %d\n", getFanTachoRPM());
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
    STANDARD = 0, SILENT, RAMPUP, FALLBACK
} state_t;

void App(void) {
    /* Duty Cycle is in ‰ */

    static uint8_t greenLED = GPIO_PIN_3;
    static uint32_t rampupIterations = 0;
    static uint32_t measuredDCfromGPUmean = 0;
    uint32_t measuredDCfromGPU = 0;
    uint32_t measuredTachofromFan = 0;
    uint32_t calculatedTachoFromGPU = 0;
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
            else if (measuredDCfromGPUmean < 310) {
                state = SILENT;
            }
            break;
        case SILENT:
            setFanDutyCycle(150);
            if (measuredTachofromFan < 500) {
                state = FALLBACK;
            }
            else if (measuredDCfromGPUmean > 350) {
                state = RAMPUP;
            }
            break;
        case RAMPUP:
            setFanDutyCycle(measuredDCfromGPUmean);
            rampupIterations++;
            if (tachoDeviation < 100) {
                rampupIterations = 0;
                state = STANDARD;
            }
            else if (rampupIterations >= 40) {
                rampupIterations = 0;
                state = FALLBACK;
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
    /* 40MHz clock using PLL */
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ConfigureUART();
    ConfigureTimerSysTick();
    ConfigurePWMFan();
    ConfigureTimerPWMTacho();
    ConfigureTimerMeasurements();

    UARTprintf("PWM Control \n");

    while(1) {
        // DevelopmentPWM();
        // DevelopmentTacho();
        App();
    }
}
