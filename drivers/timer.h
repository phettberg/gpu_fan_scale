#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>


extern void ConfigureTimerSysTick(void);
extern void ConfigurePWMFan(void);
extern void ConfigureTimerPWMTacho(void);
extern void ConfigureTimerMeasurements(void);

extern uint32_t getGPUDutyCycle(void);
extern void setFanDutyCycle(uint32_t dutyCycle);
extern uint32_t getFanTachoRPM(void);
extern void setGPUTachoRPM(uint32_t rpm);

extern void delayMS(uint32_t millis);

#endif
