#ifndef BASE_FUNCTIONS_H
#define BASE_FUNCTIONS_H
#include "STM32F407xx.h"
#include "PB_LCD_Drivers.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Config
void INIT(void);

// Input
uint32_t ADCVoltageReading(void);

// Output
void ToggleChargeDischarge(void);
void SetLCDMode(uint32_t mode);
void SetCapacitorValue(uint32_t value);
void SetInductorValue(uint32_t value);
void SetResistorValue(uint32_t value);

// Process
void Delay_us(uint32_t time_us);
void Delay_ms(uint32_t time_ms);
uint32_t findTimeConstant(void);
#endif
