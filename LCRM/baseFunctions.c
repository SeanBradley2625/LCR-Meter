#include "STM32F407xx.h"
#include "PB_LCD_Drivers.h"
#include <stdio.h>
#include <string.h>

static char LCDLine0[16];
static char LCDLine1[16];
static char Line0[16];
static char Line1[16];

uint32_t convertedData;
uint32_t LCDValueToPrint;
uint32_t CDState = 1;
uint32_t LCDMode = 3;
uint32_t CapacitorValue = 0;
uint32_t InductorValue = 0;
uint32_t ResistorValue = 0;
uint32_t dischargeTime = 0;

void INIT(void){
	// LCD Initialisation
	PB_LCD_Init();
	PB_LCD_Clear();

	// init system and update the clock, find a better description lol
	SystemInit();
	SystemCoreClockUpdate();

	// Clock init
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIO port A enable
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // ADC 1 enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;  // DMA 2 enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // Timer 4 enable for creating Delay
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Timer 3 enable for ADC and DMA
	
	// SysTick config
	SysTick_Config(SystemCoreClock/(SystemCoreClock/4000000));
	
	// Pin config
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE1_Msk) | (0x3 << GPIO_MODER_MODE1_Pos); // PA1 adc
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0_Msk) | (0x1 << GPIO_MODER_MODE0_Pos); // PA0 c/d output
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE3_Msk) | (0x0 << GPIO_MODER_MODE3_Pos); // PA3 c/d input high impedance
	
	// DI/O config
	// PA0
	GPIOA->OTYPER = (GPIOA->OTYPER & ~GPIO_OTYPER_OT0_Msk) | (0x0 << GPIO_OTYPER_OT0_Pos);
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED0_Msk) | (0x3 << GPIO_OSPEEDR_OSPEED0_Pos);
	GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD0_Msk) | (0x0 << GPIO_PUPDR_PUPD0_Pos);
	GPIOA->BSRR |= (0x1 << GPIO_BSRR_BS0_Pos); // PA0 c/d high 
	// PA3
	GPIOA->OTYPER = (GPIOA->OTYPER & ~GPIO_OTYPER_OT3_Msk) | (0x0 << GPIO_OTYPER_OT3_Pos);
	GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED3_Msk) | (0x3 << GPIO_OSPEEDR_OSPEED3_Pos);
	GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD3_Msk) | (0x0 << GPIO_PUPDR_PUPD3_Pos);
	
	// ADC config and init
	// ADC prescaler for clock
	ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE_Msk) | (0x0 << ADC_CCR_ADCPRE_Pos);
	// Set the ADC into scan mode:
	ADC->CCR = (ADC->CCR & ~ADC_CR1_SCAN_Msk) | (0x1 << ADC_CR1_SCAN_Pos);
	// set the ADC resolution to max
	ADC1->CR1 = (ADC1->CR1 & ~ADC_CR1_RES_Msk) | (0x0 << ADC_CR1_RES_Pos);
	// Select the ADC to be triggered by an external trigger from the timer,
	// and to send signals to the DMA controller when data is available:
	ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_EXTEN_Msk) | (0x1 << ADC_CR2_EXTEN_Pos);
	ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_EXTSEL_Msk) | (0x8 << ADC_CR2_EXTSEL_Pos);
	ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_DMA_Msk) | (0x1 << ADC_CR2_DMA_Pos);
	// Set the sequence to length two, and the inputs to come from channels
	// one and two (in that order):
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L_Msk) | (0x1 << ADC_SQR1_L_Pos);
	ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1_Msk) | (0x1 << ADC_SQR3_SQ1_Pos);
	ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP1_Msk) | (0x7 << ADC_SMPR2_SMP1_Pos); 
	// Enable the ADC:
	ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_ADON_Msk) | (0x1 << ADC_CR2_ADON_Pos);

	// DMA for ADC config and init ***TAKE SOME TIME TO UNDERSTAND THE DMA REGISTERS!***
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;  // Set peripheral address
	DMA2_Stream0->M0AR = (uint32_t) &convertedData;  // Set memory address
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PFCTRL_Msk) | (0x1 << DMA_SxCR_PFCTRL_Pos);  // Make the DMA run forever
	// Set channel on DMA stream to channel 0 (for ADC):
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_CHSEL_Msk) | (0x0 << DMA_SxCR_CHSEL_Pos);
	// Set DMA priority to very high:
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PL_Msk) | (0x3 << DMA_SxCR_PL_Pos);
	// Set DMA to not increment memory address and not peripheral address:
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PINC_Msk); //peripheral
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_MINC_Msk); //memory
	// Enable circular mode so the DMA controller takes one set of readings then stops:
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_CIRC_Msk) | (0x1 << DMA_SxCR_CIRC_Pos); //enabled
	// Set DMA data direction to peripheral -> memory:
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_DIR_Msk) | (0x0 << DMA_SxCR_DIR_Pos);
	// Set the size of transfers to 32-bit:
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_MSIZE_Msk) | (0x2 << DMA_SxCR_MSIZE_Pos);
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PSIZE_Msk) | (0x2 << DMA_SxCR_PSIZE_Pos);
	// Enable the DMA controller:
	DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_EN_Msk) | (0x1 << DMA_SxCR_EN_Pos);
	
	// Timer 3 config and init. ADC
	TIM3->PSC = 0x0;      // Set pre-scale rate
	TIM3->ARR = 0xffff;   // Set timer frequency
	TIM3->CR2 = (TIM3->CR2 & ~TIM_CR2_MMS_Msk) | (0x2 << TIM_CR2_MMS_Pos); // Master mode selection, update mode.
	TIM3->CR1 |= 0x1 << TIM_CR1_CEN_Pos;   // enable counter
	while (!(TIM3->SR & (1<<0)));	// wait for timer to start

	// Timer 4 config and init. Delay 
	TIM4->PSC = 16; 
	TIM4->ARR = 0xffff; //was 2000
	TIM4->CR1 |= 0x1 << TIM_CR1_CEN_Pos;
	while (!(TIM4->SR & (1<<0)));

	// Timer 2 config and init. Used to trigger the interrupt for the LCD
//	TIM2->PSC = 0xffff;
//	TIM2->ARR = 0xffff;
//	TIM2->DIER |= TIM_DIER_UIE; // update interrupt enable
//	TIM2->CR1 |= 0x1 << TIM_CR1_CEN_Pos;
//	while (!(TIM2->SR & (1<<0)));
	
	// ISR config and init
//	NVIC_SetPriority(TIM2_IRQn, 1);
//	NVIC_EnableIRQ(TIM2_IRQn);

// Went back to systick, I was having too many issues with the tim2 irq service running really fast for no apparent reason
}

uint32_t ADCVoltageReading(void){
	return ((1000000*convertedData)/4096)*3.3;
}

// switched from SysTick_Handler to TIM2_IRQHandler because I thought this would stop the timers getting upset with the delay. It did not work.
void SysTick_Handler(void){
	switch (LCDMode){
		case 0:
			strcpy(Line0, "Capacitor");
			strcpy(Line1, " nF");
			LCDValueToPrint = CapacitorValue;
			break;
		case 1:
			strcpy(Line0, "Inductor");
			strcpy(Line1, " H");
			LCDValueToPrint = InductorValue;
			break;
		case 2:
			strcpy(Line0, "Resistor");
			strcpy(Line1, " Ohms");
			LCDValueToPrint = ResistorValue;
			break;
		case 3:
			strcpy(Line0, "Voltage");
			strcpy(Line1, " uV");
			LCDValueToPrint = ADCVoltageReading();
			break;
		default:
			strcpy(Line0, "MODE ERROR:");
			strcpy(Line1, " N/A");
			LCDValueToPrint = 0;
			break;
	}
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
	snprintf(LCDLine0, 16, "%.16s", Line0);
	PB_LCD_WriteString(LCDLine0, 16);
	PB_LCD_GoToXY(0,1);
	snprintf(LCDLine1, 16, "%u%.5s", LCDValueToPrint, Line1);
	PB_LCD_WriteString(LCDLine1, 16);
}

void ToggleChargeDischarge(void){
	if (!CDState){ // charge (state 1)
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0_Msk) | (0x1 << GPIO_MODER_MODE0_Pos); // make PA0 output
	GPIOA->BSRR |= (0x1 << GPIO_BSRR_BS0_Pos); // set PA0 high
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE3_Msk) | (0x0 << GPIO_MODER_MODE3_Pos);// make PA3 input
	CDState = 1;
	}
	else { // discharge (state 0)
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0_Msk) | (0x0 << GPIO_MODER_MODE0_Pos);// make PA0 input
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE3_Msk) | (0x1 << GPIO_MODER_MODE3_Pos); // make PA3 output
	GPIOA->BSRR |= (0x1 << GPIO_BSRR_BR3_Pos); // set PA3 low
	CDState = 0;
	}
}

void Delay_us(uint32_t time_us){
	TIM4->CNT = 0;
	while (TIM4->CNT < time_us);
}

void Delay_ms(uint32_t time_ms){
	for (uint32_t i=0; i<time_ms; i++){
		Delay_us(1000);
	}
}


void SetCapacitorValue(uint32_t value){
	CapacitorValue = value;
}
void SetInductorValue(uint32_t value){
	InductorValue = value;
}
void SetResistorValue(uint32_t value){
	ResistorValue = value;
}
void SetLCDMode(uint32_t mode){
	LCDMode = mode;
}

uint32_t findTimeConstant(void){
	dischargeTime = 0;
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0_Msk) | (0x1 << GPIO_MODER_MODE0_Pos); // make PA0 output
	GPIOA->BSRR |= (0x1 << GPIO_BSRR_BS0_Pos); // set PA0 high
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE3_Msk) | (0x0 << GPIO_MODER_MODE3_Pos);// make PA3 input
	Delay_ms(100);
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE0_Msk) | (0x0 << GPIO_MODER_MODE0_Pos);// make PA0 input
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE3_Msk) | (0x1 << GPIO_MODER_MODE3_Pos); // make PA3 output
	GPIOA->BSRR |= (0x1 << GPIO_BSRR_BR3_Pos); // set PA3 low
	for(int i=0; (convertedData*100)>(4095*37); i++){
//	for(int i=0; (convertedData)>(800); i++){
		Delay_us(1);
		dischargeTime = i;
	}
	CDState = 0;
	return dischargeTime;
}
