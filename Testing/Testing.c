#include "STM32F407xx.h"
#include "PB_LCD_Drivers.h"
#include <stdio.h>
#include "Board_Buttons.h"


static char LCDLine0[16];
static char LCDLine1[16];

// Define the total number of conversions to be taken in one cycle:
//#define CONVERSION_SIZE 32

// An array to store the conversions done by the ADC:
//uint32_t convertedData[CONVERSION_SIZE];

uint32_t convertedData;

void SysTick_Handler(void){
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
	snprintf(LCDLine0, 14, "convertedData");
	PB_LCD_WriteString(LCDLine0, 16);
	PB_LCD_GoToXY(0,1);
	snprintf(LCDLine1, 14, "%u", (convertedData));
	PB_LCD_WriteString(LCDLine1, 16);
}

int main (void) {

		// LCD Initialisation
		PB_LCD_Init();
		PB_LCD_Clear();
	
		SystemCoreClockUpdate();
		SysTick_Config(SystemCoreClock/15);

    // This program demonstrates how to get a timer to trigger regular
    // ADC conversions from two inputs, storing their results in a table
    // automatically.  When the table is full, the process stops.
    //
    // The idea is that a timer generates regular triggers for the ADC
    // conversion process, and once the conversions are complete, the
    // ADC signals to the DMA controller that new data is ready.  The
    // DMA controller then moves this data into an array.
    
    // The first thing to do is identify a suitable timer and DMA channel.
    // First the timer.  In the RM0090 Reference Manual, there is a table
    // (Table 68) showing which timer outputs are available to trigger the 
    // ADC.  It is simpler to set up the timer if the TRGO event is used
    // rather than one of the channel events, so I'll select TIM3 for use
    // here; its TRGO output triggers the ADC on input 0x1000.
    //
    // Next, select which DMA stream to use.  DMA1 might be used by the 
    // DAC, so I'll use DMA2 here, and note that ADC1 is available as
    // channel 0 on streams 0 and 4.  I'll choose stream 0 here.
    //
    // Finally, which inputs to use for the ADC?  Channels 1 and 2 for
    // all three ADCs are available on PA1 and PA2 (see Table 7 in the 
    // device datasheet), so I'll use those.
        
    // Enable the clock to GPIOA, ADC1, DMA2 and Timer3:
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Configure pins PA1 to be in analogue mode:
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE1_Msk) | (0x3 << GPIO_MODER_MODE1_Pos);
        
    // Then set-up the timer to go at about one conversion every
    // second (makes things easier for testing this routine in the 
    // debugger), and produce regular trigger output events (for 
    // the ADC) when the counter reaches zero:
    TIM3->PSC = 0x200;      // Set pre-scale rate
    TIM3->ARR = 0x4000;   // Set timer frequency
    TIM3->CR2 = (TIM3->CR2 & ~TIM_CR2_MMS_Msk) | (0x2 << TIM_CR2_MMS_Pos);
    
    // Next set-up the ADC.  The ADC is set into scan mode, so that it can
    // take two readings in a regular sequence, from inputs 1 and 2, after 
    // each trigger signal from the timer.  It sends a DMA request when the 
    // new converted data is ready, which means that the readings from the
    // ADC will end up interleaved in memory:
		
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

    // Next, set-up the DMA controller to transfer data from the waveformData
    // table to the DAC when trigger events occur:      
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;  // Set peripheral address
    DMA2_Stream0->M0AR = (uint32_t) &convertedData;  // Set memory address
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PFCTRL_Msk) | (0x1 << DMA_SxCR_PFCTRL_Pos);  // Make the DMA run forever
    
    // Set channel on DMA stream to channel 0 (for ADC):
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_CHSEL_Msk) | (0x0 << DMA_SxCR_CHSEL_Pos);
    
    // Set DMA priority to very high:
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PL_Msk) | (0x3 << DMA_SxCR_PL_Pos);
    
    // Set DMA to increment memory address but not peripheral address:
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PINC_Msk); //peripheral
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_MINC_Msk); //memory
    
    // Disable circular mode so the DMA controller takes one set of readings then stops:
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_CIRC_Msk) | (0x1 << DMA_SxCR_CIRC_Pos); //enabled
    
    // Set DMA data direction to peripheral -> memory:
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_DIR_Msk) | (0x0 << DMA_SxCR_DIR_Pos);
    
    // Set the size of transfers to 32-bit:
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_MSIZE_Msk) | (0x2 << DMA_SxCR_MSIZE_Pos);
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_PSIZE_Msk) | (0x2 << DMA_SxCR_PSIZE_Pos);
		
    // Enable the DMA controller:
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_EN_Msk) | (0x1 << DMA_SxCR_EN_Pos);
    
    // For testing: fill array with constants so it's obvious when new data is there:
//    for (int q = 0; q < CONVERSION_SIZE; q++) convertedData[q] = 0xffffffff;

    // Then enable the timer to start generating events:
    TIM3->CR1 |= 0x1 << TIM_CR1_CEN_Pos;    

    while (1) {
			
        // Get on with everything else.  The convertedData array should fill up 
        // in the background.  To set it off again, disable the timer, reset the
        // transfer complete flags in the DMA controller, set the DMA's NDTR 
        // register back to CONVERSION_SIZE, re-enable the DMA in the ADC then 
        // re-enable the timer:
//        if (DMA2_Stream0->NDTR == 0x0) {
						
						

            // Disable the timer:
//            TIM3->CR1 &= ~(0x1 << TIM_CR1_CEN_Pos);         

            // For testing: refill the data array with 0xff so it's obvious if new data has arrived:
//            for (int q = 0; q < CONVERSION_SIZE; q++) convertedData[q] = 0xffffffff;
            
            // To reset the DMA transfer from the ADC, set the DMA bit in the CR2 register
            // low, then high again (see section 13.8.1 in the reference manual):
//            ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_DMA_Msk);
//           ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_DMA_Msk) | (0x1 << ADC_CR2_DMA_Pos);          
            
            // Reset the flags in the DMA, and the number of conversions to be done:
//            DMA2->LIFCR = 0x3d;
//            DMA2_Stream0->NDTR = CONVERSION_SIZE;  // Number of transfers to make

            // Re-enable the DMA controller:
//            DMA2_Stream0->CR = (DMA2_Stream0->CR & ~DMA_SxCR_EN_Msk) | (0x1 << DMA_SxCR_EN_Pos);
    
            // Then re-enable the timer to start generating more events:
//            TIM3->CR1 |= 0x1 << TIM_CR1_CEN_Pos;            
//        }
			
    }
}

/*
void SysTick_Handler(void){
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
	snprintf(LCDLine0, 16, "Resistor Value:");
	PB_LCD_WriteString(LCDLine0, 16);
	PB_LCD_GoToXY(0,1);
	snprintf(LCDLine1, 16, "%f", value);
	PB_LCD_WriteString(LCDLine1, 16);
	R = R + 0.01;
	value = (sin(R)*100)+1000;
}
*/

/*
int main (void) {
	// LCD Initialisation
	PB_LCD_Init();
	PB_LCD_Clear();
	// Input Initialisation
	Buttons_Initialize();
	// Delay Initialisation
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/10);
}
*/
/*
static char lcdLineOne[16];
uint32_t printVal = 0;

void SysTick_Handler(void){
	PB_LCD_Clear();
	if(!printVal){
		snprintf(lcdLineOne, 6, "Hello");
		PB_LCD_WriteString(lcdLineOne, 16);
	}
	else{
		snprintf(lcdLineOne, 7, "World!");
		PB_LCD_WriteString(lcdLineOne, 16);
	}
	printVal = 1 - printVal;
}

int main (void) {
	// LCD Initialisation
	PB_LCD_Init();
	PB_LCD_Clear();
	// Delay Initialisation
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/2);
	
	while(1);
	return 0;
}
*/

/*
uint32_t LEDState = 0;

void SysTick_Handler (void) {
	LEDState = 1 - LEDState;
}

int main (void) {
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/2);
	LED_Initialize();

	while (1) {
	if (LEDState) LED_On(0);
	else LED_Off(0);
	}
}
*/

