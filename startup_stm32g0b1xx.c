#include "system_stm32g0b1xx.h"

extern int main();

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _data_start;
extern uint32_t _data_end;
extern uint32_t _bss_start;
extern uint32_t _bss_end;

/*================== Function Definitions ==================================================*/

void Reset_Handler(void){
    __asm (
        "LDR R0, =_estack\n\t"
        "MOV SP, R0\n\t"
    );

    uint32_t *dataSrc = &_sidata, *dataDest = &_data_start;
    while (dataDest < &_data_end) {
        *dataDest++ = *dataSrc++;
    }

    __asm (
        "LDR R0, =_bss_start\n\t"
        "LDR R1, =_bss_end\n\t"
        "MOV R2, #0\n\t"
        "loop_zero:\n\t"
        "   CMP 	R0, R1\n\t"
        "   BGE 	end_loop\n\t"
        "   STR	    R2, [R0]\n\t"
        "   ADD     R0, R0, #4\n\t"
        "   B 	    loop_zero\n\t"
        "end_loop:\n\t"
    );

    main();
}

void System_Init(void) {

    FLASH->ACR |= FLASH_ACR_LATENCY(2);
    FLASH->ACR |= FLASH_ACR_PRFTEN_MASK;
    while ((FLASH->ACR & FLASH_ACR_LATENCY_MASK) != 2) {
    }

    RCC->CR &= ~RCC_CR_PLLON_MASK;

    while (RCC->CR & RCC_CR_PLLRDY_MASK) {
    }

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC_MASK;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC(2);

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_MASK;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLM(1);

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_MASK;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN(16);

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR_MASK;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLR(1);

    RCC->CR |= RCC_CR_PLLON_MASK;

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN_MASK;

    while (! (RCC->CR & RCC_CR_PLLRDY_MASK)) {
    }

    RCC->CFGR |= RCC_CFGR_SW(2);
}

void NMI_Handler(void) {

    while (1) {

    }
}

void HardFault_Handler(void) {

    while (1) {

    }
}

void SVCall_Handler(void) {

    while (1) {

    }
}

void PendSV_Handler(void) {

    while(1) {

    }
}

void SysTick_Handler(void) {

    while (1) {

    }
}

/*================== Interrupt Vector Table ==============================================*/
void (* g_pfnVectors[])(void) __attribute__((section (".isr_vector"))) = {
    ((void (*)(void)) (&_estack)),

    /*================== Cortex-M0+ Processor Exceptions =================================*/
    Reset_Handler,
    NMI_Handler,                // -14
    HardFault_Handler,          // -13
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    SVCall_Handler,             //  -5
    0,                          // Reserved
    0,                          // Reserved
    PendSV_Handler,             //  -2
    SysTick_Handler,            //  -1

    /*================== STM32G0xxxx Interrupts ===========================================*/
    WWDG_IRQHandler,
    PVD_VDDIO2_IRQHandler,
    RTC_TAMP_IRQHandler,
    FLASH_IRQHandler,
    RCC_CRS_IRQHandler,
    EXTI0_1_IRQHandler,
    EXTI2_3_IRQHandler,
    EXTI4_15_IRQHandler,
    USB_UCPD1_2_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_3_IRQHandler,
    DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler,
    ADC1_COMP_IRQHandler,
    TIM1_BRK_UP_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_TIM4_IRQHandler,
    TIM6_DAC_LPTIM1_IRQHandler,
    TIM7_LPTIM2_IRQHandler,
    TIM14_IRQHandler,
    TIM15_IRQHandler,
    TIM16_FDCAN_IT0_IRQHandler,
    TIM7_LPTIM2_IRQHandler,
    I2C1_IRQHandler,
    I2C2_3_IRQHandler,
    SPI1_IRQHandler,
    SPI2_3_IRQHandler,
    USART1_IRQHandler,
    USART2_LPUART2_IRQHandler,
    USART3_4_5_6_LPUART1_IRQHandler,
    CEC_IRQHandler
};