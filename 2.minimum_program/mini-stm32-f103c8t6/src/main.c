

#include "stm32f103xb.h"

#define __NOP  __nop


static void gpio_init_reg(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    GPIOB->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOB->CRL |= GPIO_CRL_MODE2_1; /* Output mode, max speed 2 MHz */

    /* Start with pin low */
    GPIOB->BRR = GPIO_BRR_BR2;
}


static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        for (volatile uint32_t i = 0; i < 8000; i++)
        {
            __NOP();
        }
    }
}


void hal__init(void)
{
    FLASH->ACR |= FLASH_ACR_PRFTBE; /* Enable prefetch */

    SCB->AIRCR = (0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                 (0x00000003U << SCB_AIRCR_PRIGROUP_Pos);

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* Disable JTAG-DP and SW-DP */
    AFIO->MAPR &= ~AFIO_MAPR_SWJ_CFG;
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
}


void SystemClock_Config(void)
{
    /* Enable HSE and wait until it is ready */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U)
    {
        /* wait */
    }

    /* Configure Flash prefetch buffer and latency */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    /* Set AHB, APB1 and APB2 prescalers */
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    /* Configure PLL: source HSE, multiplier 9 */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9);

    /* Enable PLL and wait until it is ready */
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
    {
        /* wait */
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        /* wait */
    }
}


int main(void)
{
    hal__init();
    SystemClock_Config();
    gpio_init_reg();

    while (1)
    {
        GPIOB->ODR ^= (1U << 2);  /* PB2 ??(??) */
        delay_ms(500);
    }
}




