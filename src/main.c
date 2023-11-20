#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g474xx.h"
#include "FreeRTOS.h"
#include "task.h"

void Error_Handler(void) {}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_8);
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {

    }
    LL_RCC_HSI_SetCalibTrimming(64);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {

    }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

    }
    /* Insure 1µs transition state at intermediate medium speed clock based on DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < 100);
    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
    LL_SetSystemCoreClock(170000000);
    LL_Init1msTick(170000000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
}

_Noreturn void heartbeat_task(void *argument) {
    (void) argument;
    for (;;) {
        vTaskDelay(500);
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
        vTaskDelay(500);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
    }
}

int main(void) {
    LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_DisableDeadBatteryPD();
    NVIC_SetPriorityGrouping(3);

    SystemClock_Config();

    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_GPIO_InitTypeDef gpio_init = {
            .Pin = LL_GPIO_PIN_1,
            .Mode = LL_GPIO_MODE_OUTPUT,
            .Speed = LL_GPIO_SPEED_FAST,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
            .Pull = LL_GPIO_PULL_NO
    };

    LL_GPIO_Init(GPIOB, &gpio_init);


    xTaskCreate(heartbeat_task, "heartbeat", 100, NULL, 2, NULL);
    vTaskStartScheduler();
    while (1) {}
}
