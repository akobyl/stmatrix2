#include "stm32g4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"

void NMI_Handler(void) {

}

void HardFault_Handler(void) {
    while (1) {}
}

void MemManage_Handler(void) {
    while (1) {}
}

void BusFault_Handler(void) {
    while (1) {}
}

void UsageFault_Handler(void) {
    while (1) {}
}

void DebugMon_Handler(void) {
}

void SysTick_Handler(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void EXTI2_IRQHandler(void) {

}

void EXTI4_IRQHandler(void) {

}

void EXTI9_5_IRQHandler(void) {

}

void EXTI15_10_IRQHandler(void) {

}
