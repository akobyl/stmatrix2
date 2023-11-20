cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

project(stmatrix2)
set(EXECUTABLE ${PROJECT_NAME}.elf)
set(LINKER_FILE ${CMAKE_SOURCE_DIR}/device/STM32G474RETx_FLASH.ld)
include(FetchContent)

enable_language(C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_ASM_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")

set(SRC_FILES
        src/main.c
        device/startup_stm32g474xx.s
        device/system_stm32g4xx.c
        src/stm32g4xx_it.c
        inc/stm32g4xx_it.h
)

set(SDK_FILES
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_comp.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_cordic.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_crc.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_crs.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dac.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_dma.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_exti.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_fmac.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_gpio.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_hrtim.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_i2c.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_lptim.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_lpuart.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_opamp.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_pwr.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rcc.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rng.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_rtc.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_spi.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_tim.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_ucpd.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usart.c
        STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_utils.c
)

FetchContent_Declare(freertos_kernel
        GIT_REPOSITORY "https://github.com/FreeRTOS/FreeRTOS-Kernel.git"
        GIT_TAG "V10.6.1"
)

add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM
        INTERFACE
        inc
)

target_compile_definitions(freertos_config
        INTERFACE
        projCOVERAGE_TEST=0
)

set(FREERTOS_HEAP "2" CACHE STRING "" FORCE)
# Select the native compile PORT
set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
target_compile_definitions(freertos_config INTERFACE ${definitions})
target_compile_options(freertos_config INTERFACE ${options})

FetchContent_MakeAvailable(freertos_kernel)

add_executable(${EXECUTABLE} ${SRC_FILES} ${SDK_FILES})
target_link_libraries(${EXECUTABLE} PRIVATE freertos_kernel)

target_compile_definitions(${EXECUTABLE} PRIVATE
        -DDEBUG
        -DUSE_FULL_LL_DRIVER
        -DHSE_VALUE=8000000
        -DHSE_STARTUP_TIMEOUT=100
        -DLSE_STARTUP_TIMEOUT=5000
        -DLSE_VALUE=32768
        -DEXTERNAL_CLOCK_VALUE=12288000
        -DHSI_VALUE=16000000
        -DLSI_VALUE=32000
        -DVDD_VALUE=3300
        -DPREFETCH_ENABLE=0
        -DINSTRUCTION_CACHE_ENABLE=1
        -DDATA_CACHE_ENABLE=1
        -DSTM32G474xx
)

target_include_directories(${EXECUTABLE} PRIVATE
        src
        device
        CMSIS/Device/ST/STM32G4xx/Include
        CMSIS/Core/Include
        inc
        STM32G4xx_HAL_Driver/Inc
)

# Compiler options
set(COMPILER_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -fdata-sections
        -ffunction-sections
        -Wall
        -O0
        -g3
)

target_compile_options(${EXECUTABLE} PRIVATE ${COMPILER_FLAGS})
target_compile_options(freertos_config INTERFACE ${COMPILER_FLAGS})

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
        -T${LINKER_FILE}
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -specs=nano.specs
        -lc
        -lm
        -lnosys
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        -Xlinker -print-memory-usage -Xlinker
)

add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} ${EXECUTABLE})