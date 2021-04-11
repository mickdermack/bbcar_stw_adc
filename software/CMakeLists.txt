cmake_minimum_required (VERSION 2.6)

project(bbcar_stw_adc ASM C CXX)

set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-gcc)

set(MCU "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(COMMON_FLAGS "${MCU} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(LIBS "-lc -lm -lnosys")
set(CMAKE_EXE_LINKER_FLAGS "-specs=nosys.specs -specs=nano.specs -T${CMAKE_SOURCE_DIR}/STM32G431KBUx_FLASH.ld ${LIBS} -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/software.map,--cref")

set(CMAKE_SHARED_LIBRARY_LINK_ASM_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

add_definitions(-DUSE_HAL_DRIVER -DSTM32G431xx)

include_directories(
    Inc
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    STM32CubeG4/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    STM32CubeG4/Drivers/CMSIS/Include
    bobbycar-protocol
)

add_executable(firmware.elf
    Src/main.cpp
    Src/can.c
    Src/can_feedc0de.cpp
    Src/serial_esp.c
    Src/serial_feedc0de.cpp
    Src/stm32g4xx_it.c
    Src/stm32g4xx_hal_msp.c
    Src/system_stm32g4xx.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c
    STM32CubeG4/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c

    bobbycar-protocol/protocol.h

    startup_stm32g431xx.s
)

add_custom_command(OUTPUT firmware.hex
                   COMMAND arm-none-eabi-objcopy -O ihex firmware.elf firmware.hex
                   DEPENDS firmware.elf)

add_custom_command(OUTPUT firmware.bin
                   COMMAND arm-none-eabi-objcopy -O binary -S firmware.elf firmware.bin
                   DEPENDS firmware.elf)

add_custom_target(firmware ALL
                  SOURCES firmware.hex firmware.bin)
