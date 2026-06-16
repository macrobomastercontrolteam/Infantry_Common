# ------------------------------------------------------------------------------
# Makefile for the RoboMaster standard_robot firmware (STM32F407IGHx)
# GNU Arm Embedded Toolchain (arm-none-eabi-gcc)
#
# Targets:
#   make            build elf/hex/bin (default DEBUG=1)
#   make DEBUG=0    optimized release build
#   make flash      program the board with ST-LINK (st-flash) -- optional
#   make clean      remove build artifacts
# ------------------------------------------------------------------------------

######################################
# target
######################################
TARGET = standard_robot

######################################
# building variables
######################################
# debug build? (1 = -Og -g, 0 = optimized)
DEBUG ?= 1
# optimization
OPT ?= -O2

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources (mirrors the Keil project file list)
C_SOURCES =  \
Src/main.c \
Src/gpio.c \
Src/freertos.c \
Src/adc.c \
Src/can.c \
Src/crc.c \
Src/dma.c \
Src/i2c.c \
Src/rng.c \
Src/rtc.c \
Src/spi.c \
Src/tim.c \
Src/usart.c \
Src/usb_device.c \
Src/usbd_conf.c \
Src/usbd_desc.c \
Src/usbd_cdc_if.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Src/system_stm32f4xx.c \
bsp/boards/bsp_adc.c \
bsp/boards/bsp_buzzer.c \
bsp/boards/bsp_can.c \
bsp/boards/bsp_crc32.c \
bsp/boards/bsp_delay.c \
bsp/boards/bsp_flash.c \
bsp/boards/bsp_imu_pwm.c \
bsp/boards/bsp_i2c.c \
bsp/boards/bsp_led.c \
bsp/boards/bsp_rc.c \
bsp/boards/bsp_rng.c \
bsp/boards/bsp_spi.c \
bsp/boards/bsp_usart.c \
bsp/boards/bsp_servo_pwm.c \
bsp/boards/bsp_laser.c \
bsp/boards/bsp_gpio.c \
application/calibrate_task.c \
application/CAN_receive.c \
application/chassis_behaviour.c \
application/chassis_power_control.c \
application/chassis_task.c \
application/detect_task.c \
application/gimbal_behaviour.c \
application/gimbal_task.c \
application/INS_task.c \
application/oled_task.c \
application/referee.c \
application/referee_usart_task.c \
application/remote_control.c \
application/shoot.c \
application/test_task.c \
application/usb_task.c \
application/voltage_task.c \
application/led_flow_task.c \
application/servo_task.c \
application/cv_usart_task.c \
application/custom_ui_task.c \
application/graphic.c \
application/referee_can_task.c \
application/vofa_task.c \
components/devices/OLED.c \
components/devices/BMI088driver.c \
components/devices/BMI088Middleware.c \
components/devices/ist8310driver.c \
components/devices/ist8310driver_middleware.c \
components/algorithm/AHRS_middleware.c \
components/algorithm/user_lib.c \
components/support/CRC8_CRC16.c \
components/support/fifo.c \
components/support/mem_mang4.c \
components/controller/pid.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s

# Pre-compiled (ARM) libraries shipped with the project. The GNU linker is able
# to consume these EABI archives; the only externals they require are standard
# AEABI/libgcc helpers and the AHRS_* wrappers found in AHRS_middleware.c.
EXTRA_LIBS = \
components/algorithm/AHRS.lib \
components/algorithm/arm_cortexM4lf_math.lib

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH
# variable (> make GCC_PATH=xxx) or it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS =

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-DARM_MATH_CM4 \
-D__FPU_PRESENT=1U \
-DARM_MATH_MATRIX_CHECK \
-DARM_MATH_ROUNDING \
-D__packed=

# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-Iapplication \
-Ibsp/boards \
-Icomponents/devices \
-Icomponents/algorithm \
-Icomponents/algorithm/Include \
-Icomponents/support \
-Iapplication/protocol \
-Icomponents/controller \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2 -Og
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407IGHx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
# NOTE: AHRS.lib and arm_cortexM4lf_math.lib are precompiled with the ARM Compiler.
# Their EABI build attributes (Tag_CPU_arch) differ from GCC's, so GNU ld rejects
# them with "conflicting CPU architectures". The objects are Cortex-M4 hard-float
# Thumb-2 and ABI-compatible with this build, so --no-warn-mismatch lets ld link them.
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(EXTRA_LIBS) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--no-warn-mismatch

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash (optional, requires stlink: https://github.com/stlink-org/stlink)
#######################################
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

#######################################
# clean up
#######################################
clean:
	-rmdir /S /Q $(BUILD_DIR) 2>nul || rm -rf $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
