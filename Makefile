##############################################################################
# Makefile for the Infantry_Common referee->CAN bridge firmware
# Toolchain: GNU Arm Embedded (arm-none-eabi-gcc)
# Target   : STM32F407IGHx (Cortex-M4F), RoboMaster Type-C dev board
#
# Build:   make            (produces build/standard_robot.elf/.hex/.bin)
# Clean:   make clean
# Flash:   make flash      (uses st-flash; adjust to your programmer)
#
# ---------------------------------------------------------------------------
# IMPORTANT — armcc -> GCC porting prerequisites
# This project was originally built with Keil (ARM Compiler 5 / armcc). Three
# things must be provided before a GCC link will succeed. They are called out
# here rather than silently ignored:
#
#   1. FreeRTOS port: the tree ships only the RVDS (armcc) port at
#      Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c
#      which does NOT assemble under GCC. Drop in the matching GCC port
#      (portable/GCC/ARM_CM4F/port.c + portmacro.h) for your FreeRTOS version
#      and point FREERTOS_PORT_DIR (below) at it.
#
#   2. CMSIS-DSP math: components/algorithm/arm_cortexM4lf_math.lib is an armcc
#      binary and cannot be linked by GCC. Use the CMSIS-DSP GCC archive
#      (libarm_cortexM4lf_math.a) and set MATH_LIB / MATH_LIB_DIR below.
#
#   3. components/algorithm/AHRS.lib is an armcc-only precompiled library with
#      no GCC-compatible equivalent shipped here. It is required by
#      AHRS_middleware.c. You must obtain/rebuild a GCC (.a) version, otherwise
#      the final link will fail with unresolved AHRS_* symbols.
#
#   Source-level note: some files use armcc keywords (e.g. __packed). GCC needs
#   these rewritten (e.g. __attribute__((packed))) or defined away.
##############################################################################

######################################
# target
######################################
TARGET = standard_robot

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og

#######################################
# paths
#######################################
BUILD_DIR = build

######################################
# source
######################################
# C sources (mirrors MDK-ARM/standard_robot.uvprojx after the bridge cleanup)
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
application/calibrate_task.c \
application/CAN_receive.c \
application/detect_task.c \
application/INS_task.c \
application/referee.c \
application/referee_usart_task.c \
application/remote_control.c \
application/usb_task.c \
application/voltage_task.c \
application/led_flow_task.c \
application/custom_ui_task.c \
application/graphic.c \
application/referee_can_task.c \
application/buzzer_task.c \
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
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

# FreeRTOS GCC port (see prerequisite #1). Provide the GCC port then uncomment:
FREERTOS_PORT_DIR = Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
C_SOURCES += $(FREERTOS_PORT_DIR)/port.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in your PATH or set here:
# GCC_PATH =
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
-D__FPU_USED=1U \
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
-Iapplication/protocol \
-Ibsp/boards \
-Icomponents/devices \
-Icomponents/algorithm \
-Icomponents/algorithm/Include \
-Icomponents/support \
-Icomponents/controller \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wno-attributes -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407IGHx_FLASH.ld

# libraries
# CMSIS-DSP GCC archive (see prerequisite #2) and AHRS GCC archive (#3).
MATH_LIB_DIR = components/algorithm
# MATH_LIB = arm_cortexM4lf_math
# AHRS_LIB: the shipped components/algorithm/AHRS.lib is an ARM Compiler 5 (armcc)
# archive but links cleanly with GNU ld (AAPCS/hard-float compatible leaf code).
# NOTE: components/algorithm/arm_cortexM4lf_math.lib (CMSIS-DSP) is NOT usable by
# GNU ld ("conflicting CPU architectures"). The only DSP calls (arm_sin_f32/
# arm_cos_f32 in AHRS_middleware.c) were switched to libm sinf/cosf, so the DSP
# archive is no longer needed. If you need full CMSIS-DSP, add the GCC build
# libarm_cortexM4lf_math.a and re-enable MATH_LIB above.
AHRS_LIB = components/algorithm/AHRS.lib
LIBS = -lc -lm -lnosys
LIBDIR = -L$(MATH_LIB_DIR)
ifdef MATH_LIB
LIBS += -l$(MATH_LIB)
endif
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

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
	$(CC) $(OBJECTS) $(AHRS_LIB) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash (adjust to your programmer)
#######################################
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash
