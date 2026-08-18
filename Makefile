PROJECT := flowcan-rtos
BUILD := build
TOOLCHAIN ?= /Users/lyhenry/.ardupilot-tools/gcc-arm-none-eabi-10-2020-q4-major/bin
CC := $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY := $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE := $(TOOLCHAIN)/arm-none-eabi-size
READELF := $(TOOLCHAIN)/arm-none-eabi-readelf
OBJDUMP := $(TOOLCHAIN)/arm-none-eabi-objdump
NM := $(TOOLCHAIN)/arm-none-eabi-nm
HOST_CC ?= cc

INCLUDES := -Iconfig -Iinclude -Iapp -Idrivers -Iplatform -Iprotocol \
 -Ithird_party/freertos/include -Ithird_party/freertos/portable/GCC/ARM_CM3 \
 -Ithird_party/libcanard -Ithird_party/st-vl53l1x/core -Ithird_party/st-vl53l1x/platform \
 -Ithird_party/dronecan/generated/include \
 -Ithird_party/artery-at32f415/CMSIS/cm4/core_support \
 -Ithird_party/artery-at32f415/CMSIS/cm4/device_support \
 -Ithird_party/artery-at32f415/drivers/inc

ARCH := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
COMMON_WARN := -Wall -Wextra -Werror -Wshadow -Wdouble-promotion -Wformat=2 -Wundef
CFLAGS := $(ARCH) -std=c11 -Os -ffunction-sections -fdata-sections -fno-common -fno-builtin \
 $(COMMON_WARN) $(INCLUDES) -DAT32F415KBU7_4 -DHEXT_VALUE=8000000UL
LDFLAGS := $(ARCH) -Tlinker/at32f415kb_flash.ld -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map \
 --specs=nano.specs --specs=nosys.specs -Wl,--cref

BSP := third_party/artery-at32f415/drivers/src
GEN := third_party/dronecan/generated/src
SOURCES := app/main.c app/rtos_app.c app/supervisor.c app/activity.c app/status_led.c app/scheduler.c app/can_recovery.c \
 drivers/pmw3901.c drivers/vl53l1x.c protocol/msp.c protocol/dronecan.c \
 platform/platform.c platform/system_at32f415.c platform/vl53l1_platform.c \
 third_party/st-vl53l1x/core/VL53L1X_api.c third_party/libcanard/canard.c \
 third_party/freertos/tasks.c third_party/freertos/queue.c third_party/freertos/list.c \
 third_party/freertos/event_groups.c third_party/freertos/portable/GCC/ARM_CM3/port.c \
 $(BSP)/at32f415_crm.c $(BSP)/at32f415_flash.c $(BSP)/at32f415_gpio.c \
 $(BSP)/at32f415_misc.c $(BSP)/at32f415_dma.c $(BSP)/at32f415_exint.c \
 $(BSP)/at32f415_spi.c $(BSP)/at32f415_i2c.c $(BSP)/at32f415_usart.c \
 $(BSP)/at32f415_can.c $(BSP)/at32f415_tmr.c $(BSP)/at32f415_wdt.c \
 $(GEN)/com.hex.equipment.flow.Measurement.c $(GEN)/uavcan.equipment.range_sensor.Measurement.c \
 $(GEN)/uavcan.CoarseOrientation.c $(GEN)/uavcan.Timestamp.c $(GEN)/uavcan.protocol.NodeStatus.c \
 $(GEN)/uavcan.protocol.GetNodeInfo_req.c $(GEN)/uavcan.protocol.GetNodeInfo_res.c \
 $(GEN)/uavcan.protocol.HardwareVersion.c $(GEN)/uavcan.protocol.SoftwareVersion.c
OBJECTS := $(addprefix $(BUILD)/,$(SOURCES:.c=.o)) $(BUILD)/startup/startup_at32f415.o

HOST_SOURCES := tests/test_main.c protocol/msp.c protocol/dronecan.c drivers/pmw3901.c drivers/vl53l1x.c app/scheduler.c app/supervisor.c app/activity.c app/can_recovery.c \
 third_party/libcanard/canard.c $(GEN)/com.hex.equipment.flow.Measurement.c \
 $(GEN)/uavcan.equipment.range_sensor.Measurement.c $(GEN)/uavcan.CoarseOrientation.c \
 $(GEN)/uavcan.Timestamp.c $(GEN)/uavcan.protocol.NodeStatus.c $(GEN)/uavcan.protocol.GetNodeInfo_req.c \
 $(GEN)/uavcan.protocol.GetNodeInfo_res.c $(GEN)/uavcan.protocol.HardwareVersion.c $(GEN)/uavcan.protocol.SoftwareVersion.c

.PHONY: all clean test size release flash verify
all: $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).hex $(BUILD)/$(PROJECT).bin size

$(BUILD)/$(PROJECT).elf: $(OBJECTS) linker/at32f415kb_flash.ld
	@mkdir -p $(@D)
	$(CC) $(OBJECTS) $(LDFLAGS) -lm -lc -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/startup/startup_at32f415.o: startup/startup_at32f415.s
	@mkdir -p $(@D)
	$(CC) $(ARCH) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(PROJECT).hex: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@
$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/host_tests: $(HOST_SOURCES)
	@mkdir -p $(@D)
	$(HOST_CC) -std=c11 -O2 $(COMMON_WARN) $(INCLUDES) $(HOST_SOURCES) -lm -o $@

test: $(BUILD)/host_tests
	./$(BUILD)/host_tests

size: $(BUILD)/$(PROJECT).elf
	$(SIZE) $<
	READELF=$(READELF) OBJDUMP=$(OBJDUMP) OBJCOPY=$(OBJCOPY) NM=$(NM) SIZE=$(SIZE) ./tools/check_firmware.sh $< $(BUILD)/$(PROJECT).map

verify: test all
	./tools/verify_tree.sh

release: clean verify
	@mkdir -p $(BUILD)/release
	cp $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).hex $(BUILD)/$(PROJECT).bin $(BUILD)/$(PROJECT).map $(BUILD)/release/
	$(SIZE) $(BUILD)/$(PROJECT).elf > $(BUILD)/release/SIZE.txt
	cd $(BUILD)/release && shasum -a 256 $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin $(PROJECT).map SIZE.txt > SHA256SUMS.txt

flash: $(BUILD)/$(PROJECT).elf
	@command -v openocd >/dev/null || { echo "openocd not installed"; exit 1; }
	@test -f "$(AT32_OPENOCD_TARGET)" || { echo "Set AT32_OPENOCD_TARGET to Artery's at32f415xx.cfg"; exit 1; }
	openocd -f interface/cmsis-dap.cfg -f "$(AT32_OPENOCD_TARGET)" -c "program $< verify reset exit"

clean:
	rm -rf $(BUILD)

-include $(OBJECTS:.o=.d)
