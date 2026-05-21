obj-m += x8h7_drv.o
obj-m += x8h7_adc.o
obj-m += x8h7_gpio.o
obj-m += x8h7_pwm.o
obj-m += x8h7_rtc.o
obj-m += x8h7_can.o
obj-m += x8h7_uart.o
obj-m += x8h7_ui.o
obj-m += x8h7_h7.o

SRC := $(shell pwd)

# Phase 7.4.35 Plan 01 ccflags hook (carry-forward; re-added in this worktree
# 2026-05-21 to recover Phase 7.4.35 Plan 01 submodule commit aa93f0b3 state
# unreachable in this worktree's local remote — see Phase 7.4.36 Plan 01
# Rule 3 deviation). Default OFF for production; bench/cross-compile builds
# enable via `make CONFIG_X8H7_CAN_DIAG_COUNTERS=1`.
ccflags-y += $(if $(CONFIG_X8H7_CAN_DIAG_COUNTERS),-DCONFIG_X8H7_CAN_DIAG_COUNTERS)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
