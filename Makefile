PROJECT := f2c23t_hello
VERSION ?= v2026.07.1
BUILD_ROOT ?= build
BUILD ?= $(BUILD_ROOT)
DIST ?= dist
APP_BASE ?= 0x08008000
APP_FLASH_SIZE ?= 0x00038000
FW_STAGE_BASE ?= 0x08040000
HW_TARGET ?= lt-hw4
HW_TARGET_HW40 ?= 0
HW_TARGET_2C53T ?= 0
DMM_UART_BAUD ?= 9600
FPGA_SPI_BR ?= 2
SCOPE_HW_CAPTURE ?= 1
SCOPE_UI_SAFE_STUB ?= 0
SCOPE_ANALOG_CONFIG ?= 1
SCOPE_ATTENUATOR_CONFIG ?= 1
# The CDC shell (live telemetry + fwload over the same cable). ON for 2C53T,
# where the composite USB device is bench-verified; OFF by default, because the
# 2C23T hardware cannot be tested here and an untested USB identity is not
# something to ship blind. Also off in the 2C53T recovery image, which carries
# the 113 KB bitstream and has no room for it.
USB_CDC_SHELL ?= 0
# 2C53T targets opt in; release-2c53t-embed overrides this back to 0.
USB_CDC_SHELL_2C53T ?= 1

CLANG ?= clang
# rust-lld if a Rust toolchain is installed, otherwise any ld.lld on PATH
# (Homebrew llvm provides one). Either links this project identically.
RUST_LLD := $(shell find $(HOME)/.rustup/toolchains -path '*/bin/rust-lld' 2>/dev/null | head -1)
ifeq ($(RUST_LLD),)
RUST_LLD := $(shell command -v ld.lld 2>/dev/null)
endif
LLD_DIR := $(BUILD)/lld

CFLAGS := \
	-target armv7em-none-eabi \
	-mcpu=cortex-m4 \
	-mthumb \
	-ffreestanding \
	-fno-builtin \
	-fdata-sections \
	-ffunction-sections \
	-fno-unwind-tables \
	-fno-asynchronous-unwind-tables \
	-Os \
	-Wall \
	-Wextra \
	-Werror \
	-I src \
	-DAPP_BASE_ADDR=$(APP_BASE)u \
	-DAPP_FLASH_SIZE_BYTES=$(APP_FLASH_SIZE)u \
	-DFW_STAGE_BASE_ADDR=$(FW_STAGE_BASE)u \
	-DHW_TARGET_HW40=$(HW_TARGET_HW40) \
	-DHW_TARGET_2C53T=$(HW_TARGET_2C53T) \
	-DDMM_UART_BAUD=$(DMM_UART_BAUD) \
	-DFPGA_SPI_BR=$(FPGA_SPI_BR) \
	-DSCOPE_HW_CAPTURE=$(SCOPE_HW_CAPTURE) \
	-DSCOPE_UI_SAFE_STUB=$(SCOPE_UI_SAFE_STUB) \
	-DSCOPE_ANALOG_CONFIG=$(SCOPE_ANALOG_CONFIG) \
	-DSCOPE_ATTENUATOR_CONFIG=$(SCOPE_ATTENUATOR_CONFIG) \
	-DFW_VERSION_TEXT='"$(VERSION)"' \
	-DUSB_CDC_SHELL=$(USB_CDC_SHELL) \
	$(EXTRA_CFLAGS)

LDFLAGS := \
	-target armv7em-none-eabi \
	-mcpu=cortex-m4 \
	-mthumb \
	-nostdlib \
	-fuse-ld=lld \
	-B$(LLD_DIR) \
	-Wl,-T,$(BUILD)/linker.ld \
	-Wl,--gc-sections \
	-Wl,-Map,$(BUILD)/$(PROJECT).map

SRCS := src/startup.c src/board.c src/display.c src/font.c src/dmm.c src/dmm53.c src/meter_data.c src/settings.c src/fpga.c src/scope.c src/siggen.c src/fw_update.c src/fw_cache.c src/screenshot.c src/dbgdump.c src/w25q.c src/usb_msc.c src/usb_cdc.c src/cdc_shell.c src/meter_plan.c src/ui.c src/main.c src/fft.c src/arb_csv.c
ifeq ($(HW_TARGET_HW40),1)
SRCS += src/fpga_bitstream_hw4.c
else
SRCS += src/fpga_bitstream.c
endif
OBJS := $(patsubst src/%.c,$(BUILD)/%.o,$(SRCS))

.PHONY: all clean clean-dist release release-lt-hw4 release-hw4

all: $(BUILD)/$(PROJECT).bin

$(LLD_DIR)/ld.lld:
	@test -n "$(RUST_LLD)" || (echo "rust-lld not found; install an ARM linker or Rust toolchain with rust-lld" >&2; exit 1)
	@mkdir -p $(LLD_DIR)
	@ln -sf "$(RUST_LLD)" $(LLD_DIR)/ld.lld

$(BUILD)/linker.ld: linker.ld
	@mkdir -p $(BUILD)
	sed \
		-e 's/ORIGIN = 0x08008000/ORIGIN = $(APP_BASE)/' \
		-e 's/LENGTH = 224K/LENGTH = $(APP_FLASH_SIZE)/' \
		$< > $@

$(BUILD)/%.o: src/%.c
	@mkdir -p $(BUILD)
	$(CLANG) $(CFLAGS) -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(OBJS) $(BUILD)/linker.ld $(LLD_DIR)/ld.lld
	$(CLANG) $(LDFLAGS) $(OBJS) -o $@

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf tools/elf2bin.py
	python3 tools/elf2bin.py $< $@ $(APP_BASE)

clean:
	rm -rf $(BUILD_ROOT)

clean-dist:
	rm -rf $(DIST)

release: clean-dist
	$(MAKE) release-lt-hw4
	$(MAKE) release-hw4

release-lt-hw4:
	$(MAKE) BUILD=$(BUILD_ROOT)/lt-hw4 APP_BASE=0x08008000 HW_TARGET=lt-hw4 HW_TARGET_HW40=0 all
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/lt-hw4/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-08008000.bin

release-hw4:
	$(MAKE) BUILD=$(BUILD_ROOT)/hw4.0 APP_BASE=0x08007000 HW_TARGET=hw4.0 HW_TARGET_HW40=1 all
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/hw4.0/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-HW4.0-08007000.bin

# Experimental 2C53T port: 2C53T button matrix / power / backlight,
# FPGA stubbed (no scope trace), linked for the stock 2C53T IAP bootloader.
# NOTE: this target carries NO FPGA flags, so it builds a meter-only image of
# ~106-110 KB. For anything involving a trace use release-2c53t-scope below;
# an image under ~220 KB has no bitstream in it and the scope is dead.
# APP_FLASH_SIZE: the app slot runs to the bitstream store (0x08007000..
# 0x080C0000). The old 0x38000 ceiling was our own staging base, and staging
# is gone on this target - docs/plans/drop-internal-staging-2026-08-22.md
# in the workspace.
release-2c53t:
	$(MAKE) BUILD=$(BUILD_ROOT)/2c53t APP_BASE=0x08007000 APP_FLASH_SIZE=0x000B9000 USB_CDC_SHELL=$(USB_CDC_SHELL_2C53T) HW_TARGET=2c53t HW_TARGET_HW40=1 HW_TARGET_2C53T=1 SCOPE_HW_CAPTURE=1 SCOPE_ANALOG_CONFIG=0 SCOPE_ATTENUATOR_CONFIG=0 EXTRA_CFLAGS="-Wno-unused-variable -Wno-unused-function -Wno-unused-parameter -Wno-unused-but-set-variable $(SCOPE53_EXTRA)" all
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-08007000.bin

# The bench-proven scope configuration (cold boot -> live dual-channel trace,
# 2026-08-13/14): V0.4 bit-banged config entry, the five arm writes at /256
# after a 600 ms settle, PC0 treated as active-low, stock-paced readout, the
# analog scope pose and the TMR13 CH2 trigger reference. Expect 223-227 KB.
SCOPE53_FLAGS := \
	-DFPGA53_V04_CONFIG=1 \
	-DFPGA53_SEND_CFG=1 \
	-DFPGA53_SEND_CFG_DELAY_MS=600 \
	-DFPGA53_PC0_READY_LOW=1 \
	-DFPGA53_READ_PACED=1 \
	-DFPGA53_FE_SCOPE_POSE=1 \
	-DFPGA53_TMR13_REF=1

# Variant targets wipe build/2c53t first: object files carry no record of the
# -D flags they were built with, so switching variants without a clean silently
# links a mix of two firmwares.
release-2c53t-scope:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-SCOPE-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-SCOPE-08007000.bin

# The bitstream store file. Drop it on the device's USB volume once (same
# gesture as a firmware update — the device routes it by its GWBS header, not
# by name) and every later scope image can leave the 113 KB payload out.
BITSTREAM_BIN := $(DIST)/F2C23T-FPGA-BITSTREAM.BIN
bitstream-bin:
	@mkdir -p $(DIST)
	python3 tools/mkbitstream.py src/fpga_bitstream_2c53t.h $(BITSTREAM_BIN)

# Provisioning / recovery image: the old arrangement with the bitstream inside
# the image. Needed on a unit whose store has never been written — and as the
# way back if a store ever turns out to be bad.
release-2c53t-embed:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t USB_CDC_SHELL_2C53T=0 SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_BITSTREAM_EXTERN=0 $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-EMBED-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-EMBED-08007000.bin

# The CH2 relay ladder: hold each row of the stock table for a dwell and record
# what CH2's window swings by under it (RSW table in DBG.TXT). Needs a steady
# signal on the CH2 probe — 50 kHz is fine, the envelope does not care — and
# takes about ten seconds, after which the bank parks back on its normal row.
release-2c53t-rsweep:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_RELAY_SWEEP=1 $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-RSWEEP-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-RSWEEP-08007000.bin

# Stock's op 0x09 / 0x0A pair, read twice with the engine armed (OP0A lines in
# DBG.TXT). Upstream reads a stable 0x0089 on their unit and does not know what
# it is; a second unit's value is what tells a design constant from something
# per-device. Otherwise an ordinary scope build - no signal needed, but the
# usual honest cold boot is, since the read happens right after the arm.
release-2c53t-op0a:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_OP0A_PROBE=1 $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-OP0A-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-OP0A-08007000.bin

# PB11 held LOW through config and the arm writes, where every other build
# holds it HIGH like stock. This is the discriminator for issue #18: on this
# bench an already-armed capture keeps its data with PB11 LOW, on upstream's
# the part will not arm without it — and both can be true if the pin only
# matters before the pose runs. Needs an honest COLD boot (a warm reboot skips
# configuration and answers nothing) and a signal on the probe; read `A=` for
# DONE_FINAL, `PB11 b11_arm=` for the level config and arm actually saw, and
# C1/C2 for whether frames carry data.
release-2c53t-pb11low:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_PB11_LOW_AT_CONFIG=1 $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-PB11LOW-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-PB11LOW-08007000.bin

# Same, plus the per-frame seam record (SEAM table in DBG.TXT). Feed it a
# periodic signal with several periods in the window — 50 kHz gives ~10 — and
# read the table: r2 against the seam's position, one row per frame.
release-2c53t-seam:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_SEAM_LOG=1 $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-SEAM-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-SEAM-08007000.bin

# The timebase-index sweep in the form stock actually sends it: the two-byte
# command `01 <idx>` over indices 0x00-0x13, results in DBG.TXT under TSW.
# The 2026-08-15 sweep sent the index as a bare byte with no command in front
# and measured a no-op. Feed a periodic signal with several periods in the
# window (50 kHz gives ten) and keep a fast timebase.
TB01_DWELL ?= 16
release-2c53t-tb01:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_SWEEP_TB01=1 -DFPGA53_SWEEP_TIMING_DWELL=$(TB01_DWELL) $(SCOPE53_EXTRA)"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-TB01-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-TB01-08007000.bin

# Same, plus the timing-register falsification sweep (0x0F/0x10/0x11 ladder,
# results in DBG.TXT under TSW). Feed it a periodic signal before reading.
release-2c53t-tsweep:
	rm -rf $(BUILD_ROOT)/2c53t
	$(MAKE) release-2c53t SCOPE53_EXTRA="$(SCOPE53_FLAGS) -DFPGA53_SWEEP_TIMING=1"
	@mkdir -p $(DIST)
	cp $(BUILD_ROOT)/2c53t/$(PROJECT).bin $(DIST)/F2C23T-$(VERSION)-2C53T-TSWEEP-08007000.bin
	@ls -l $(DIST)/F2C23T-$(VERSION)-2C53T-TSWEEP-08007000.bin
