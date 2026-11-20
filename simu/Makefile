# JVS controller simulation
#
# Targets
#   make smart        tb_smart_device: init, polling, outputs and link recovery against
#                     the synthetic I/O board model (smart_jvs_device.sv)
#   make replay       tb_capture_replay: replay of a real bus capture (REPLAY=...)
#   make regression   smart + replay of every capture in captures/ (used by the CI)
#   make clean
#
# Options
#   SIM=questa|verilator   simulator (default questa; the CI uses verilator)
#   FAST=1|0               JVS_SIM_FAST: shortened startup/reset delays (default 1)
#   REPLAY=captures/x.replay  capture for `make replay` (generated from x.bin or x.jvs)
#   VCD=1                  waveform dump (tb_smart_device only, large file)
#
# A new capture (jvs_dump.bin from the Pocket) just has to be copied to
# captures/<name>.bin: the replay script is generated with ../scripts/jvs_dump.py.

SIM    ?= questa
FAST   ?= 1
REPLAY ?= captures/sega_838-13683B.replay
VCD    ?= 0

RTL_DIR = ..
PKGS    = $(RTL_DIR)/rtl/jvs_defs_pkg.sv $(RTL_DIR)/rtl/jvs_node_info_pkg.sv
CTRL    = $(RTL_DIR)/uart_tx.v $(RTL_DIR)/uart_rx.v $(RTL_DIR)/jvs_com.sv $(RTL_DIR)/jvs_ctrl.sv

SMART_SOURCES  = $(PKGS) $(CTRL) $(RTL_DIR)/jvs_bus_logger.sv smart_jvs_device.sv tb_smart_device.sv
REPLAY_SOURCES = $(PKGS) $(CTRL) replay_jvs_device.sv tb_capture_replay.sv

# Captures: jvs_dump.bin from the Pocket, or .jvs text captures (oscilloscope decode, one
# frame per line). An optional captures/<name>.opts holds jvs_dump.py conversion options
# (e.g. --prepend-resets --drop <host only request>).
CAPTURES = $(sort $(wildcard captures/*.bin) $(wildcard captures/*.jvs))
REPLAYS  = $(addsuffix .replay,$(basename $(CAPTURES)))

DEFINES = $(if $(filter 1,$(FAST)),JVS_SIM_FAST)
RESULT_FILTER = "\[CHECK\]|CHECKS|ALL CHECKS|CHECK\(S\)|TX MISMATCH|script done|\*\* (Error|Fatal)|%Error"

#---------------------------------------------------------------------------
# Simulator specific commands: $(call run_sim,<top>,<sources>,<extra defines>,<log>)
#---------------------------------------------------------------------------
ifeq ($(SIM),verilator)
VERILATOR ?= verilator
VERILATOR_FLAGS = --binary --timing -j 0 -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH \
                  -Wno-MULTIDRIVEN -Wno-INITIALDLY -Wno-UNOPTFLAT --assert \
                  -I$(RTL_DIR) -I$(RTL_DIR)/rtl
define run_sim
	rm -rf obj_$(1)
	$(VERILATOR) $(VERILATOR_FLAGS) --top-module $(1) --Mdir obj_$(1) \
	    $(foreach d,$(DEFINES) $(3),-D$(d)) $(2) > $(4).build 2>&1 || { tail -40 $(4).build; exit 1; }
	./obj_$(1)/V$(1) $(if $(filter 1,$(VCD)),+vcd) > $(4) 2>&1; grep -E $(RESULT_FILTER) $(4); grep -q "ALL CHECKS PASSED" $(4)
endef
else
QUESTA_LICENSE ?= $(HOME)/.altera.quartus/questa_lic.dat
export SALT_LICENSE_SERVER ?= $(QUESTA_LICENSE)
VLIB = vlib
VLOG = vlog
VSIM = vsim
VLOG_FLAGS = -sv -suppress 2892 +incdir+$(RTL_DIR) +incdir+$(RTL_DIR)/rtl
define run_sim
	rm -rf work_$(1)
	$(VLIB) work_$(1) > /dev/null
	$(VLOG) $(VLOG_FLAGS) -work work_$(1) $(foreach d,$(DEFINES) $(3),+define+$(d)) $(2) > $(4).build 2>&1 || { grep -E "Error" $(4).build; exit 1; }
	$(VSIM) -c -lib work_$(1) $(if $(filter 1,$(VCD)),+vcd) -do "run -all; quit -f" $(1) 2>&1 \
	    | grep -v -E "^# \[(JVS_COM|SMART_JVS|FIFO|RS485|CONTROLLER|TEST)\]|^# \[[0-9]+\] JVS_COM" > $(4); \
	grep -E $(RESULT_FILTER) $(4); grep -q "ALL CHECKS PASSED" $(4)
endef
endif

.PHONY: all smart replay replays regression clean help
all: regression

smart:
	@echo "=== tb_smart_device ($(SIM), FAST=$(FAST)) ==="
	$(call run_sim,tb_smart_device,$(SMART_SOURCES),,smart.log)

replay: $(REPLAY)
	@echo "=== tb_capture_replay $(REPLAY) ($(SIM)) ==="
	$(call run_sim,tb_capture_replay,$(REPLAY_SOURCES),REPLAY_FILE=\"$(REPLAY)\",replay_$(notdir $(basename $(REPLAY))).log)

# smart device test, then every capture of captures/ (stops on the first failure)
regression: $(REPLAYS)
	@$(MAKE) --no-print-directory smart
	@for r in $(REPLAYS); do $(MAKE) --no-print-directory replay REPLAY=$$r || exit 1; done
	@echo "*** REGRESSION PASSED ($(SIM)): smart device + $(words $(REPLAYS)) capture(s) ***"

captures/%.replay: captures/%.bin ../scripts/jvs_dump.py
	python3 ../scripts/jvs_dump.py $< --replay $@ $(shell cat captures/$*.opts 2>/dev/null)

captures/%.replay: captures/%.jvs ../scripts/jvs_dump.py
	python3 ../scripts/jvs_dump.py $< --replay $@ $(shell cat captures/$*.opts 2>/dev/null)

# replay scripts only (the CI generates them on the host, then runs the simulator in docker)
replays: $(REPLAYS)

clean:
	rm -rf work work_* obj_* transcript *.wlf *.vcd *.log *.build sim_jvs_dump.hex

help:
	@sed -n '1,20p' Makefile
