# Makefile pour les tests JVS Controller avec Modelsim/Questasim

# Variables (auto-détection de l'OS)
# Force .exe pour WSL qui utilise les outils Windows
ifeq ($(OS),Windows_NT)
    VSIM = vsim.exe
    VLOG = vlog.exe
    VLIB = vlib.exe
    VMAP = vmap.exe
else ifneq ($(shell uname -r | grep -i microsoft),)
    # WSL détecté - utilise les .exe Windows
    VSIM = vsim.exe
    VLOG = vlog.exe
    VLIB = vlib.exe
    VMAP = vmap.exe
else
    # Linux natif
    VSIM = vsim
    VLOG = vlog
    VLIB = vlib
    VMAP = vmap
endif

# Bibliothèques
WORK_LIB = work

# Sources nécessaires
SOURCES = ../rtl/jvs_node_info_pkg.sv \
          ../jvscom/uart_tx.v \
          ../jvscom/uart_rx.v \
          ../jvs_controller.sv \
          fake_jvs_device.sv

# Sources pour tests avancés avec Smart Device
SMART_SOURCES = ../rtl/jvs_node_info_pkg.sv \
                ../jvscom/uart_tx.v \
                ../jvscom/uart_rx.v \
                ../jvs_controller.sv \
                smart_jvs_device.sv

# Flags de compilation
VLOG_FLAGS = -sv +incdir+../include +incdir+../rtl +incdir+../jvscom +incdir+.. -suppress 2892

# Targets de test
.PHONY: all test_01 test_02 test_03 clean setup

all: setup test_01

# Configuration des bibliothèques
setup:
	@echo "=== Configuration des bibliothèques Modelsim ==="
	$(VLIB) $(WORK_LIB) || true
	$(VMAP) work $(WORK_LIB) || true

test_01: setup
	@echo "=== Compilation du Test 01 avec Fake JVS Device ==="
	$(VLOG) $(VLOG_FLAGS) -work $(WORK_LIB) $(SOURCES) test_01_with_fake_jvs.sv
	@echo "=== Exécution du Test 01 avec décodage UART fiable ==="
	$(VSIM) -c -do "run -all; quit" $(WORK_LIB).test_01_with_fake_jvs

test_02: setup
	@echo "=== Compilation du Test 02: Address Assignment Sequence ==="
	$(VLOG) $(VLOG_FLAGS) -work $(WORK_LIB) $(SOURCES) test_02_address_assignment.sv
	@echo "=== Exécution du Test 02 ==="
	$(VSIM) -c -do "run -all; quit" $(WORK_LIB).test_02_address_assignment

test_03: setup
	@echo "=== Compilation du Test 03: Device Identification with Smart JVS ==="
	$(VLOG) $(VLOG_FLAGS) -work $(WORK_LIB) $(SMART_SOURCES) test_03_device_identification.sv
	@echo "=== Exécution du Test 03 ==="
	$(VSIM) -c -do "run -all; quit" $(WORK_LIB).test_03_device_identification


# Test avec GUI pour debug
test_01_gui: setup
	@echo "=== Test 01 avec interface graphique ==="
	$(VLOG) $(VLOG_FLAGS) -work $(WORK_LIB) $(SOURCES) test_01_with_fake_jvs.sv
	$(VSIM) -gui $(WORK_LIB).test_01_with_fake_jvs

# Test en mode batch avec génération de traces
test_01_wave: setup
	@echo "=== Test 01 avec génération de waveform ==="
	$(VLOG) $(VLOG_FLAGS) -work $(WORK_LIB) $(SOURCES) test_01_with_fake_jvs.sv
	$(VSIM) -c -do "add wave -radix hex /test_01_with_fake_jvs/*; run -all; write format wave -window .main_pane.wave.interior.cs.body.pw.wf test_01.wlf; quit" $(WORK_LIB).test_01_with_fake_jvs

clean:
	rm -rf $(WORK_LIB) transcript *.wlf *.vcd vsim.wlf

# Affichage de l'aide
help:
	@echo "Targets disponibles:"
	@echo "  test_01       - Test de trame reset UART (optimisé 250kHz)"
	@echo "  test_02       - Test de séquence d'assignation d'adresse (optimisé 250kHz)"
	@echo "  test_03       - Test d'identification JVS complète avec Smart Device"
	@echo "  test_01_gui   - Test 01 avec interface graphique Modelsim"
	@echo "  test_01_wave  - Test 01 avec génération de waveform"
	@echo "  setup         - Configuration des bibliothèques"
	@echo "  clean         - Nettoyer les fichiers générés"
